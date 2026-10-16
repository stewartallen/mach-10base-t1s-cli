//! `t1s-cli` — configure a MACH SYSTEMS 10BASET1S-USB-IF dongle over its CDC-ACM control channel.
//!
//! The command surface deliberately mirrors `ethtool`'s PLCA interface, because that is what anyone
//! configuring 10BASE-T1S on Linux already knows.

use std::time::Duration;

use anyhow::{Context as _, Result, bail};
use clap::{Args, Parser as ClapParser, Subcommand, ValueEnum};
use serde_json::json;

use mach_10base_t1s_cli::device::{self, Device};
use mach_10base_t1s_cli::proto::{self, Config, FwVersion, PlcaMode, Role, Status};

/// Configure and diagnose a MACH SYSTEMS 10BASE-T1S USB interface.
#[derive(Debug, ClapParser)]
#[command(name = "t1s-cli", version, about, long_about = None)]
struct Cli {
    /// Serial port of the dongle's control channel.
    ///
    /// Defaults to autodetection; the MACH_T1S_PORT environment variable is used if set.
    #[arg(long, global = true, value_name = "PATH")]
    port: Option<String>,

    /// Per-transaction timeout in milliseconds.
    #[arg(long, global = true, value_name = "MS", default_value_t = 1000)]
    timeout_ms: u64,

    /// Emit machine-readable JSON instead of the human-readable layout.
    #[arg(long, global = true)]
    json: bool,

    #[command(subcommand)]
    command: Command,
}

/// What the user asked for.
///
/// Deliberately limited to what `ethtool`'s PLCA options cover, plus role selection and device
/// identification. The signal-quality, cable-test, test-mode and load messages are implemented in
/// the library but not exposed here: on the hardware tested they return constants that do not
/// track reality, so offering them as commands would only mislead.
#[derive(Debug, Subcommand)]
enum Command {
    /// List serial ports, marking those that look like a MACH T1S control channel.
    ListPorts,

    /// Show serial number, hardware revision, firmware version, MAC address and PHY revision.
    ///
    /// These are the values the control channel reports. The MAC here is the control channel's;
    /// the Ethernet function loads its own from EEPROM and the two have been seen to differ, so
    /// compare against your OS's interface list if it matters.
    Info,

    /// Show the PLCA and role configuration (mirrors `ethtool --get-plca-cfg`).
    ///
    /// Fields left to the DIP switches show the value the switches resolved to, read from status.
    /// That status read clears the read-to-clear diagnostic flags.
    GetPlcaCfg,

    /// Change the PLCA and role configuration (mirrors `ethtool --set-plca-cfg`).
    ///
    /// Only the options you pass are changed; the rest are read from the device and preserved.
    /// Writes take effect on the PHY immediately. A node ID or node count of 0 means "take the
    /// value from the DIP switches". Nothing persists without --save. Every write is verified
    /// against status, and that read clears the read-to-clear diagnostic flags.
    SetPlcaCfg(SetPlcaArgs),

    /// Show whether PLCA is engaged, plus every status and diagnostic flag.
    ///
    /// The flags are read-to-clear: each read reports events since the previous one, so reading here
    /// consumes the events another reader would have seen.
    GetPlcaStatus,

    /// Reset the active configuration to the device defaults (`DefaultDeviceConfiguration`).
    ///
    /// This discards the active configuration, so re-apply your settings afterwards. It leaves
    /// every selector at "switch" with node ID 0 and node count 8 in effect.
    Defaults,

    /// Ask the device to persist the active configuration to its EEPROM.
    ///
    /// Persists the burst count, burst timer and TO timer, which is all the protocol specification
    /// scopes it to; role, PLCA enable, node ID and node count come from the DIP switches at
    /// power-up (measured on firmware 1.2). On firmware 1.1 nothing survived a power cycle (a known
    /// device fault).
    Save,
}

/// Options for `set-plca-cfg`. Every field is optional; omitted fields are left as they are.
#[derive(Debug, Args)]
struct SetPlcaArgs {
    /// Turn PLCA on, off, or defer to the dongle's switch.
    #[arg(long, value_name = "on|off|switch")]
    enable: Option<EnableArg>,

    /// Act as the PLCA coordinator, a follower, or defer to the dongle's switch.
    #[arg(long, value_name = "coordinator|follower|switch")]
    role: Option<RoleArg>,

    /// Local PLCA node ID. 0 takes the value from the DIP switches; the coordinator is node 0.
    #[arg(long, value_name = "0-255")]
    node_id: Option<u8>,

    /// Number of nodes in the PLCA cycle. 0 takes the value from the DIP switches. Meaningful on
    /// the coordinator.
    #[arg(long, value_name = "0-255")]
    node_cnt: Option<u8>,

    /// Transmit-opportunity timer in bit times. The device rejects anything below 29.
    ///
    /// `--to-tmr` is accepted as an alias, since that is how ethtool spells it.
    #[arg(long, visible_alias = "to-tmr", value_name = "29-255")]
    to_timer: Option<u8>,

    /// Maximum extra frames per transmit opportunity. 0 disables bursting.
    #[arg(long, value_name = "0-255")]
    burst_cnt: Option<u8>,

    /// Inter-burst timer in bit times.
    #[arg(long, value_name = "0-255")]
    burst_tmr: Option<u8>,

    /// Also ask the device to persist the new configuration to EEPROM.
    #[arg(long)]
    save: bool,
}

impl SetPlcaArgs {
    /// Whether the user asked for no change at all.
    fn is_empty(&self) -> bool {
        self.enable.is_none()
            && self.role.is_none()
            && self.node_id.is_none()
            && self.node_cnt.is_none()
            && self.to_timer.is_none()
            && self.burst_cnt.is_none()
            && self.burst_tmr.is_none()
    }

    /// Checks that need no device: an empty request, and a TO timer the device would refuse.
    ///
    /// Run before the port is opened so a plain usage mistake does not first fail as a port error.
    fn check_local(&self) -> Result<()> {
        if self.is_empty() {
            bail!(
                "nothing to change; pass at least one setting (see `t1s-cli set-plca-cfg --help`)"
            );
        }
        if let Some(got) = self.to_timer.filter(|&v| v < proto::TO_TIMER_MIN) {
            let min = proto::TO_TIMER_MIN;
            return Err(proto::PayloadError::ToTimerTooSmall { got, min }.into());
        }
        Ok(())
    }
}

/// PLCA enable selector, spelled the way ethtool spells it.
#[derive(Debug, Clone, Copy, ValueEnum)]
enum EnableArg {
    /// PLCA on.
    On,
    /// PLCA off; the segment falls back to CSMA/CD.
    Off,
    /// Defer to the dongle's physical switch.
    Switch,
}

/// Role selector.
#[derive(Debug, Clone, Copy, ValueEnum)]
enum RoleArg {
    /// Generate beacons and own the PLCA cycle.
    Coordinator,
    /// Transmit only in the opportunity assigned by the coordinator.
    Follower,
    /// Defer to the dongle's physical switch.
    Switch,
}

impl From<EnableArg> for PlcaMode {
    fn from(a: EnableArg) -> Self {
        match a {
            EnableArg::On => PlcaMode::Enabled,
            EnableArg::Off => PlcaMode::Disabled,
            EnableArg::Switch => PlcaMode::SetBySwitch,
        }
    }
}

impl From<RoleArg> for Role {
    fn from(a: RoleArg) -> Self {
        match a {
            RoleArg::Coordinator => Role::Coordinator,
            RoleArg::Follower => Role::Follower,
            RoleArg::Switch => Role::SetBySwitch,
        }
    }
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    if let Err(e) = run(&cli) {
        // A device-reported error is a normal outcome, not a crash; keep it on one line.
        eprintln!("t1s-cli: {e:#}");
        std::process::exit(1);
    }
    Ok(())
}

fn run(cli: &Cli) -> Result<()> {
    if let Command::ListPorts = cli.command {
        return list_ports(cli.json);
    }

    // Purely local argument checks run before any port is opened, so a plain usage mistake never
    // surfaces as "cannot open /dev/..." on a machine with no dongle attached.
    if let Command::SetPlcaCfg(args) = &cli.command {
        args.check_local()?;
    }

    let path = device::resolve_port(cli.port.as_deref())?;
    let timeout = Duration::from_millis(cli.timeout_ms);
    let mut dev = device::open(&path, timeout).with_context(|| format!("opening {path}"))?;

    // One cheap read decides which firmware-specific caveats the rest of the output carries.
    let fw = dev
        .firmware_version()
        .context("reading the firmware version")?;

    let outcome = match &cli.command {
        Command::ListPorts => unreachable!("handled above"),
        Command::Info => info(&mut dev, &path, cli.json),
        Command::GetPlcaCfg => get_plca_cfg(&mut dev, &path, cli.json, fw),
        Command::SetPlcaCfg(args) => set_plca_cfg(&mut dev, &path, args, cli.json, fw),
        Command::GetPlcaStatus => get_plca_status(&mut dev, &path, cli.json),
        Command::Defaults => defaults(&mut dev, &path, cli.json),
        Command::Save => {
            dev.save_config()?;
            done(save_message(fw), cli.json)
        }
    };

    // A BootUp arriving mid-session means the dongle reset, so whatever we just read or wrote may
    // describe a configuration that no longer applies. Say so rather than let it pass unnoticed.
    report_unsolicited(&mut dev);
    outcome
}

/// Warn about frames the device sent that were not replies to anything we asked.
fn report_unsolicited<C: device::Channel>(dev: &mut Device<C>) {
    for f in dev.take_unsolicited() {
        if f.id() == Some(proto::MsgId::BootUp) {
            eprintln!(
                "warning: the device sent BootUp mid-session, meaning it reset; its active \
                 configuration may have reverted to the saved one"
            );
        } else {
            eprintln!(
                "warning: unsolicited frame from device: id {:#04x} payload {}",
                f.id_raw,
                hex::encode(&f.payload)
            );
        }
    }
}

/// What `save` reports on success, with the power-cycle caveat only where it was measured.
fn save_message(fw: FwVersion) -> &'static str {
    if fw.has_1_1_faults() {
        "device acknowledged SaveDeviceConfiguration (note: on firmware 1.1 the configuration \
         does not survive a power cycle)"
    } else {
        "device acknowledged SaveDeviceConfiguration"
    }
}

fn done(msg: &str, json: bool) -> Result<()> {
    if json {
        emit(&json!({ "ok": true, "message": msg }))
    } else {
        println!("{msg}");
        Ok(())
    }
}

/// Print a serde value as one line of JSON.
fn emit(v: &serde_json::Value) -> Result<()> {
    println!("{}", serde_json::to_string(v)?);
    Ok(())
}

fn list_ports(json: bool) -> Result<()> {
    let ports = device::list_ports()?;
    if json {
        let items: Vec<serde_json::Value> = ports
            .iter()
            .map(|p| {
                json!({
                    "path": p.path,
                    "vid": p.vid.map(|v| format!("{v:04x}")),
                    "pid": p.pid.map(|v| format!("{v:04x}")),
                    "serial": p.serial,
                    "match": p.is_match,
                })
            })
            .collect();
        return emit(&json!({ "ports": items }));
    }
    if ports.is_empty() {
        println!("No serial ports found.");
        return Ok(());
    }
    println!("Serial ports:");
    for p in &ports {
        let ids = match (p.vid, p.pid) {
            (Some(v), Some(d)) => format!("{v:04x}:{d:04x}"),
            _ => "-".to_string(),
        };
        let mark = if p.is_match { " <- MACH T1S" } else { "" };
        println!(
            "    {:<40} {:<10} {}{}",
            p.path,
            ids,
            p.serial.as_deref().unwrap_or("-"),
            mark
        );
    }
    Ok(())
}

/// The PHY revision line: the vendor's name for the code, the bare code if unlisted, or the fact
/// that the firmware does not answer `GetLan8670Rev`.
fn phy_revision_text(code: Option<u8>) -> String {
    match code {
        None => "not reported by this firmware".to_string(),
        Some(c) => match proto::phy_revision_name(c) {
            Some(name) => format!("{name} ({c:#04x})"),
            None => format!("unknown code ({c:#04x})"),
        },
    }
}

/// Render the identity block. Returns a String so the exact lines are assertable in tests.
fn render_info(path: &str, i: &proto::DeviceInfo) -> String {
    use std::fmt::Write as _;
    let mut o = String::new();
    let _ = writeln!(o, "Device information for {path}:");
    let _ = writeln!(o, "    Serial number: {}", i.serial);
    let _ = writeln!(o, "    Hardware version: {}", i.hardware);
    let _ = writeln!(o, "    Firmware version: {}", i.firmware);
    let _ = writeln!(o, "    MAC address: {}", proto::format_mac(&i.mac));
    let _ = writeln!(o, "    PHY revision: {}", phy_revision_text(i.phy_revision));
    o
}

fn info<C: device::Channel>(dev: &mut Device<C>, path: &str, json: bool) -> Result<()> {
    let i = dev.read_info()?;
    if json {
        return emit(&json!({
            "port": path,
            "serial": i.serial,
            "hardware": i.hardware,
            "firmware": i.firmware,
            "mac": proto::format_mac(&i.mac),
            "phy_revision": i.phy_revision.and_then(proto::phy_revision_name),
            "phy_revision_code": i.phy_revision,
        }));
    }
    print!("{}", render_info(path, &i));
    Ok(())
}

/// Render configuration in a layout deliberately close to `ethtool --get-plca-cfg`.
///
/// The record holds selectors; where a selector defers to the DIP switches, the value the switches
/// resolved to comes from `status`, which is what the PHY actually has. Returns a String rather
/// than printing, so the exact block is assertable in tests.
fn render_cfg(path: &str, c: &Config, s: &Status) -> String {
    use std::fmt::Write as _;
    let mut o = String::new();
    let _ = writeln!(o, "PLCA settings for {path}:");
    // Every selector line reads "value (set by switch|config)": the value is what is in effect,
    // taken from status when the record defers to the DIP switches, and the parenthesis says
    // which. Explicit values are shown as stored; they match status unless a write failed, which
    // set-plca-cfg reports separately.
    let (enabled, source) = match c.plca {
        PlcaMode::Enabled => ("Yes", "config"),
        PlcaMode::Disabled => ("No", "config"),
        PlcaMode::SetBySwitch => (yesno(s.plca_enabled), "switch"),
    };
    let _ = writeln!(o, "\tEnabled: {enabled} (set by {source})");
    // The coordinator's node ID is 0 by the standard, so for a coordinator at 0 there is no
    // provenance to report. A coordinator at a nonzero ID is the contradiction set-plca-cfg warns
    // about and keeps its provenance so the reader can see where the value came from.
    let is_coordinator = match c.role {
        Role::Coordinator => true,
        Role::SetBySwitch => s.coordinator_enabled,
        Role::Follower => false,
    };
    let node_id_in_effect = if c.node_id == Config::SET_BY_SWITCH {
        s.node_id
    } else {
        c.node_id
    };
    let node_id = match c.node_id {
        0 if is_coordinator && node_id_in_effect == 0 => "0".to_string(),
        0xff => "255 (set by config, unconfigured)".to_string(),
        Config::SET_BY_SWITCH => format!("{} (set by switch)", s.node_id),
        n => format!("{n} (set by config)"),
    };
    let _ = writeln!(o, "\tLocal node ID: {node_id}");
    let node_count = if c.node_count == Config::SET_BY_SWITCH {
        format!("{} (set by switch)", s.node_count)
    } else if c.role == Role::Follower {
        format!("{} (set by config, ignored)", c.node_count)
    } else {
        format!("{} (set by config)", c.node_count)
    };
    let _ = writeln!(o, "\tNode count: {node_count}");
    let _ = writeln!(o, "\tTO timer: {} BT", c.to_timer);
    let _ = writeln!(
        o,
        "\tBurst count: {}{}",
        c.max_burst_count,
        if c.max_burst_count == 0 {
            " (disabled)"
        } else {
            ""
        }
    );
    let _ = writeln!(o, "\tBurst timer: {} BT", c.burst_timer);
    // ethtool has no equivalent: on this dongle the role is a device setting, not a PHY register the
    // host can infer.
    let (role, source) = match c.role {
        Role::SetBySwitch if s.coordinator_enabled => ("coordinator", "switch"),
        Role::SetBySwitch => ("follower", "switch"),
        r => (r.name(), "config"),
    };
    let _ = writeln!(o, "\tRole: {role} (set by {source})");
    o
}

/// The stored record plus what is in effect, for every JSON view of a configuration.
fn cfg_json(path: &str, c: &Config, s: &Status) -> serde_json::Value {
    json!({
        "port": path,
        "enabled": c.plca == PlcaMode::Enabled,
        "plca": c.plca.name(),
        "role": c.role.name(),
        "node_id": c.node_id,
        "node_count": c.node_count,
        "to_timer": c.to_timer,
        "burst_count": c.max_burst_count,
        "burst_timer": c.burst_timer,
        "flags_byte": format!("{:02x}", c.flags_byte()),
        "in_effect": {
            "plca_enabled": s.plca_enabled,
            "coordinator_enabled": s.coordinator_enabled,
            "node_id": s.node_id,
            "node_count": s.node_count,
        },
    })
}

/// `get-plca-cfg --json` output: the configuration, plus on firmware 1.1 the caveat that a stored
/// node ID of 0 may never have reached the PHY.
fn get_cfg_json(path: &str, c: &Config, s: &Status, fw: FwVersion) -> serde_json::Value {
    let mut v = cfg_json(path, c, s);
    let suspect = fw.has_1_1_faults() && c.node_id == Config::SET_BY_SWITCH;
    v["node_id_may_be_unapplied"] = json!(suspect);
    if suspect {
        v["note"] = json!(
            "firmware 1.1 stores a written node ID of 0 without applying it; in_effect reports what \
             the PHY has"
        );
    }
    v
}

fn get_plca_cfg<C: device::Channel>(
    dev: &mut Device<C>,
    path: &str,
    json: bool,
    fw: FwVersion,
) -> Result<()> {
    let c = dev.read_config()?;
    let s = dev.read_status()?;
    if json {
        return emit(&get_cfg_json(path, &c, &s, fw));
    }
    print!("{}", render_cfg(path, &c, &s));
    if let Some(note) = zero_node_id_caveat(&c, fw) {
        println!("{note}");
    }
    Ok(())
}

fn set_plca_cfg<C: device::Channel>(
    dev: &mut Device<C>,
    path: &str,
    args: &SetPlcaArgs,
    json: bool,
    fw: FwVersion,
) -> Result<()> {
    // Already checked before the port was opened; re-checked here so the function is safe to call
    // directly.
    args.check_local()?;

    // Read-modify-write, so settings the user did not mention are preserved.
    let before = dev.read_config()?;
    let after = Config {
        plca: args.enable.map_or(before.plca, PlcaMode::from),
        role: args.role.map_or(before.role, Role::from),
        node_id: args.node_id.unwrap_or(before.node_id),
        node_count: args.node_cnt.unwrap_or(before.node_count),
        to_timer: args.to_timer.unwrap_or(before.to_timer),
        max_burst_count: args.burst_cnt.unwrap_or(before.max_burst_count),
        burst_timer: args.burst_tmr.unwrap_or(before.burst_timer),
    };

    // Refuse what the device refuses, with an explanation rather than a bare device error.
    after.validate()?;

    // Warn about settings the device accepts but which break PLCA in practice.
    let mut warnings = after.warnings();
    if !json {
        for w in &warnings {
            eprintln!("warning: {w}");
        }
    }
    // Everything pushed from here on is known only after the device has been written and read, so
    // it is printed in one place below, after the last I/O.
    let printed = warnings.len();

    dev.write_config(&after)?;
    if args.save {
        dev.save_config()?;
    }

    // Read back, because neither the ack nor the status reply proves the PHY took the value.
    let readback = dev.read_config()?;

    // Status is what the PHY has: explicit values must match it, and switch sentinels resolve
    // through it. This read also clears the read-to-clear diagnostic flags.
    let status = dev.read_status()?;
    let unapplied = after.unapplied(&status);

    // A value the user asked for and did not get is an error. One merely carried over by the
    // read-modify-write is a warning: this command did what it was asked, but the device is not in
    // the state its record describes.
    let (asked, carried): (Vec<proto::Unapplied>, Vec<proto::Unapplied>) =
        unapplied.into_iter().partition(|u| match u.field {
            proto::CfgField::NodeId => args.node_id.is_some(),
            proto::CfgField::NodeCount => args.node_cnt.is_some(),
        });

    // The coordinator is node 0, and the manual says the coordinator's node ID is always 0. So when
    // the user asked for the role, did not ask for a nonzero node ID, and PLCA is running, a
    // nonzero node ID in effect means the command failed at what it was asked. A nonzero node ID
    // the user asked for themselves is their own contradiction, already warned about, not failed.
    let coordinator_not_in_effect = matches!(args.role, Some(RoleArg::Coordinator))
        && status.plca_enabled
        && !status.coordinator_in_effect()
        && args.node_id.unwrap_or(Config::SET_BY_SWITCH) == Config::SET_BY_SWITCH;

    // Computed before the output branches so --json reports it too, not only the human path.
    for u in &carried {
        // The coordinator error below explains a carried node ID better than the warning would.
        if coordinator_not_in_effect && u.field == proto::CfgField::NodeId {
            continue;
        }
        warnings.push(carried_note(u));
    }

    // Firmware 1.1 stored a written 0 without applying it, so the value in effect is whatever was
    // there before, not the switch value the write asked for.
    if fw.has_1_1_faults() {
        for (field, wrote, in_effect) in [
            (proto::CfgField::NodeId, after.node_id, status.node_id),
            (
                proto::CfgField::NodeCount,
                after.node_count,
                status.node_count,
            ),
        ] {
            if wrote != Config::SET_BY_SWITCH
                || (coordinator_not_in_effect && field == proto::CfgField::NodeId)
            {
                continue;
            }
            warnings.push(format!(
                "{field} was written as 0 (set by switch), which firmware 1.1 does not apply; \
                 {in_effect} remains in effect"
            ));
        }
    }

    if readback != after {
        warnings.push(format!(
            "read-back differs from what was written (wrote flags {:#04x}, read {:#04x}); the device \
             masks the reserved upper nibble, so this is expected only for those bits",
            after.flags_byte(),
            readback.flags_byte()
        ));
    }

    if !json {
        for w in &warnings[printed..] {
            eprintln!("warning: {w}");
        }
    }

    let failed = !asked.is_empty() || coordinator_not_in_effect;
    if json {
        emit(&set_result_json(
            path,
            &SetOutcome {
                saved: args.save,
                ok: !failed,
                after: &after,
                readback: &readback,
                status: &status,
                asked: &asked,
                warnings: &warnings,
            },
        ))?;
    }
    if failed {
        let mut parts = Vec::new();
        if !asked.is_empty() {
            parts.push(unapplied_message(&asked));
        }
        if coordinator_not_in_effect && !asked.iter().any(|u| u.field == proto::CfgField::NodeId) {
            parts.push(coordinator_unapplied_message(&status, fw));
        }
        bail!("{}", parts.join(" Additionally: "));
    }
    if json {
        return Ok(());
    }

    // Only claim everything is in effect when it is; a carried-over value that was ignored has just
    // been warned about, and repeating "verified in effect" over the top of that would contradict it.
    let verified = if carried.is_empty() {
        " and verified in effect"
    } else {
        "; every setting you asked for is in effect, but see the warning above"
    };
    let persistence = if args.save {
        format!("; {}", save_message(fw))
    } else {
        " (not persistent without --save)".to_string()
    };
    println!("Configuration written{verified}{persistence}.");
    print!("{}", render_cfg(path, &readback, &status));
    Ok(())
}

/// Warning for a nonzero value the read-modify-write carried over and the device did not apply.
fn carried_note(u: &proto::Unapplied) -> String {
    let flag = match u.field {
        proto::CfgField::NodeId => "--node-id",
        proto::CfgField::NodeCount => "--node-cnt",
    };
    format!(
        "{u} - carried over from the stored configuration, not asked for here; pass {flag} with \
         the value you want (0 takes it from the DIP switches)"
    )
}

/// Everything `set-plca-cfg --json` reports, success or failure.
struct SetOutcome<'a> {
    saved: bool,
    ok: bool,
    after: &'a Config,
    readback: &'a Config,
    status: &'a Status,
    asked: &'a [proto::Unapplied],
    warnings: &'a [String],
}

/// One JSON schema for `set-plca-cfg`, so success and failure differ only in the values.
fn set_result_json(path: &str, o: &SetOutcome<'_>) -> serde_json::Value {
    json!({
        "ok": o.ok,
        "saved": o.saved,
        "readback_matches": o.readback == o.after,
        "unapplied": o.asked,
        "node_id_in_effect": o.status.node_id,
        "node_count_in_effect": o.status.node_count,
        "warnings": o.warnings,
        "config": cfg_json(path, o.readback, o.status),
    })
}

/// Explain why an asked-for coordinator role is not in effect, and name the recovery.
fn coordinator_unapplied_message(status: &Status, fw: FwVersion) -> String {
    let recovery = if fw.has_1_1_faults() {
        "On firmware 1.1 a written node ID of 0 is stored but never applied, so a stored 0 is \
         re-sent and ignored by every write. Recover with `t1s-cli defaults` (which puts node ID 0 \
         in effect), then re-run this command"
    } else {
        "Re-run with --node-id 0 so the node ID follows the role (0 takes it from the DIP \
         switches, and the coordinator's node ID is always 0), or run `t1s-cli defaults` and \
         re-apply your settings"
    };
    format!(
        "the coordinator role is selected, but node ID {} is in effect and the coordinator is \
         node 0: this device is a follower and is not beaconing. {recovery}",
        status.node_id
    )
}

/// Explain explicit values the device acknowledged but did not put in effect.
///
/// Kept separate from the call site so the wording is assertable in tests.
fn unapplied_message(us: &[proto::Unapplied]) -> String {
    let list = us
        .iter()
        .map(ToString::to_string)
        .collect::<Vec<_>>()
        .join("; ");
    format!(
        "{list}. The device acknowledged the write but reports different values in effect \
         (`t1s-cli get-plca-status`)"
    )
}

/// On firmware 1.1 a stored node ID of 0 is exactly the value that may never have been applied.
/// `None` on any other firmware, and for any other value, which the device does apply.
fn zero_node_id_caveat(c: &Config, fw: FwVersion) -> Option<&'static str> {
    (fw.has_1_1_faults() && c.node_id == Config::SET_BY_SWITCH).then_some(
        "note: on firmware 1.1 a stored node ID of 0 is not proof the PHY has it - that firmware \
         ignores a written 0.\n\
         \x20     `t1s-cli get-plca-status` reports the node ID actually in effect.",
    )
}

/// Render status in a layout close to `ethtool --get-plca-status`, plus the fields ethtool has no
/// equivalent for.
///
/// Returns a String rather than printing, so the exact block is assertable in tests.
fn render_status(path: &str, s: &Status) -> String {
    use std::fmt::Write as _;
    let mut o = String::new();
    let _ = writeln!(o, "PLCA status of {path}:");
    let _ = writeln!(o, "\tStatus: {}", if s.plca_enabled { "on" } else { "off" });
    let _ = writeln!(
        o,
        "\tCoordinator enabled: {}{}",
        yesno(s.coordinator_enabled),
        if s.coordinator_claim_unapplied() {
            " (selected, but NOT in effect)"
        } else {
            ""
        }
    );
    let _ = writeln!(o, "\tPLCA enabled: {}", yesno(s.plca_enabled));
    let _ = writeln!(
        o,
        "\tTest mode: {}",
        s.test_mode.map_or("undefined", proto::TestMode::describe)
    );
    let _ = writeln!(o, "\tLocal node ID: {}", s.node_id);
    let _ = writeln!(o, "\tNode count: {}", s.node_count);
    let _ = writeln!(o, "\tFlags:");
    for (name, v) in s.flags() {
        let _ = writeln!(o, "\t    {name}: {}", yesno(v));
    }
    let _ = writeln!(o, "\tRaw payload: {}", hex::encode(&s.raw));
    let _ = writeln!(o);
    if s.coordinator_claim_unapplied() {
        let _ = writeln!(
            o,
            "WARNING: the coordinator role is selected but node ID {} is in effect. The coordinator is\n\
             \x20        node 0, so this device is a follower and is not beaconing. Re-run set-plca-cfg\n\
             \x20        with --node-id 0 (the coordinator's node ID is always 0), or run\n\
             \x20        `t1s-cli defaults` and re-apply your settings.",
            s.node_id
        );
        let _ = writeln!(o);
    }
    let _ = writeln!(
        o,
        "note: beaconing can only be confirmed at the far end, e.g. `ethtool --get-plca-status` on a"
    );
    let _ = writeln!(
        o,
        "      peer. The flags above are read-to-clear: they cover events since the previous read,"
    );
    let _ = writeln!(o, "      and this read has just cleared them.");
    o
}

fn status_json(path: &str, s: &Status) -> serde_json::Value {
    let flags: serde_json::Map<String, serde_json::Value> = s
        .flags()
        .iter()
        .map(|(n, v)| ((*n).to_string(), serde_json::Value::Bool(*v)))
        .collect();
    json!({
        "port": path,
        "plca_engaged": s.plca_enabled,
        "coordinator_enabled": s.coordinator_enabled,
        "plca_enabled": s.plca_enabled,
        "test_mode": s.test_mode.map(|m| m.describe()),
        "node_id": s.node_id,
        "node_count": s.node_count,
        // The coordinator is node 0, so this is the question `coordinator_enabled` cannot answer.
        "coordinator_in_effect": s.coordinator_in_effect(),
        "coordinator_claim_unapplied": s.coordinator_claim_unapplied(),
        "flags": flags,
        "raw": hex::encode(&s.raw),
    })
}

fn get_plca_status<C: device::Channel>(dev: &mut Device<C>, path: &str, json: bool) -> Result<()> {
    let s = dev.read_status()?;
    if json {
        return emit(&status_json(path, &s));
    }
    print!("{}", render_status(path, &s));
    Ok(())
}

/// The warning `defaults` emits when the reset did not put node ID 0 in effect.
fn failed_reset_warning(s: &Status) -> Option<String> {
    (!s.coordinator_in_effect()).then(|| {
        format!(
            "node ID {} is still in effect after a reset, which is not the expected 0; a \
             physical replug is the remaining recovery",
            s.node_id
        )
    })
}

fn defaults_json(path: &str, c: &Config, s: &Status) -> serde_json::Value {
    let warnings: Vec<String> = failed_reset_warning(s).into_iter().collect();
    json!({
        "ok": true,
        "config": cfg_json(path, c, s),
        "node_id_in_effect": s.node_id,
        "node_count_in_effect": s.node_count,
        "warnings": warnings,
    })
}

/// Reset the active configuration.
fn defaults<C: device::Channel>(dev: &mut Device<C>, path: &str, json: bool) -> Result<()> {
    dev.default_config()?;
    let c = dev.read_config()?;
    // Report what is now in effect, not just what is stored: after a reset every selector is
    // "switch", so the record alone says nothing about the PHY.
    let s = dev.read_status()?;
    if json {
        return emit(&defaults_json(path, &c, &s));
    }
    println!("Active configuration reset to the device defaults; re-apply your settings.");
    print!("{}", render_cfg(path, &c, &s));
    println!("(The status read that resolved the switch values cleared the diagnostic flags.)");
    if let Some(w) = failed_reset_warning(&s) {
        eprintln!("warning: {w}");
    }
    Ok(())
}

fn yesno(b: bool) -> &'static str {
    if b { "Yes" } else { "No" }
}

#[cfg(test)]
mod tests {
    use super::*;
    use clap::CommandFactory as _;

    const FW_1_1: FwVersion = FwVersion { major: 1, minor: 1 };
    const FW_1_2: FwVersion = FwVersion { major: 1, minor: 2 };
    /// The configuration most tests start from: coordinator of a two-node segment, default timers.
    const COORD_0_OF_2: Config = Config {
        role: Role::Coordinator,
        plca: PlcaMode::Enabled,
        node_id: 0,
        node_count: 2,
        max_burst_count: 0,
        burst_timer: 128,
        to_timer: 32,
    };
    const COORD_0_OF_8: Config = Config {
        node_count: 8,
        ..COORD_0_OF_2
    };

    #[test]
    fn cli_definition_is_valid() {
        Cli::command().debug_assert();
    }

    #[test]
    fn save_message_carries_the_power_cycle_caveat_only_on_firmware_1_1() {
        assert!(save_message(FW_1_1).contains("power cycle"));
        assert!(!save_message(FW_1_2).contains("power cycle"));
        assert!(save_message(FW_1_2).contains("SaveDeviceConfiguration"));
    }

    #[test]
    fn info_block_names_the_phy_revision_or_its_absence() {
        let mut i = proto::DeviceInfo {
            serial: "0C020104".into(),
            hardware: "0.2".into(),
            firmware: "1.2".into(),
            mac: [0xfc, 0x61, 0x79, 0x90, 0x04, 0x56],
            phy_revision: Some(0x05),
        };
        assert!(render_info("/dev/x", &i).contains("PHY revision: C2 (0x05)\n"));
        i.phy_revision = Some(0x07);
        assert!(render_info("/dev/x", &i).contains("PHY revision: unknown code (0x07)\n"));
        i.phy_revision = None;
        assert!(
            render_info("/dev/x", &i).contains("PHY revision: not reported by this firmware\n")
        );
        assert!(render_info("/dev/x", &i).contains("Firmware version: 1.2\n"));
    }

    #[test]
    fn config_json_round_trips_through_serde() {
        let c = COORD_0_OF_2;
        let s = Status::decode(&[0x03, 0x00, 0x02, 0x00, 0x00]).expect("decodes");
        let v = cfg_json("/dev/cu.usbmodem1", &c, &s);
        // Re-parse rather than string-matching, so the assertions are about JSON, not formatting.
        let back: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&v).expect("serialises")).expect("parses");
        assert_eq!(back["port"], "/dev/cu.usbmodem1");
        assert_eq!(back["role"], "coordinator");
        assert_eq!(back["plca"], "on");
        assert_eq!(back["enabled"], true);
        assert_eq!(back["node_count"], 2);
        assert_eq!(back["to_timer"], 32);
        assert_eq!(back["flags_byte"], "0a");
        assert_eq!(back["in_effect"]["node_id"], 0);
        assert_eq!(back["in_effect"]["node_count"], 2);
        assert_eq!(back["in_effect"]["plca_enabled"], true);
        assert_eq!(back["in_effect"]["coordinator_enabled"], true);
    }

    #[test]
    fn status_json_round_trips_and_names_every_flag() {
        let st = Status::decode(&[0x03, 0x01, 0x02, 0x08, 0x00]).expect("decodes");
        let v = status_json("/dev/x", &st);
        let back: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&v).expect("serialises")).expect("parses");
        assert_eq!(back["flags"]["Dec5bError"], true);
        assert_eq!(back["flags"]["UnexpectedBeaconReceived"], false);
        assert_eq!(back["raw"], "0301020800");
        assert_eq!(
            back["flags"].as_object().expect("flags is an object").len(),
            9,
            "all nine diagnostic flags should appear"
        );
    }

    #[test]
    fn json_strings_needing_escapes_survive_a_round_trip() {
        // serde_json owns escaping now; this proves awkward input still parses back identically.
        let weird = "a\"b\\c\nd\te\u{1}f";
        let v = json!({ "message": weird });
        let back: serde_json::Value =
            serde_json::from_str(&serde_json::to_string(&v).expect("serialises")).expect("parses");
        assert_eq!(back["message"], weird);
    }

    /// Build a `SetPlcaArgs` by parsing a real command line, so the tests exercise clap too.
    fn parse_set(argv: &[&str]) -> SetPlcaArgs {
        let mut full = vec!["t1s-cli", "set-plca-cfg"];
        full.extend_from_slice(argv);
        let cli = Cli::try_parse_from(full).expect("parses");
        match cli.command {
            Command::SetPlcaCfg(a) => a,
            other => panic!("wrong subcommand: {other:?}"),
        }
    }

    #[test]
    fn empty_set_plca_cfg_is_rejected_by_the_guard_itself() {
        let empty = parse_set(&[]);
        assert!(empty.is_empty());
        let err = empty
            .check_local()
            .expect_err("an empty request must be refused");
        assert!(err.to_string().contains("nothing to change"), "got: {err}");
    }

    #[test]
    fn any_single_setting_satisfies_the_emptiness_guard() {
        for argv in [
            vec!["--enable", "on"],
            vec!["--role", "follower"],
            vec!["--node-id", "1"],
            vec!["--node-cnt", "8"],
            vec!["--to-timer", "32"],
            vec!["--burst-cnt", "0"],
            vec!["--burst-tmr", "128"],
        ] {
            let a = parse_set(&argv);
            assert!(!a.is_empty(), "{argv:?} should count as a change");
            assert!(a.check_local().is_ok(), "{argv:?} should pass local checks");
        }
    }

    #[test]
    fn to_timer_below_the_minimum_is_refused_without_touching_a_device() {
        for bad in ["0", "10", "28"] {
            let a = parse_set(&["--to-timer", bad]);
            let err = a.check_local().expect_err("must be refused locally");
            assert!(
                err.to_string().contains("below the device minimum"),
                "got: {err}"
            );
        }
        assert!(parse_set(&["--to-timer", "29"]).check_local().is_ok());
    }

    #[test]
    fn ethtool_style_option_names_are_accepted() {
        let a = parse_set(&[
            "--enable",
            "on",
            "--node-id",
            "1",
            "--node-cnt",
            "8",
            "--to-timer",
            "32",
            "--burst-cnt",
            "0",
            "--burst-tmr",
            "128",
        ]);
        assert!(matches!(a.enable, Some(EnableArg::On)));
        assert_eq!((a.node_id, a.node_cnt), (Some(1), Some(8)));
        assert_eq!(
            (a.to_timer, a.burst_cnt, a.burst_tmr),
            (Some(32), Some(0), Some(128))
        );
    }

    #[test]
    fn ethtools_to_tmr_spelling_is_accepted_as_an_alias() {
        assert_eq!(parse_set(&["--to-tmr", "40"]).to_timer, Some(40));
        assert_eq!(parse_set(&["--to-timer", "40"]).to_timer, Some(40));
    }

    #[test]
    fn defaults_subcommand_parses() {
        let cli = Cli::try_parse_from(["t1s-cli", "defaults"]).expect("parses");
        assert!(matches!(cli.command, Command::Defaults));
    }

    #[test]
    fn unapplied_message_names_the_values_and_points_at_status() {
        // A written 0 is the switch sentinel and is never reported, so the only mismatches that
        // reach this message are explicit nonzero values the device did not take.
        let msg = unapplied_message(&[
            proto::Unapplied {
                field: proto::CfgField::NodeId,
                wanted: 5,
                in_effect: 3,
            },
            proto::Unapplied {
                field: proto::CfgField::NodeCount,
                wanted: 8,
                in_effect: 4,
            },
        ]);
        assert!(
            msg.contains("node ID 5 was not applied; 3 is still in effect"),
            "got: {msg}"
        );
        assert!(
            msg.contains("node count 8 was not applied; 4 is still in effect"),
            "got: {msg}"
        );
        assert!(msg.contains("get-plca-status"), "got: {msg}");
        assert!(
            !msg.contains("firmware 1.1"),
            "no 1.1 story for a nonzero mismatch, got: {msg}"
        );
    }

    /// Scripted byte channel so `set_plca_cfg` can run without hardware.
    struct FakeChan {
        replies: std::collections::VecDeque<Vec<u8>>,
    }

    impl FakeChan {
        fn new(replies: &[&[u8]]) -> Self {
            Self {
                replies: replies.iter().map(|r| r.to_vec()).collect(),
            }
        }
    }

    impl std::io::Read for FakeChan {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            match self.replies.pop_front() {
                Some(c) => {
                    let n = c.len().min(buf.len());
                    buf[..n].copy_from_slice(&c[..n]);
                    Ok(n)
                }
                None => Err(std::io::Error::new(
                    std::io::ErrorKind::TimedOut,
                    "no more data",
                )),
            }
        }
    }

    impl std::io::Write for FakeChan {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    // Stored configuration: coordinator + enabled, node ID 0, node count 8 (the stuck-zero state).
    const CFG_STUCK_ZERO: &[u8] = &[
        0x02, 0x21, 0x06, 0x00, 0x0a, 0x00, 0x08, 0x00, 0x80, 0x20, 0xd9, 0x03,
    ];
    const WRITE_ACK: &[u8] = &[0x02, 0x20, 0x00, 0x00, 0x20, 0x03];
    // Status: coordinator selected, but node ID 5 / count 8 in effect.
    const STATUS_ID5: &[u8] = &[
        0x02, 0x25, 0x05, 0x00, 0x03, 0x05, 0x08, 0x00, 0x00, 0x3a, 0x03,
    ];

    #[test]
    fn asked_coordinator_role_that_is_not_in_effect_is_an_error_not_a_success() {
        // `--role coordinator` on a device stuck with node ID 5 in effect: the thing asked for
        // (being the coordinator) did not happen, so exiting 0 with "in effect" would be a lie.
        let chan = FakeChan::new(&[CFG_STUCK_ZERO, WRITE_ACK, CFG_STUCK_ZERO, STATUS_ID5]);
        let mut dev = Device::new(chan, Duration::from_millis(200));
        let args = parse_set(&["--role", "coordinator"]);
        let err = set_plca_cfg(&mut dev, "/dev/x", &args, false, FW_1_2)
            .expect_err("a coordinator that is not in effect must fail the command");
        let msg = format!("{err:#}");
        assert!(msg.contains("not beaconing"), "got: {msg}");
        assert!(
            msg.contains("--node-id 0"),
            "the spec's fix must be named, got: {msg}"
        );
        assert!(
            !msg.contains("firmware 1.1"),
            "no 1.1 blame on 1.2, got: {msg}"
        );

        // On 1.1 the same symptom has a known cause, which the message may name.
        let chan = FakeChan::new(&[CFG_STUCK_ZERO, WRITE_ACK, CFG_STUCK_ZERO, STATUS_ID5]);
        let mut dev = Device::new(chan, Duration::from_millis(200));
        let err = set_plca_cfg(&mut dev, "/dev/x", &args, false, FW_1_1).expect_err("still fails");
        assert!(format!("{err:#}").contains("firmware 1.1"), "got: {err:#}");
    }

    #[test]
    fn explicitly_asked_coordinator_with_nonzero_node_id_warns_but_succeeds() {
        // The user asked for the contradiction themselves; the device applied both values, so per
        // the tool's policy (warn about accepted-but-broken configs, do not refuse) this is Ok.
        let cfg_id5: &[u8] = &[
            0x02, 0x21, 0x06, 0x00, 0x0a, 0x05, 0x08, 0x00, 0x80, 0x20, 0xde, 0x03,
        ];
        let chan = FakeChan::new(&[CFG_STUCK_ZERO, WRITE_ACK, cfg_id5, STATUS_ID5]);
        let mut dev = Device::new(chan, Duration::from_millis(200));
        let args = parse_set(&["--role", "coordinator", "--node-id", "5"]);
        set_plca_cfg(&mut dev, "/dev/x", &args, false, FW_1_2)
            .expect("an explicitly requested contradiction is a warning, not an error");
    }

    #[test]
    fn switch_sentinels_left_in_the_record_are_not_a_failure() {
        // After `defaults` the stored config is 00 00 00 00 80 20 while node ID 0 / count 8 are in
        // effect. Re-applying `--enable on --role coordinator --node-id 0` must succeed: the stored
        // count 0 means "from the switches", and 8 in effect is what the switches resolved to.
        let cfg_post_defaults: &[u8] = &[
            0x02, 0x21, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x20, 0xc7, 0x03,
        ];
        let cfg_readback: &[u8] = &[
            0x02, 0x21, 0x06, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x80, 0x20, 0xd1, 0x03,
        ];
        let status_id0_cnt8: &[u8] = &[
            0x02, 0x25, 0x05, 0x00, 0x03, 0x00, 0x08, 0x00, 0x00, 0x35, 0x03,
        ];
        let chan = FakeChan::new(&[cfg_post_defaults, WRITE_ACK, cfg_readback, status_id0_cnt8]);
        let mut dev = Device::new(chan, Duration::from_millis(200));
        let args = parse_set(&["--enable", "on", "--role", "coordinator", "--node-id", "0"]);
        set_plca_cfg(&mut dev, "/dev/x", &args, false, FW_1_2)
            .expect("switch sentinels in the record must not fail the command");
    }

    #[test]
    fn carried_note_names_the_value_in_effect_and_the_fix() {
        let note = carried_note(&proto::Unapplied {
            field: proto::CfgField::NodeId,
            wanted: 5,
            in_effect: 3,
        });
        assert!(
            note.contains("node ID 5 was not applied; 3 is still in effect"),
            "got: {note}"
        );
        assert!(note.contains("carried over"), "got: {note}");
        assert!(
            note.contains("--node-id"),
            "must point at the explicit fix, got: {note}"
        );
    }

    #[test]
    fn rendered_status_flags_a_coordinator_claim_that_was_never_applied() {
        // Status bytes measured on firmware 1.1 hardware.
        let stuck = Status::decode(&[0x03, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        let out = render_status("/dev/x", &stuck);
        assert!(out.contains("\tLocal node ID: 5\n"), "got:\n{out}");
        assert!(out.contains("\tNode count: 8\n"), "got:\n{out}");
        // The one place "in effect" survives is the contradiction marker on the coordinator line.
        assert!(
            out.contains(
                "	Coordinator enabled: Yes (selected, but NOT in effect)
"
            ),
            "got:
{out}"
        );
        assert!(
            out.contains("is not beaconing"),
            "the contradiction must be spelled out, got:\n{out}"
        );
        assert!(out.contains("t1s-cli defaults"), "got:\n{out}");
        assert!(
            !out.contains("unverified field"),
            "these fields are no longer unverified, got:\n{out}"
        );
    }

    #[test]
    fn status_warning_states_the_contradiction_without_a_firmware_story() {
        // Coordinator + nonzero node ID arises from a deliberate nonzero write; on firmware 1.1 it
        // also arose from the written-zero fault. The status view cannot know which, so it names
        // the contradiction and the fix and nothing else.
        let stuck = Status::decode(&[0x03, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        let out = render_status("/dev/x", &stuck);
        assert!(!out.contains("firmware 1.1"), "got:\n{out}");
        assert!(
            out.contains("--node-id 0"),
            "the fix must be named, got:\n{out}"
        );
    }

    #[test]
    fn rendered_status_of_an_applied_coordinator_has_no_contradiction() {
        let real = Status::decode(&[0x03, 0x00, 0x08, 0x00, 0x00]).expect("decodes");
        let out = render_status("/dev/x", &real);
        assert!(out.contains("\tLocal node ID: 0\n"), "got:\n{out}");
        assert!(!out.contains("is not beaconing"), "got:\n{out}");
        assert!(
            !out.contains("reports what is"),
            "the stored-versus-in-effect preamble is gone; got:\n{out}"
        );
        assert!(
            out.contains("read-to-clear"),
            "the flags caveat stays; got:\n{out}"
        );
        assert!(!out.contains("t1s-cli defaults"), "got:\n{out}");

        // A follower with a nonzero id is ordinary, not a contradiction.
        let follower = Status::decode(&[0x02, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        assert!(!render_status("/dev/x", &follower).contains("is not beaconing"));
    }

    #[test]
    fn set_plca_cfg_json_success_and_failure_share_one_schema() {
        let cfg = COORD_0_OF_8;
        let status = Status::decode(&[0x03, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        let asked = [proto::Unapplied {
            field: proto::CfgField::NodeId,
            wanted: 0,
            in_effect: 5,
        }];
        let warnings = vec!["w".to_string()];
        let fail = set_result_json(
            "/dev/x",
            &SetOutcome {
                saved: false,
                ok: false,
                after: &cfg,
                readback: &cfg,
                status: &status,
                asked: &asked,
                warnings: &warnings,
            },
        );
        let ok = set_result_json(
            "/dev/x",
            &SetOutcome {
                saved: true,
                ok: true,
                after: &cfg,
                readback: &cfg,
                status: &status,
                asked: &[],
                warnings: &[],
            },
        );
        let keys = |v: &serde_json::Value| -> Vec<String> {
            let mut k: Vec<String> = v.as_object().expect("object").keys().cloned().collect();
            k.sort();
            k
        };
        assert_eq!(keys(&fail), keys(&ok), "one schema for both outcomes");
        assert_eq!(fail["ok"], false);
        assert!(
            fail["readback_matches"].is_boolean(),
            "failure must not drop readback_matches, got: {fail}"
        );
        assert_eq!(fail["unapplied"].as_array().expect("array").len(), 1);
        assert_eq!(ok["unapplied"].as_array().expect("array").len(), 0);
    }

    #[test]
    fn get_plca_cfg_json_carries_the_written_zero_caveat_only_on_firmware_1_1() {
        let c = COORD_0_OF_2;
        let s = Status::decode(&[0x03, 0x00, 0x02, 0x00, 0x00]).expect("decodes");
        let v = get_cfg_json("/dev/x", &c, &s, FW_1_1);
        assert_eq!(v["node_id_may_be_unapplied"], true, "got: {v}");
        assert!(
            v["note"].as_str().expect("note").contains("firmware 1.1"),
            "the note must name the firmware it applies to, got: {v}"
        );

        // On 1.2 a stored 0 is the switch sentinel and the resolved value is reported alongside.
        let v = get_cfg_json("/dev/x", &c, &s, FW_1_2);
        assert_eq!(v["node_id_may_be_unapplied"], false, "got: {v}");
        assert!(v.get("note").is_none(), "no caveat on 1.2, got: {v}");
        assert_eq!(v["in_effect"]["node_id"], 0, "got: {v}");
        assert_eq!(v["in_effect"]["node_count"], 2, "got: {v}");
    }

    #[test]
    fn defaults_json_carries_the_failed_reset_warning_like_the_human_path() {
        let c = Config::default();
        // Reset did not take: node ID 5 still in effect. A script must be told.
        let stuck = Status::decode(&[0x03, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        let v = defaults_json("/dev/x", &c, &stuck);
        let warnings = v["warnings"].as_array().expect("warnings array");
        assert_eq!(warnings.len(), 1, "got: {v}");
        assert!(
            warnings[0].as_str().expect("string").contains("replug"),
            "the terminal condition must be named, got: {v}"
        );
        assert_eq!(v["node_id_in_effect"], 5);

        // A reset that took has an empty warnings array, same schema.
        let ok = Status::decode(&[0x03, 0x00, 0x08, 0x00, 0x00]).expect("decodes");
        let v = defaults_json("/dev/x", &c, &ok);
        assert_eq!(v["warnings"].as_array().expect("warnings array").len(), 0);
    }

    #[test]
    fn a_stored_node_id_of_zero_gets_a_caveat_and_other_values_do_not() {
        let coordinator = COORD_0_OF_8;
        let note = zero_node_id_caveat(&coordinator, FW_1_1).expect("a stored 0 needs the caveat");
        assert!(note.contains("get-plca-status"), "got: {note}");
        assert!(
            zero_node_id_caveat(
                &Config {
                    node_id: 1,
                    ..coordinator
                },
                FW_1_1
            )
            .is_none(),
            "a nonzero stored node ID is applied, so it needs no caveat"
        );
        assert!(
            zero_node_id_caveat(&coordinator, FW_1_2).is_none(),
            "firmware 1.2 applies the switch value, so there is nothing to caveat"
        );
    }

    #[test]
    fn rendered_config_matches_the_ethtool_block_layout() {
        let c = COORD_0_OF_2;
        let s = Status::decode(&[0x03, 0x00, 0x02, 0x00, 0x00]).expect("decodes");
        // Exactly the block ethtool users expect, tab-indented, plus the extra Role line.
        // Value first, provenance second. A coordinator's node ID is 0 by the standard, so it
        // carries no provenance at all.
        assert_eq!(
            render_cfg("/dev/cu.usbmodem1", &c, &s),
            "PLCA settings for /dev/cu.usbmodem1:\n\
             \tEnabled: Yes (set by config)\n\
             \tLocal node ID: 0\n\
             \tNode count: 2 (set by config)\n\
             \tTO timer: 32 BT\n\
             \tBurst count: 0 (disabled)\n\
             \tBurst timer: 128 BT\n\
             \tRole: coordinator (set by config)\n"
        );
    }

    #[test]
    fn rendered_config_annotates_follower_and_unconfigured_cases() {
        let follower = Config {
            role: Role::Follower,
            plca: PlcaMode::Enabled,
            node_id: 1,
            node_count: 2,
            max_burst_count: 3,
            burst_timer: 64,
            to_timer: 32,
        };
        let s = Status::decode(&[0x02, 0x01, 0x02, 0x00, 0x00]).expect("decodes");
        let out = render_cfg("/dev/x", &follower, &s);
        assert!(
            out.contains("\tNode count: 2 (set by config, ignored)"),
            "got:\n{out}"
        );
        assert!(
            out.contains("\tLocal node ID: 1 (set by config)\n"),
            "got:\n{out}"
        );
        assert!(
            out.contains("\tBurst count: 3\n"),
            "burst count 3 is not disabled"
        );
        assert!(
            out.contains("\tRole: follower (set by config)"),
            "got:\n{out}"
        );

        let unconfigured = Config {
            node_id: 0xff,
            plca: PlcaMode::Disabled,
            ..follower
        };
        let out = render_cfg("/dev/x", &unconfigured, &s);
        assert!(
            out.contains("\tLocal node ID: 255 (set by config, unconfigured)"),
            "got:\n{out}"
        );
        assert!(out.contains("\tEnabled: No (set by config)"), "got:\n{out}");

        // Switch-controlled selectors show the value the switches resolved to, from status.
        let switched = Config {
            plca: PlcaMode::SetBySwitch,
            role: Role::SetBySwitch,
            ..follower
        };
        let s = Status::decode(&[0x03, 0x01, 0x02, 0x00, 0x00]).expect("decodes");
        let out = render_cfg("/dev/x", &switched, &s);
        assert!(
            out.contains("\tEnabled: Yes (set by switch)"),
            "got:\n{out}"
        );
        assert!(
            out.contains("\tRole: coordinator (set by switch)"),
            "got:\n{out}"
        );
    }

    #[test]
    fn rendered_config_resolves_switch_sentinels_from_status() {
        // Measured on firmware 1.2: the record holds
        // the sentinels, status holds what the switches said.
        let c = Config::decode(&[0x00, 0x00, 0x00, 0x03, 0x64, 0x28]).expect("decodes");
        let s = Status::decode(&[0x03, 0x00, 0x03, 0x00, 0x00]).expect("decodes");
        assert_eq!(
            render_cfg("/dev/x", &c, &s),
            "PLCA settings for /dev/x:\n\
             \tEnabled: Yes (set by switch)\n\
             \tLocal node ID: 0\n\
             \tNode count: 3 (set by switch)\n\
             \tTO timer: 40 BT\n\
             \tBurst count: 3\n\
             \tBurst timer: 100 BT\n\
             \tRole: coordinator (set by switch)\n"
        );

        // The same record with the switches at follower, node 2: now the node ID is information.
        let s = Status::decode(&[0x02, 0x02, 0x03, 0x00, 0x00]).expect("decodes");
        let out = render_cfg("/dev/x", &c, &s);
        assert!(
            out.contains("\tLocal node ID: 2 (set by switch)\n"),
            "got:\n{out}"
        );
        assert!(
            out.contains("\tRole: follower (set by switch)\n"),
            "got:\n{out}"
        );
    }

    #[test]
    fn coordinator_with_an_explicit_nonzero_node_id_still_shows_the_value() {
        // The contradiction set-plca-cfg warns about must stay visible in the rendering.
        let c = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_id: 5,
            node_count: 8,
            max_burst_count: 0,
            burst_timer: 128,
            to_timer: 32,
        };
        let s = Status::decode(&[0x03, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        let out = render_cfg("/dev/x", &c, &s);
        assert!(
            out.contains("\tLocal node ID: 5 (set by config)\n"),
            "got:\n{out}"
        );
    }
}
