# t1s-cli — configure a MACH SYSTEMS 10BASET1S-USB-IF dongle from the command line

`t1s-cli` configures the **MACH SYSTEMS 10BASET1S-USB-IF** 10BASE-T1S dongle over the CDC-ACM
(virtual COM port) control channel exposed by its STM32. The command surface deliberately mirrors
`ethtool`'s PLCA interface.

## Why this exists

The dongle's Ethernet side appears as an ordinary USB network interface. Its **T1S side** — PLCA
role, node ID, node count, timers — is owned by the dongle's STM32 firmware and is reachable only
through a serial control protocol on the CDC-ACM port:

- **The vendor's configuration utility is Windows-only.** On macOS or Linux there is no first-party
  way to set the dongle's PLCA parameters.
- **`ethtool` cannot reach this PHY on any host.** `ethtool --set-plca-cfg` drives a PHY the host's
  own driver owns via MDIO; here the PHY belongs to the STM32, and the host sees only a USB Ethernet
  endpoint downstream of it.

In a typical bring-up, the Linux/embedded end of the link is configured with `ethtool` and the MACH
end with this tool; the [mapping table](#mapping-to-ethtool) lets you read one configuration off the
other.

The control protocol is implemented from MACH SYSTEMS' "10BASE-T1S USB-IF Communication Protocol
Specification", checked against wire captures of the vendor's configuration utility and against the
device itself. Where the specification and the device disagree, this tool follows the device.

## Install

Requires a stable Rust toolchain supporting **edition 2024** (Rust 1.85 or newer).

```sh
cargo install --git https://github.com/stewartallen/mach-10base-t1s-cli
```

or from a local checkout:

```sh
git clone https://github.com/stewartallen/mach-10base-t1s-cli
cd mach-10base-t1s-cli
cargo install --path .
```

Either way `cargo` puts the `t1s-cli` binary in `~/.cargo/bin`, so make sure that is on your `PATH`.
To build without installing, `cargo build --release` leaves it at `target/release/t1s-cli`.

This crate is also a library: `mach_10base_t1s_cli::proto` is a pure, IO-free implementation of the
framing and payloads (useful for decoding captures offline), and `mach_10base_t1s_cli::device` adds
the serial transport.

### Serial port access

The dongle's control channel is a CDC-ACM device:

- **macOS**: `/dev/cu.usbmodem*`. macOS exposes every serial device twice, as `cu.*` and `tty.*`;
  `t1s-cli` uses the correct `cu.*` callout node and discards the `tty.*` twin during autodetection.
  No special privileges are needed.
- **Linux**: `/dev/ttyACM*`. Your user must be able to open it — typically membership of the
  `dialout` group, or an equivalent udev rule.

The port must be in raw mode; `t1s-cli` handles that for you.

## Quickstart

Every example below is real output from a dongle running firmware 1.2.

**1. Find the control port.** Autodetection matches the STM32 CDC-ACM identity `0483:5740`; that
VID:PID is ST's stock pair and shared with many STM32 boards, so confirm identity with `info` if you
have other ST devices attached.

```sh
$ t1s-cli list-ports
Serial ports:
    /dev/cu.debug-console                    -          -
    /dev/cu.Bluetooth-Incoming-Port          -          -
    /dev/cu.usbmodem2088388856311            0483:5740  208838885631 <- MACH T1S
    /dev/tty.usbmodem2088388856311           0483:5740  208838885631 <- MACH T1S
```

If exactly one dongle is found, no other command needs `--port`. Otherwise pass `--port`, or set
`MACH_T1S_PORT` in your environment (an explicit `--port` still wins).

**2. Identify the device.**

```sh
$ t1s-cli info
Device information for /dev/cu.usbmodem2088388856311:
    Serial number: 0C020104
    Hardware version: 0.0
    Firmware version: 1.2
    MAC address: fc:61:79:90:04:56
    PHY revision: C2 (0x05)
```

The MAC shown is the one the control channel reports; the address your operating system sees on the
Ethernet interface can differ, see [Limitations](#limitations).

**3. Read the PLCA configuration.** The layout mirrors `ethtool --get-plca-cfg`, with one extra
line — `Role` — that ethtool has no equivalent for. Each selector shows the value in effect and
whether it came from the configuration or from the DIP switches; a coordinator's node ID is 0 by
the standard, so it shows no source.

```sh
$ t1s-cli get-plca-cfg
PLCA settings for /dev/cu.usbmodem2088388856311:
	Enabled: Yes (set by switch)
	Local node ID: 0
	Node count: 3 (set by switch)
	TO timer: 40 BT
	Burst count: 3
	Burst timer: 100 BT
	Role: coordinator (set by switch)
```

**4. Make this dongle the coordinator of a two-node segment.** `set-plca-cfg` is a
read-modify-write, so anything you do not mention is preserved. Every write is verified against the
device's status before it is reported as done.

```sh
$ t1s-cli set-plca-cfg --role coordinator --node-id 0 --node-cnt 2 --save
Configuration written and verified in effect; device acknowledged SaveDeviceConfiguration.
PLCA settings for /dev/cu.usbmodem2088388856311:
	Enabled: Yes (set by config)
	Local node ID: 0
	Node count: 2 (set by config)
	...
```

A node ID or node count of `0` means "take the value from the DIP switches"; the coordinator's node
ID is always 0, so `--node-id 0` is the right spelling for a coordinator either way. Without `--save`
the write still takes effect immediately; what `--save` persists is described under
[Persistence](#persistence).

**5. Check status.** This reports the PHY's current state and the diagnostic flags. Whether the
coordinator is actually beaconing can only be confirmed at the far end of the link (on Linux:
`ethtool --get-plca-status`).

```sh
$ t1s-cli get-plca-status
PLCA status of /dev/cu.usbmodem2088388856311:
	Status: on
	Coordinator enabled: Yes
	PLCA enabled: Yes
	Test mode: no test mode
	Local node ID: 0
	Node count: 2
	Flags:
	    UnexpectedCarrierSense: No
	    ...
	Raw payload: 0300020000
```

**Validation.** The device's only numeric check is that the TO timer must be at least 29, and
`t1s-cli` refuses smaller values locally. Everything else the device accepts verbatim, including
values that break PLCA; those produce a warning but are still sent:

```sh
$ t1s-cli set-plca-cfg --role follower --node-id 5 --node-cnt 2
warning: node ID 5 is not below the node count 2: a follower outside the coordinator's cycle is
silently starved of transmit opportunities (it still receives, and its host TX counter still
increments, but nothing reaches the wire)
```

**Machine-readable output.** Every command takes `--json`:

```sh
$ t1s-cli --json get-plca-cfg
{"burst_count":3,"burst_timer":100,"enabled":true,"flags_byte":"0a","in_effect":{"coordinator_enabled":true,"node_count":2,"node_id":0,"plca_enabled":true},"node_count":2,"node_id":0,"node_id_may_be_unapplied":false,"plca":"on","port":"/dev/cu.usbmodem2088388856311","role":"coordinator","to_timer":40}
```

`in_effect` is what the PHY has, read from status; the other fields are the stored selectors.

## Commands

| Command | Purpose |
|---|---|
| `list-ports` | List serial ports, marking those that look like a MACH T1S control channel. |
| `info` | Show serial number, hardware revision, firmware version, MAC address and PHY silicon revision. |
| `get-plca-cfg` | Show the stored PLCA and role configuration, with switch-controlled fields resolved from status. |
| `set-plca-cfg` | Change the configuration (read-modify-write; verified against status). |
| `get-plca-status` | Show the PHY's current state, plus the diagnostic flags. |
| `defaults` | Reset the active configuration to the device defaults. |
| `save` | Persist the burst count, burst timer and TO timer to EEPROM (see [Persistence](#persistence)). |

The protocol's diagnostic messages (`ReadSQI`, `DoCableTest`, `T1STestMode`,
`LoadDeviceConfiguration`, and the DCQ signal-quality reads) are implemented in the library but
deliberately not exposed as subcommands; see [Limitations](#limitations) for why.

## Mapping to `ethtool`

For each `t1s-cli` subcommand, the `ethtool` invocation that does the equivalent thing **to a local
Linux PHY** — i.e. what you would run on the *other* end of the segment to make it agree with the
dongle. It is a translation aid, not a claim that `ethtool` can drive this dongle (it cannot; see
[Why this exists](#why-this-exists)).

| `t1s-cli` | Equivalent `ethtool` (local Linux PHY) | Notes |
|---|---|---|
| `list-ports` | *no equivalent* | The nearest analogue is `ip link` / `ethtool -i <dev>`. |
| `info` | `ethtool -i <dev>` (partial) | `-i` gives driver and firmware strings; the MAC comes from `ip link show <dev>`. |
| `get-plca-cfg` | `ethtool --get-plca-cfg <dev>` | Closest one-to-one mapping. |
| `set-plca-cfg --node-id N` | `ethtool --set-plca-cfg <dev> node-id N` | `0` takes the value from the DIP switches. |
| `set-plca-cfg --node-cnt N` | `ethtool --set-plca-cfg <dev> node-cnt N` | `0` takes the value from the DIP switches. |
| `set-plca-cfg --to-timer N` (alias `--to-tmr`) | `ethtool --set-plca-cfg <dev> to-tmr N` | This device additionally enforces `N >= 29`. |
| `set-plca-cfg --burst-cnt N` | `ethtool --set-plca-cfg <dev> burst-cnt N` | |
| `set-plca-cfg --burst-tmr N` | `ethtool --set-plca-cfg <dev> burst-tmr N` | |
| `set-plca-cfg --enable on\|off` | `ethtool --set-plca-cfg <dev> enable on\|off` | `ethtool` refuses `enable on` until a node ID has been set, so set `node-id` first. `t1s-cli` has a third value, `switch`, with no `ethtool` counterpart. |
| `set-plca-cfg --role coordinator\|follower` | *no equivalent* | See below. |
| `get-plca-status` | `ethtool --get-plca-status <dev>` | `t1s-cli` additionally surfaces the device's raw status bytes and flag bits. |
| `save` | *no equivalent* | `ethtool` settings are volatile; persistence is the distribution's job. |

The `enable`/`node-id`/`node-cnt` spellings were verified against `ethtool` 6.14 driving a
kernel-owned LAN867x; check `ethtool --help` on your version before copying the rest verbatim.

**Role selection (`--role`).** `ethtool` has no coordinator/follower knob; in the 802.3cg model the
coordinator is simply the node with node ID 0. This dongle *additionally* has a dedicated role field
whose third value, `switch`, defers to a physical switch on the dongle. The role field does **not**
replace the 802.3cg rule: node ID 0 is still what makes a coordinator, and selecting `coordinator`
with a nonzero node ID in effect produces a device that does not beacon at all.

## Persistence

The dongle has two ways to come up configured after a power cycle, and they cover different fields:

- **The DIP switches** set PLCA enable, the coordinator/follower role, and the node ID or node count
  (switches 1 to 8 are the count when switch 9 selects the coordinator, the ID otherwise). A stored
  selector of `switch`, or a node ID / node count of `0`, means "use the switch value". The switch
  values are applied at runtime as well: both when such a configuration is written and when a switch
  is flipped afterwards, with no command needed. Flipping the node count between 1 and 2 moved a
  follower's ping loss between about 90 % and 0 % with nothing written over the protocol.
- **`save`** persists the three timing fields the protocol specification scopes it to: burst count,
  burst timer and TO timer. Role, enable, node ID and node count are not saved; the switches are
  their power-on source.

Measured across a power cycle: the record came back with the saved timing values and every selector
at `switch`, status showed the switch positions in effect, and the far end saw beacons at once.

## Limitations

- **Two MAC addresses.** `info` reports the MAC the STM32 control channel holds. The Ethernet
  interface your operating system creates carries the MAC the LAN9500A loads from its own EEPROM, and
  on the unit tested the two differ (`fc:61:79:90:04:56` against `00:80:0f:95:04:00`) since its
  firmware 1.2 update, with the EEPROM verified as loaded correctly. The same unit reports hardware
  version `0.0` where its label says `0.2`. The cause is not established and has been reported to the
  vendor. Use `ip link` or `ifconfig` for the address that is actually on the wire.
- **Beaconing cannot be confirmed locally.** `get-plca-status` reports the PHY's configuration and
  event flags, not whether a PLCA cycle is running; only the far end of the link can tell you that
  (on Linux, `ethtool --get-plca-status` on the peer).
- **The status flags are read-to-clear.** Each `get-plca-status`, and the status read inside
  `get-plca-cfg`, `set-plca-cfg` and `defaults`, reports events since the previous read and clears
  them, so anything else polling the device will miss them.
- **A coordinator must be node 0.** Selecting `--role coordinator` with an explicit nonzero node ID
  produces a device that does not beacon; the tool warns, and fails the command when the role was
  asked for and the nonzero node ID was merely carried over from the stored configuration.
- **Diagnostics are not exposed.** `ReadSQI` and `DoCableTest` returned constants that did not track
  the link on the hardware tested; `T1STestMode` has no safe use from a configuration tool; the DCQ
  signal-quality reads need LAN8670 silicon D0, and the unit tested is C2. All are available in the
  library.
- **Older firmware.** Devices still on firmware 1.1 have several faults, including a written node ID
  of 0 never being applied and the DIP switches never being applied. The tool reads the firmware
  version at start-up and prints the relevant caveats on 1.1 only.
- **One device, one line rate.** Everything was verified against a single dongle (hardware 0.2,
  LAN8670 C2, firmware 1.2). 115200 8N1 is the only verified line rate.

## Provenance and licensing

The protocol is implemented from MACH SYSTEMS' published "10BASE-T1S USB-IF Communication Protocol
Specification", verified against wire captures of the vendor's configuration utility talking to the
device and against the device itself. No vendor code or documents are redistributed in this
repository; it contains only the factual protocol details needed for interoperability.

**MACH SYSTEMS is not affiliated with this project.** This is not an official tool, it is not
endorsed or supported by MACH SYSTEMS, and the vendor's trademarks are used only to identify the
hardware it talks to. Use at your own risk.

Licensed under either of [Apache License 2.0](LICENSE-APACHE) or [MIT](LICENSE-MIT), at your option.
