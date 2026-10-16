//! Serial transport: finding the dongle, opening its control channel, and running transactions.
//!
//! The transaction logic is generic over any byte channel so that the state machine can be tested
//! against in-memory fakes replaying real captured bytes, with no hardware present.

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use thiserror::Error;

use crate::proto::{
    self, Config, DeviceError, DeviceInfo, ErrorCode, Frame, FwVersion, MsgId, ParseEvent, Parser,
    PayloadError, Status, TestMode,
};

/// USB vendor ID of the dongle's STM32 CDC-ACM control interface.
pub const STM32_VID: u16 = 0x0483;
/// USB product ID of the dongle's STM32 CDC-ACM control interface.
pub const STM32_PID: u16 = 0x5740;

/// Line settings. The device's control channel is fixed at these; only 115200 is verified to work.
pub const BAUD: u32 = 115_200;

/// Default per-transaction timeout.
///
/// Comfortably above the slowest measured response: reads answer in 0-1 ms, a config write in
/// 3-4 ms, a save in 6-7 ms, and a cable test — the slowest — in about 14 ms.
pub const DEFAULT_TIMEOUT: Duration = Duration::from_millis(1000);

/// Anything that went wrong talking to the device.
#[derive(Debug, Error)]
pub enum Error {
    /// The device replied `GeneralError`.
    #[error(transparent)]
    Device(#[from] DeviceError),
    /// A reply arrived but its payload could not be decoded.
    #[error(transparent)]
    Payload(#[from] PayloadError),
    /// No usable reply arrived before the timeout elapsed.
    #[error("no reply to {} within {} ms", proto::request_name(*request), waited.as_millis())]
    Timeout {
        /// The request that went unanswered.
        request: u8,
        /// How long was waited.
        waited: Duration,
    },
    /// A reply carried the expected ID but an unusable payload length.
    ///
    /// The payload is retained so the transaction can complete as a diagnosable decode error rather
    /// than discarding the evidence.
    #[error("reply to {} had an unusable payload of {} byte(s): {}", proto::request_name(*request), payload.len(), hex::encode(payload))]
    UnexpectedLength {
        /// The request that was sent.
        request: u8,
        /// The payload exactly as received.
        payload: Vec<u8>,
    },
    /// A request this transport will not send: `Restart` (0xFD) and `RestartBoot` (0xFE).
    ///
    /// `RestartBoot` leaves the device in a bootloader that only the vendor's updater can exit, and
    /// `Restart` drops the control channel mid-session; neither has a place in a configuration tool.
    #[error("refusing to send {}: it restarts the device", proto::request_name(*request))]
    Refused {
        /// The request that was refused.
        request: u8,
    },
    /// Underlying serial or IO failure.
    #[error("serial IO error")]
    Io(#[from] io::Error),
    /// Port enumeration or selection failed.
    #[error("{0}")]
    Port(String),
}

/// Result alias for device operations.
pub type Result<T> = std::result::Result<T, Error>;

/// A serial port candidate found during enumeration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortCandidate {
    /// Device node path.
    pub path: String,
    /// USB vendor ID, when the port reports one.
    pub vid: Option<u16>,
    /// USB product ID, when the port reports one.
    pub pid: Option<u16>,
    /// Serial number string, when the port reports one.
    pub serial: Option<String>,
    /// Whether this looks like the MACH dongle's control channel.
    pub is_match: bool,
}

/// Enumerate serial ports, flagging those that look like the dongle's control channel.
///
/// A port matches on VID:PID when the OS reports USB metadata; otherwise the node-name pattern is
/// used, which covers hosts that expose CDC-ACM without USB descriptors.
pub fn list_ports() -> Result<Vec<PortCandidate>> {
    let ports = serialport::available_ports().map_err(|e| Error::Port(e.to_string()))?;
    let mut out = Vec::new();
    for p in ports {
        let (vid, pid, serial) = match p.port_type {
            serialport::SerialPortType::UsbPort(info) => {
                (Some(info.vid), Some(info.pid), info.serial_number)
            }
            _ => (None, None, None),
        };
        let by_ids = vid == Some(STM32_VID) && pid == Some(STM32_PID);
        // Only fall back to the name when the OS gave us no USB identity to judge by.
        let by_name = vid.is_none() && looks_like_cdc_acm(&p.port_name);
        out.push(PortCandidate {
            path: p.port_name,
            vid,
            pid,
            serial,
            is_match: by_ids || by_name,
        });
    }
    Ok(out)
}

fn looks_like_cdc_acm(name: &str) -> bool {
    // macOS exposes the callout device as /dev/cu.usbmodem*; Linux uses /dev/ttyACM*.
    name.contains("cu.usbmodem") || name.contains("ttyACM")
}

/// Drop macOS `/dev/tty.*` nodes when the matching `/dev/cu.*` callout node is also present.
///
/// macOS exposes every serial device twice. The `tty.` node blocks on open until carrier is
/// asserted, which a USB CDC-ACM device does not do, so the `cu.` callout node is the one to use.
/// Without this, autodetection sees two candidates for one dongle and refuses to choose.
fn prefer_callout_nodes(mut candidates: Vec<PortCandidate>) -> Vec<PortCandidate> {
    let callouts: Vec<String> = candidates
        .iter()
        .filter(|c| c.path.contains("/cu."))
        .map(|c| c.path.clone())
        .collect();
    candidates.retain(|c| match c.path.split_once("/tty.") {
        Some((dir, suffix)) => !callouts.contains(&format!("{dir}/cu.{suffix}")),
        None => true,
    });
    candidates
}

/// Choose a port: an explicit path wins, then `MACH_T1S_PORT`, then autodetection.
pub fn resolve_port(explicit: Option<&str>) -> Result<String> {
    if let Some(p) = explicit {
        return Ok(p.to_string());
    }
    if let Some(p) = std::env::var_os("MACH_T1S_PORT") {
        return Ok(p.to_string_lossy().into_owned());
    }
    let matched: Vec<_> = list_ports()?.into_iter().filter(|c| c.is_match).collect();
    let candidates = prefer_callout_nodes(matched);
    match candidates.len() {
        1 => Ok(candidates[0].path.clone()),
        0 => Err(Error::Port(
            "no MACH 10BASE-T1S control port found. Plug the dongle in, or pass --port, or set \
             MACH_T1S_PORT. Run `t1s-cli list-ports` to see what is available."
                .to_string(),
        )),
        _ => {
            let list = candidates
                .iter()
                .map(|c| format!("  {}", c.path))
                .collect::<Vec<_>>()
                .join("\n");
            Err(Error::Port(format!(
                "several candidate ports found; pass --port to choose one:\n{list}"
            )))
        }
    }
}

/// A byte channel to the device. Implemented by a real serial port and by test fakes alike.
pub trait Channel: Read + Write {}
impl<T: Read + Write> Channel for T {}

/// An open control-channel session.
#[derive(Debug)]
pub struct Device<C: Channel> {
    chan: C,
    parser: Parser,
    /// Events parsed but not yet matched to a request, so several frames arriving in one read are
    /// all consumed rather than the extras being dropped.
    pending: VecDeque<ParseEvent>,
    timeout: Duration,
    /// Unsolicited frames seen while awaiting replies; see [`Device::take_unsolicited`].
    unsolicited: Vec<Frame>,
}

/// Open the dongle's control channel on the given path.
///
/// The `serialport` crate puts the tty into raw mode for us. That is load-bearing: a default line
/// discipline corrupts this protocol, whose payloads include `VINTR`, `XON`/`XOFF` and newline
/// bytes.
pub fn open(path: &str, timeout: Duration) -> Result<Device<Box<dyn serialport::SerialPort>>> {
    let port = serialport::new(path, BAUD)
        .data_bits(serialport::DataBits::Eight)
        .parity(serialport::Parity::None)
        .stop_bits(serialport::StopBits::One)
        .flow_control(serialport::FlowControl::None)
        // A read timeout well under the transaction timeout keeps the wait loop responsive without
        // spinning: each read blocks until bytes arrive or this elapses.
        .timeout(Duration::from_millis(50))
        .open()
        .map_err(|e| Error::Port(format!("cannot open {path}: {e}")))?;
    Ok(Device::new(port, timeout))
}

impl<C: Channel> Device<C> {
    /// Wrap an already-open channel.
    pub fn new(chan: C, timeout: Duration) -> Self {
        Self {
            chan,
            parser: Parser::new(),
            pending: VecDeque::new(),
            timeout,
            unsolicited: Vec::new(),
        }
    }

    /// The configured per-transaction timeout.
    #[must_use]
    pub fn timeout(&self) -> Duration {
        self.timeout
    }

    /// Send one request and wait for its reply.
    ///
    /// Returns the reply payload on success. A `GeneralError` reply becomes [`Error::Device`] rather
    /// than being ignored — waiting only for the matching ID would hang forever, since the device
    /// sends the error *instead of* the expected reply.
    pub fn transact(&mut self, id_raw: u8, payload: &[u8]) -> Result<Vec<u8>> {
        if id_raw == MsgId::Restart.as_u8() || id_raw == MsgId::RestartBoot.as_u8() {
            return Err(Error::Refused { request: id_raw });
        }
        let frame = proto::encode(id_raw, payload);
        // Complete the whole frame. Abandoning a partial write would leave the device's
        // length-driven parser mid-frame, where it consumes the next frame's header instead.
        self.chan.write_all(&frame)?;
        self.chan.flush()?;

        let deadline = Instant::now() + self.timeout;
        let mut buf = [0u8; 256];
        loop {
            // Drain anything already complete in the parser before reading more.
            if let Some(done) = self.take_reply(id_raw)? {
                return Ok(done);
            }
            if Instant::now() >= deadline {
                // Nothing legitimate can be half-buffered: only one request is ever in flight.
                self.parser.reset();
                return Err(Error::Timeout {
                    request: id_raw,
                    waited: self.timeout,
                });
            }
            match self.chan.read(&mut buf) {
                // A serial port signals "nothing yet" with a TimedOut error, so a zero-length read
                // means the channel is gone, not "keep waiting".
                Ok(0) => {
                    return Err(Error::Io(io::Error::new(
                        io::ErrorKind::UnexpectedEof,
                        "device closed the control channel",
                    )));
                }
                Ok(n) => {
                    let events = self.parser.feed(&buf[..n]);
                    self.pending.extend(events);
                }
                Err(e) if e.kind() == io::ErrorKind::TimedOut => continue,
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => return Err(Error::Io(e)),
            }
        }
    }

    /// Inspect buffered events for the reply to `id_raw`.
    fn take_reply(&mut self, id_raw: u8) -> Result<Option<Vec<u8>>> {
        while let Some(ev) = self.pending.pop_front() {
            match ev {
                ParseEvent::Frame(f) => {
                    if f.id_raw == MsgId::GeneralError.as_u8() {
                        return Err(Error::Device(DeviceError::decode(&f.payload)?));
                    }
                    if f.id_raw == id_raw {
                        return Ok(Some(f.payload));
                    }
                    // Anything else — notably an unsolicited BootUp after a power-up — is recorded
                    // and skipped rather than mistaken for the reply.
                    self.unsolicited.push(f);
                }
                ParseEvent::Invalid(_) => {
                    // The parser has already resynchronised; keep waiting for a usable reply.
                }
            }
        }
        Ok(None)
    }

    /// Send a request for a known message.
    pub fn request(&mut self, id: MsgId, payload: &[u8]) -> Result<Vec<u8>> {
        self.transact(id.as_u8(), payload)
    }

    /// Discard buffered state and re-establish frame sync using a `ReadSWInfo` probe.
    ///
    /// A `GeneralError` naming `ReadSWInfo` counts as success: it is exactly what a desynchronised
    /// device sends, and either answer proves the device is parsing our frames again.
    pub fn resync(&mut self) -> Result<()> {
        self.parser.reset();
        self.pending.clear();
        match self.request(MsgId::ReadSwInfo, &[]) {
            Ok(payload) if payload.len() == 2 => Ok(()),
            Ok(payload) => Err(Error::UnexpectedLength {
                request: MsgId::ReadSwInfo.as_u8(),
                payload,
            }),
            Err(Error::Device(e)) if e.related_id == MsgId::ReadSwInfo.as_u8() => Ok(()),
            Err(e) => Err(e),
        }
    }

    /// Take the unsolicited frames seen since the last call.
    ///
    /// A mid-session `BootUp` means the dongle reset, so its active configuration may have reverted;
    /// the caller is expected to say so rather than let it pass unnoticed.
    pub fn take_unsolicited(&mut self) -> Vec<Frame> {
        std::mem::take(&mut self.unsolicited)
    }

    /// Read serial number, hardware revision, firmware version, MAC address and, where the
    /// firmware knows the message, the LAN8670 silicon revision.
    pub fn read_info(&mut self) -> Result<DeviceInfo> {
        let serial = proto::decode_serial(&self.request(MsgId::ReadSerialNumber, &[])?)?;
        let hardware = proto::decode_hw_info(&self.request(MsgId::ReadHwInfo, &[])?)?;
        let firmware = self.firmware_version()?.to_string();
        let mac = proto::decode_mac(&self.request(MsgId::ReadMacAddress, &[])?)?;
        let phy_revision = match self.request(MsgId::GetLan8670Rev, &[]) {
            Ok(p) => Some(proto::decode_phy_revision(&p)?),
            // Firmware 1.1 predates 0x29; the spec names this exact reply for unsupported messages.
            Err(Error::Device(e)) if e.code == ErrorCode::UnknownMessageId => None,
            Err(e) => return Err(e),
        };
        Ok(DeviceInfo {
            serial,
            hardware,
            firmware,
            mac,
            phy_revision,
        })
    }

    /// Read the firmware version.
    ///
    /// Cheap (one 2-byte reply) and used to decide which firmware-specific caveats still apply.
    pub fn firmware_version(&mut self) -> Result<FwVersion> {
        let p = self.request(MsgId::ReadSwInfo, &[])?;
        Ok(proto::decode_sw_version(&p)?)
    }

    /// Read the active configuration.
    pub fn read_config(&mut self) -> Result<Config> {
        let p = self.request(MsgId::ReadConfig, &[])?;
        Ok(Config::decode(&p)?)
    }

    /// Write the active configuration. Takes effect on the PHY immediately; it does not persist.
    pub fn write_config(&mut self, cfg: &Config) -> Result<()> {
        cfg.validate()?;
        let p = self.request(MsgId::WriteConfig, &cfg.encode())?;
        expect_ack(MsgId::WriteConfig, p)
    }

    /// Persist the active configuration to EEPROM.
    pub fn save_config(&mut self) -> Result<()> {
        let p = self.request(MsgId::SaveConfig, &[])?;
        expect_ack(MsgId::SaveConfig, p)
    }

    /// Send `LoadDeviceConfiguration`.
    ///
    /// Measured on hardware, this is **not** a revert: issuing it after an unsaved write did not
    /// restore the saved configuration. Its actual effect is undetermined.
    pub fn load_config(&mut self) -> Result<()> {
        let p = self.request(MsgId::LoadConfig, &[])?;
        expect_ack(MsgId::LoadConfig, p)
    }

    /// Reset the active configuration to defaults.
    ///
    /// This is the recovery path when the live PHY state has drifted from the stored configuration.
    pub fn default_config(&mut self) -> Result<()> {
        let p = self.request(MsgId::DefaultConfig, &[])?;
        expect_ack(MsgId::DefaultConfig, p)
    }

    /// Read status and diagnostic flags.
    pub fn read_status(&mut self) -> Result<Status> {
        let p = self.request(MsgId::ReadStatus, &[])?;
        Ok(Status::decode(&p)?)
    }

    /// Read the raw signal-quality byte.
    ///
    /// Returned raw because its meaning is unverified on this product: it reads `0x00` regardless
    /// of link health.
    pub fn read_sqi(&mut self) -> Result<u8> {
        let p = self.request(MsgId::ReadSqi, &[])?;
        expect_byte(MsgId::ReadSqi, p)
    }

    /// Run a cable test, returning the raw result byte. Decode it with
    /// [`proto::CableTestResult::from_u8`].
    pub fn cable_test(&mut self) -> Result<u8> {
        let p = self.request(MsgId::CableTest, &[])?;
        expect_byte(MsgId::CableTest, p)
    }

    /// Select a PHY test mode.
    pub fn set_test_mode(&mut self, mode: TestMode) -> Result<()> {
        let p = self.request(MsgId::TestMode, &[mode.as_u8()])?;
        expect_ack(MsgId::TestMode, p)
    }
}

/// A command whose success is signalled by a zero-length reply.
fn expect_ack(id: MsgId, payload: Vec<u8>) -> Result<()> {
    if payload.is_empty() {
        Ok(())
    } else {
        // Surface it rather than resetting the stream: the transaction did complete.
        Err(Error::UnexpectedLength {
            request: id.as_u8(),
            payload,
        })
    }
}

/// A command whose reply is exactly one byte.
fn expect_byte(id: MsgId, payload: Vec<u8>) -> Result<u8> {
    match payload.first() {
        Some(&b) => Ok(b),
        None => Err(Error::UnexpectedLength {
            request: id.as_u8(),
            payload,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{ErrorCode, PlcaMode, Role};

    /// A fake channel: records what was written, replays scripted reply chunks.
    ///
    /// `chunks` are handed out one per `read` call, which lets a test dribble a reply a byte at a
    /// time or deliver several frames at once.
    #[derive(Debug)]
    struct Fake {
        written: Vec<u8>,
        chunks: std::collections::VecDeque<Vec<u8>>,
        /// Fail every write after this many bytes, to model a short write.
        write_limit: Option<usize>,
    }

    impl Fake {
        fn new(chunks: Vec<Vec<u8>>) -> Self {
            Self {
                written: Vec::new(),
                chunks: chunks.into(),
                write_limit: None,
            }
        }
        fn short_writes(chunks: Vec<Vec<u8>>, limit: usize) -> Self {
            Self {
                write_limit: Some(limit),
                ..Self::new(chunks)
            }
        }
    }

    impl Read for Fake {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            match self.chunks.pop_front() {
                Some(c) => {
                    let n = c.len().min(buf.len());
                    buf[..n].copy_from_slice(&c[..n]);
                    Ok(n)
                }
                // Nothing scripted left: behave like a serial port whose read timeout expired.
                None => Err(io::Error::new(io::ErrorKind::TimedOut, "no more data")),
            }
        }
    }

    impl Write for Fake {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            let n = match self.write_limit {
                Some(limit) => buf.len().min(limit),
                None => buf.len(),
            };
            self.written.extend_from_slice(&buf[..n]);
            Ok(n)
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn dev(chunks: Vec<Vec<u8>>) -> Device<Fake> {
        Device::new(Fake::new(chunks), Duration::from_millis(200))
    }

    // Real reply frames taken from the capture logs.
    const CFG_REPLY: &[u8] = &[
        0x02, 0x21, 0x06, 0x00, 0x0a, 0x00, 0x02, 0x00, 0x80, 0x20, 0xd3, 0x03,
    ];
    const SW_REPLY: &[u8] = &[0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03];
    const STATUS_REPLY: &[u8] = &[
        0x02, 0x25, 0x05, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x2f, 0x03,
    ];
    const WRITE_ACK: &[u8] = &[0x02, 0x20, 0x00, 0x00, 0x20, 0x03];
    const ERR_CONFIG: &[u8] = &[0x02, 0xff, 0x02, 0x00, 0xf0, 0x20, 0x11, 0x03];
    // BOOT_UP carries CANID_BASE_RX (default 0x123) as four bytes, per the spec.
    const BOOTUP: &[u8] = &[0x02, 0x01, 0x04, 0x00, 0x23, 0x01, 0x00, 0x00, 0x29, 0x03];
    const SW_REPLY_1_2: &[u8] = &[0x02, 0x13, 0x02, 0x00, 0x02, 0x01, 0x18, 0x03];
    const REV_REPLY_C2: &[u8] = &[0x02, 0x29, 0x01, 0x00, 0x05, 0x2f, 0x03];
    // GeneralError(UnknownMessageID, 0x29): what pre-1.2 firmware says to GetLan8670Rev.
    const ERR_UNKNOWN_29: &[u8] = &[0x02, 0xff, 0x02, 0x00, 0xa2, 0x29, 0xcc, 0x03];

    #[test]
    fn request_writes_the_exact_wire_bytes_and_decodes_the_reply() {
        let mut d = dev(vec![CFG_REPLY.to_vec()]);
        let cfg = d.read_config().expect("reads config");
        assert_eq!(d.chan.written, vec![0x02, 0x21, 0x00, 0x00, 0x21, 0x03]);
        assert_eq!(cfg.role, Role::Coordinator);
        assert_eq!(cfg.plca, PlcaMode::Enabled);
        assert_eq!((cfg.node_id, cfg.node_count, cfg.to_timer), (0, 2, 32));
    }

    #[test]
    fn general_error_reply_becomes_an_error_instead_of_hanging() {
        // The device sends 0xFF *instead of* 0x20's ack; waiting only for 0x20 would hang forever.
        let mut d = dev(vec![ERR_CONFIG.to_vec()]);
        let cfg = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_count: 2,
            ..Config::default()
        };
        match d.write_config(&cfg) {
            Err(Error::Device(e)) => {
                assert_eq!(e.code, ErrorCode::ConfigurationError);
                assert_eq!(e.related_id, 0x20);
            }
            other => panic!("expected a device error, got {other:?}"),
        }
    }

    #[test]
    fn unsolicited_bootup_before_the_reply_is_skipped_not_mistaken_for_it() {
        let mut chunk = BOOTUP.to_vec();
        chunk.extend_from_slice(CFG_REPLY);
        let mut d = dev(vec![chunk]);
        let cfg = d.read_config().expect("reads config past the BootUp");
        assert_eq!(cfg.node_count, 2);
        assert_eq!(d.unsolicited.len(), 1, "the BootUp should be recorded");
        assert_eq!(d.unsolicited[0].id(), Some(MsgId::BootUp));
    }

    #[test]
    fn reply_dribbled_one_byte_per_read_is_assembled() {
        let chunks = CFG_REPLY.iter().map(|&b| vec![b]).collect();
        let mut d = dev(chunks);
        let cfg = d.read_config().expect("reassembles a dribbled reply");
        assert_eq!(cfg.to_timer, 32);
    }

    #[test]
    fn two_frames_in_one_read_are_both_consumed() {
        // Both replies land in a single read; the second must not be lost.
        let mut chunk = SW_REPLY.to_vec();
        chunk.extend_from_slice(STATUS_REPLY);
        let mut d = dev(vec![chunk]);
        let sw = d.request(MsgId::ReadSwInfo, &[]).expect("first reply");
        assert_eq!(sw, vec![0x01, 0x01]);
        let st = d
            .read_status()
            .expect("second reply came from the same read");
        assert_eq!(st.node_count, 2);
    }

    #[test]
    fn silence_times_out_and_clears_buffered_state() {
        let mut d = dev(vec![]);
        match d.read_config() {
            Err(Error::Timeout { request: 0x21, .. }) => {}
            other => panic!("expected a timeout, got {other:?}"),
        }
        assert_eq!(
            d.parser.buffered(),
            0,
            "a timeout must clear the accumulator"
        );
    }

    #[test]
    fn partial_frame_then_silence_times_out_without_hanging() {
        let mut d = dev(vec![CFG_REPLY[..5].to_vec()]);
        assert!(matches!(d.read_config(), Err(Error::Timeout { .. })));
        assert_eq!(d.parser.buffered(), 0);
    }

    #[test]
    fn short_writes_are_completed_rather_than_abandoned() {
        // A frame truncated on the wire would desynchronise the device's length-driven parser.
        let mut d = Device::new(
            Fake::short_writes(vec![CFG_REPLY.to_vec()], 3),
            Duration::from_millis(200),
        );
        d.read_config().expect("reads config despite short writes");
        assert_eq!(
            d.chan.written,
            vec![0x02, 0x21, 0x00, 0x00, 0x21, 0x03],
            "the whole frame must still reach the wire"
        );
    }

    #[test]
    fn garbage_before_the_reply_is_skipped() {
        let mut chunk = vec![0xde, 0xad, 0xbe, 0xef];
        chunk.extend_from_slice(CFG_REPLY);
        let mut d = dev(vec![chunk]);
        assert!(d.read_config().is_ok());
    }

    #[test]
    fn corrupt_frame_then_good_frame_still_completes() {
        let mut bad = CFG_REPLY.to_vec();
        let last = bad.len() - 2;
        bad[last] ^= 0xff; // ruin the checksum
        let mut chunk = bad;
        chunk.extend_from_slice(CFG_REPLY);
        let mut d = dev(vec![chunk]);
        let cfg = d
            .read_config()
            .expect("recovers on the following good frame");
        assert_eq!(cfg.node_count, 2);
    }

    #[test]
    fn ack_with_unexpected_payload_completes_as_an_error() {
        // Right ID, wrong length: report it, but do not reset the stream.
        let odd_ack = &[0x02, 0x22, 0x02, 0x00, 0xaa, 0xbb, 0x89, 0x03];
        let mut d = dev(vec![odd_ack.to_vec()]);
        match d.save_config() {
            Err(Error::UnexpectedLength {
                request: 0x22,
                ref payload,
            }) if payload == &[0xaa, 0xbb] => {}
            other => panic!("expected an unexpected-length error, got {other:?}"),
        }
    }

    #[test]
    fn write_config_refuses_a_to_timer_the_device_would_reject() {
        let mut d = dev(vec![]);
        let cfg = Config {
            to_timer: 10,
            ..Config::default()
        };
        match d.write_config(&cfg) {
            Err(Error::Payload(PayloadError::ToTimerTooSmall { got: 10, min: 29 })) => {}
            other => panic!("expected local validation to reject it, got {other:?}"),
        }
        assert!(d.chan.written.is_empty(), "nothing should have been sent");
    }

    #[test]
    fn write_config_sends_the_expected_frame() {
        let mut d = dev(vec![WRITE_ACK.to_vec()]);
        let cfg = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_id: 0,
            node_count: 2,
            max_burst_count: 0,
            burst_timer: 128,
            to_timer: 32,
        };
        d.write_config(&cfg).expect("writes config");
        // Byte-for-byte the frame the vendor's configuration utility sends for the same settings.
        assert_eq!(
            d.chan.written,
            vec![
                0x02, 0x20, 0x06, 0x00, 0x0a, 0x00, 0x02, 0x00, 0x80, 0x20, 0xd2, 0x03
            ]
        );
    }

    fn product_replies(sw: &[u8], rev: &[u8]) -> Vec<Vec<u8>> {
        vec![
            vec![0x02, 0x11, 0x04, 0x00, 0x04, 0x01, 0x02, 0x0c, 0x28, 0x03],
            vec![
                0x02, 0x12, 0x06, 0x00, 0x0c, 0x00, 0x02, 0x00, 0x02, 0x00, 0x28, 0x03,
            ],
            sw.to_vec(),
            vec![
                0x02, 0x1b, 0x06, 0x00, 0xfc, 0x61, 0x79, 0x90, 0x04, 0x56, 0xe1, 0x03,
            ],
            rev.to_vec(),
        ]
    }

    #[test]
    fn info_reads_all_five_product_messages() {
        let mut d = dev(product_replies(SW_REPLY_1_2, REV_REPLY_C2));
        let info = d.read_info().expect("reads info");
        assert_eq!(info.serial, "0C020104");
        assert_eq!(info.hardware, "0.2");
        assert_eq!(info.firmware, "1.2");
        assert_eq!(crate::proto::format_mac(&info.mac), "fc:61:79:90:04:56");
        assert_eq!(
            info.phy_revision,
            Some(0x05),
            "firmware 1.2 answers 0x29 with C2"
        );
        // Exactly the five request frames, in order, nothing else.
        assert_eq!(
            d.chan.written,
            [0x11u8, 0x12, 0x13, 0x1b, 0x29]
                .iter()
                .flat_map(|&id| vec![0x02, id, 0x00, 0x00, id, 0x03])
                .collect::<Vec<u8>>()
        );
    }

    #[test]
    fn info_tolerates_a_firmware_that_does_not_know_get_lan8670_rev() {
        // Firmware 1.1 predates 0x29 and answers GeneralError(UnknownMessageID). That is not a
        // failure of `info`; the revision is simply unavailable.
        let mut d = dev(product_replies(SW_REPLY, ERR_UNKNOWN_29));
        let info = d.read_info().expect("info must still succeed");
        assert_eq!(info.firmware, "1.1");
        assert_eq!(info.phy_revision, None);
    }

    #[test]
    fn firmware_version_reads_major_and_minor() {
        let mut d = dev(vec![SW_REPLY_1_2.to_vec()]);
        let fw = d.firmware_version().expect("reads");
        assert_eq!((fw.major, fw.minor), (1, 2));
        assert_eq!(fw.to_string(), "1.2");
        assert!(!fw.has_1_1_faults());
        let mut d = dev(vec![SW_REPLY.to_vec()]);
        assert!(d.firmware_version().expect("reads").has_1_1_faults());
    }

    #[test]
    fn restart_ids_are_refused_before_reaching_the_wire() {
        // 0xFE enters a bootloader that only the vendor's updater can leave; 0xFD restarts the
        // device under the caller's feet. Neither is something this transport will send.
        for (id, payload) in [(0xfeu8, vec![0x01]), (0xfd, vec![])] {
            let mut d = dev(vec![]);
            match d.transact(id, &payload) {
                Err(Error::Refused { request }) => assert_eq!(request, id),
                other => panic!("expected Refused for {id:#04x}, got {other:?}"),
            }
            assert!(d.chan.written.is_empty(), "{id:#04x} must never be written");
        }
    }

    #[test]
    fn resync_clears_leftover_state_and_reestablishes_sync() {
        let mut d = dev(vec![SW_REPLY.to_vec()]);
        // Simulate the debris a previous aborted transaction would leave behind.
        d.parser.feed(&[0x02, 0x21, 0x06]);
        d.pending
            .push_back(ParseEvent::Frame(Frame::new(0x99, vec![0xaa])));
        assert!(d.parser.buffered() > 0 && !d.pending.is_empty());

        d.resync().expect("resyncs on a fresh ReadSWInfo exchange");
        assert_eq!(d.parser.buffered(), 0, "no debris should remain");
        assert_eq!(
            d.chan.written,
            vec![0x02, 0x13, 0x00, 0x00, 0x13, 0x03],
            "resync should use the documented ReadSWInfo probe"
        );
    }

    #[test]
    fn sqi_and_cable_test_return_raw_bytes() {
        let mut d = dev(vec![vec![0x02, 0x26, 0x01, 0x00, 0x00, 0x27, 0x03]]);
        assert_eq!(d.read_sqi().expect("sqi"), 0x00);
        let mut d = dev(vec![vec![0x02, 0x27, 0x01, 0x00, 0x06, 0x2e, 0x03]]);
        assert_eq!(d.cable_test().expect("cable test"), 0x06);
    }

    fn cand(path: &str) -> PortCandidate {
        PortCandidate {
            path: path.to_string(),
            vid: Some(STM32_VID),
            pid: Some(STM32_PID),
            serial: None,
            is_match: true,
        }
    }

    #[test]
    fn macos_tty_twin_is_dropped_in_favour_of_the_callout_node() {
        // Exactly what the real dongle presents on macOS: the same device twice.
        let got = prefer_callout_nodes(vec![
            cand("/dev/cu.usbmodem2088388856311"),
            cand("/dev/tty.usbmodem2088388856311"),
        ]);
        assert_eq!(
            got.len(),
            1,
            "one physical dongle should yield one candidate"
        );
        assert_eq!(got[0].path, "/dev/cu.usbmodem2088388856311");
    }

    #[test]
    fn a_tty_node_without_a_callout_twin_is_kept() {
        // Linux only ever exposes /dev/ttyACM0, and it must not be discarded.
        let got = prefer_callout_nodes(vec![cand("/dev/ttyACM0")]);
        assert_eq!(got.len(), 1);
        assert_eq!(got[0].path, "/dev/ttyACM0");
        // A tty. node whose callout twin is absent is also kept.
        let got = prefer_callout_nodes(vec![cand("/dev/tty.usbmodemXYZ")]);
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn two_distinct_dongles_remain_two_candidates() {
        let got = prefer_callout_nodes(vec![
            cand("/dev/cu.usbmodem1111"),
            cand("/dev/tty.usbmodem1111"),
            cand("/dev/cu.usbmodem2222"),
            cand("/dev/tty.usbmodem2222"),
        ]);
        assert_eq!(got.len(), 2, "two dongles, two callout nodes");
        assert!(got.iter().all(|c| c.path.contains("/cu.")));
    }

    #[test]
    fn cdc_acm_name_matching() {
        assert!(looks_like_cdc_acm("/dev/cu.usbmodem2088388856311"));
        assert!(looks_like_cdc_acm("/dev/ttyACM0"));
        assert!(!looks_like_cdc_acm("/dev/cu.Bluetooth-Incoming-Port"));
        assert!(!looks_like_cdc_acm("/dev/cu.debug-console"));
    }
}
