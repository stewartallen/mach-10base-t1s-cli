//! Pure, IO-free implementation of the MACH SYSTEMS 10BASE-T1S serial control protocol.
//!
//! Frame layout, identical in both directions:
//!
//! ```text
//! STX(0x02) | ID(1) | len_lo | len_hi | payload[len] | checksum(1) | ETX(0x03)
//! ```
//!
//! `len` is a little-endian u16 counting only the payload, so a whole frame is `len + 6` bytes.
//! The checksum is `(ID + len + sum(payload)) & 0xff`, where `len` is the numeric length rather
//! than its two bytes summed separately.
//!
//! There is no byte stuffing: payloads carry `0x02` and `0x03` verbatim, so parsing must be driven
//! by the length field and must never scan for `ETX`.

use std::fmt;

use serde::Serialize;
use thiserror::Error;

/// Start-of-frame marker.
pub const STX: u8 = 0x02;
/// End-of-frame marker.
pub const ETX: u8 = 0x03;
/// Bytes of framing overhead around the payload: STX, ID, two length bytes, checksum, ETX.
pub const FRAME_OVERHEAD: usize = 6;

/// Sanity cap applied to a decoded payload length.
///
/// The 10BASE-T1S messages carry at most 6 payload bytes, but the same VCP also carries the
/// device's USB-CAN side, whose `CAN_RECEIVED_MESSAGE` (0x6B) runs to 79 payload bytes and arrives
/// unsolicited whenever the CAN channel is running. Anything above this is a false start-of-frame:
/// treat the stream as desynchronised rather than waiting for bytes that will never arrive.
pub const MAX_PAYLOAD_LEN: usize = 80;

/// Message identifiers, as they appear in the ID byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum MsgId {
    /// Emitted unsolicited by the device after power-up; has no request form.
    BootUp = 0x01,
    /// Read the device serial number.
    ReadSerialNumber = 0x11,
    /// Read hardware revision information.
    ReadHwInfo = 0x12,
    /// Read firmware version.
    ReadSwInfo = 0x13,
    /// Read the Ethernet MAC address.
    ReadMacAddress = 0x1B,
    /// Write the active 10BASE-T1S/PLCA configuration.
    WriteConfig = 0x20,
    /// Read the active 10BASE-T1S/PLCA configuration.
    ReadConfig = 0x21,
    /// Persist the active configuration to EEPROM.
    SaveConfig = 0x22,
    /// Load configuration from EEPROM. Not a revert; its real effect is undetermined.
    LoadConfig = 0x23,
    /// Reset the active configuration to defaults.
    DefaultConfig = 0x24,
    /// Read link/PLCA status and diagnostic flags.
    ReadStatus = 0x25,
    /// Read the signal quality indicator.
    ReadSqi = 0x26,
    /// Run a cable test.
    CableTest = 0x27,
    /// Select a PHY test mode.
    TestMode = 0x28,
    /// Read the LAN8670 silicon revision code. Answered by firmware 1.2; see [`phy_revision_name`].
    GetLan8670Rev = 0x29,
    /// Read the 8-level DCQ signal quality for a node ID. Needs LAN8670 revision D0 or later;
    /// earlier silicon answers `UnknownMessageID`.
    ReadDcqSqi = 0x30,
    /// Read the 32-level DCQ signal quality for a node ID. Same silicon requirement as
    /// [`MsgId::ReadDcqSqi`].
    ReadDcqSqiPlus = 0x31,
    /// Restart the device. Never sent by this crate's transport.
    Restart = 0xFD,
    /// Restart into a bootloader (payload 0 = system, 1 = USB). Never sent by this crate's
    /// transport: recovering from it needs the vendor's firmware updater.
    RestartBoot = 0xFE,
    /// Error reply; carries the failing request's ID. Has no request form.
    GeneralError = 0xFF,
}

impl MsgId {
    /// Every 10BASE-T1S identifier the protocol defines, in ascending wire order. The USB-CAN
    /// identifiers (0x5A to 0x6C) are not modelled.
    pub const ALL: &'static [MsgId] = &[
        MsgId::BootUp,
        MsgId::ReadSerialNumber,
        MsgId::ReadHwInfo,
        MsgId::ReadSwInfo,
        MsgId::ReadMacAddress,
        MsgId::WriteConfig,
        MsgId::ReadConfig,
        MsgId::SaveConfig,
        MsgId::LoadConfig,
        MsgId::DefaultConfig,
        MsgId::ReadStatus,
        MsgId::ReadSqi,
        MsgId::CableTest,
        MsgId::TestMode,
        MsgId::GetLan8670Rev,
        MsgId::ReadDcqSqi,
        MsgId::ReadDcqSqiPlus,
        MsgId::Restart,
        MsgId::RestartBoot,
        MsgId::GeneralError,
    ];

    /// The raw ID byte.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Map an ID byte back to a known identifier, or `None` if unassigned.
    #[must_use]
    pub fn from_u8(v: u8) -> Option<Self> {
        Self::ALL.iter().copied().find(|m| m.as_u8() == v)
    }

    /// Human-readable name: the specification's message name in CamelCase.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            MsgId::BootUp => "BootUp",
            MsgId::ReadSerialNumber => "ReadSerialNumber",
            MsgId::ReadHwInfo => "ReadHWInfo",
            MsgId::ReadSwInfo => "ReadSWInfo",
            MsgId::ReadMacAddress => "ETHReadMACAddress",
            MsgId::WriteConfig => "WriteDeviceConfiguration",
            MsgId::ReadConfig => "ReadDeviceConfiguration",
            MsgId::SaveConfig => "SaveDeviceConfiguration",
            MsgId::LoadConfig => "LoadDeviceConfiguration",
            MsgId::DefaultConfig => "DefaultDeviceConfiguration",
            MsgId::ReadStatus => "ReadStatus",
            MsgId::ReadSqi => "ReadSQI",
            MsgId::CableTest => "DoCableTest",
            MsgId::TestMode => "T1STestMode",
            MsgId::GetLan8670Rev => "GetLAN8670Rev",
            MsgId::ReadDcqSqi => "ReadDCQSQI",
            MsgId::ReadDcqSqiPlus => "ReadDCQSQIPlus",
            MsgId::Restart => "Restart",
            MsgId::RestartBoot => "RestartBoot",
            MsgId::GeneralError => "GeneralError",
        }
    }
}

impl fmt::Display for MsgId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ({:#04x})", self.name(), self.as_u8())
    }
}

/// One decoded frame: the ID byte as received, plus its payload.
///
/// `id_raw` is kept rather than only the parsed [`MsgId`] so that a reply bearing an unassigned
/// identifier can still be surfaced to the user instead of being silently dropped.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    /// The ID byte exactly as it appeared on the wire.
    pub id_raw: u8,
    /// Payload bytes, with no unescaping (the protocol has none).
    pub payload: Vec<u8>,
}

impl Frame {
    /// Build a frame from an ID byte and payload.
    #[must_use]
    pub fn new(id_raw: u8, payload: Vec<u8>) -> Self {
        Self { id_raw, payload }
    }

    /// The parsed identifier, or `None` if the ID byte is unassigned.
    #[must_use]
    pub fn id(&self) -> Option<MsgId> {
        MsgId::from_u8(self.id_raw)
    }

    /// Serialise to wire bytes, computing the checksum.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        encode(self.id_raw, &self.payload)
    }
}

/// Checksum over an ID byte and payload: `(id + len + sum(payload)) & 0xff`.
///
/// The length contribution is the numeric payload length, not its two wire bytes added separately.
/// Both give the same answer for every payload this device produces (the high length byte is always
/// zero), but they diverge at 256 bytes, and this is the rule the device implements.
#[must_use]
pub fn checksum(id_raw: u8, payload: &[u8]) -> u8 {
    let sum: u32 = payload.iter().map(|&b| u32::from(b)).sum();
    ((u32::from(id_raw) + payload.len() as u32 + sum) & 0xff) as u8
}

/// Serialise an ID byte and payload into a complete frame.
#[must_use]
pub fn encode(id_raw: u8, payload: &[u8]) -> Vec<u8> {
    let len = payload.len();
    let mut out = Vec::with_capacity(len + FRAME_OVERHEAD);
    out.push(STX);
    out.push(id_raw);
    out.push((len & 0xff) as u8);
    out.push(((len >> 8) & 0xff) as u8);
    out.extend_from_slice(payload);
    out.push(checksum(id_raw, payload));
    out.push(ETX);
    out
}

/// Why a byte stream could not be interpreted as a frame.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum FrameError {
    /// The checksum byte did not match the payload.
    #[error("bad checksum: frame carried {got:#04x}, computed {want:#04x}")]
    BadChecksum {
        /// Value carried in the frame.
        got: u8,
        /// Value computed over the received ID and payload.
        want: u8,
    },
    /// The byte in the ETX position was not `0x03`.
    #[error("bad end byte: expected 0x03, found {got:#04x}")]
    BadEtx {
        /// The byte found where ETX was expected.
        got: u8,
    },
    /// The decoded length exceeded [`MAX_PAYLOAD_LEN`], so the stream is desynchronised.
    #[error(
        "implausible payload length {len} (cap {MAX_PAYLOAD_LEN}); treating stream as desynchronised"
    )]
    LengthTooLarge {
        /// The implausible length that was decoded.
        len: usize,
    },
}

/// One item produced by the incremental [`Parser`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ParseEvent {
    /// A well-formed frame.
    Frame(Frame),
    /// A frame-shaped run of bytes that failed validation. The parser has already resynchronised.
    Invalid(FrameError),
}

/// Incremental, length-driven frame parser.
///
/// Feed it whatever bytes a read returned; it copes with a frame split across reads, several frames
/// in one read, and leading garbage. It never scans for `ETX`, because payloads may contain it.
#[derive(Debug, Default)]
pub struct Parser {
    buf: Vec<u8>,
}

impl Parser {
    /// A parser with an empty buffer.
    #[must_use]
    pub fn new() -> Self {
        Self { buf: Vec::new() }
    }

    /// Discard any buffered partial frame.
    ///
    /// Correct to call after a transaction timeout: the protocol allows only one request in flight,
    /// so nothing legitimate can be partially buffered once a transaction has been abandoned.
    pub fn reset(&mut self) {
        self.buf.clear();
    }

    /// Bytes currently held awaiting more input.
    #[must_use]
    pub fn buffered(&self) -> usize {
        self.buf.len()
    }

    /// Append bytes and return every event they completed.
    pub fn feed(&mut self, bytes: &[u8]) -> Vec<ParseEvent> {
        self.buf.extend_from_slice(bytes);
        std::iter::from_fn(|| self.step()).collect()
    }

    /// The next completed event, or `None` when more bytes are needed.
    fn step(&mut self) -> Option<ParseEvent> {
        // Skip anything before the first plausible start-of-frame.
        match self.buf.iter().position(|&b| b == STX) {
            Some(0) => {}
            Some(skip) => {
                self.buf.drain(..skip);
            }
            None => {
                self.buf.clear();
                return None;
            }
        }

        // Need the 4-byte header before the length is known.
        if self.buf.len() < 4 {
            return None;
        }
        let id_raw = self.buf[1];
        let len = usize::from(self.buf[2]) | (usize::from(self.buf[3]) << 8);

        if len > MAX_PAYLOAD_LEN {
            // A false start-of-frame. Drop this byte and resynchronise on the next one so a real
            // frame later in the buffer is still found.
            self.buf.remove(0);
            return Some(ParseEvent::Invalid(FrameError::LengthTooLarge { len }));
        }

        let total = len + FRAME_OVERHEAD;
        if self.buf.len() < total {
            return None;
        }

        let payload = self.buf[4..4 + len].to_vec();
        let cks = self.buf[4 + len];
        let etx = self.buf[5 + len];
        let want = checksum(id_raw, &payload);

        if cks != want {
            self.buf.remove(0);
            return Some(ParseEvent::Invalid(FrameError::BadChecksum {
                got: cks,
                want,
            }));
        }
        if etx != ETX {
            self.buf.remove(0);
            return Some(ParseEvent::Invalid(FrameError::BadEtx { got: etx }));
        }

        self.buf.drain(..total);
        Some(ParseEvent::Frame(Frame::new(id_raw, payload)))
    }
}

/// Parse a buffer that is expected to hold whole frames, ignoring incomplete trailing bytes.
///
/// Convenience wrapper over [`Parser`] for tests and one-shot decoding.
#[must_use]
pub fn parse_all(bytes: &[u8]) -> Vec<ParseEvent> {
    let mut p = Parser::new();
    p.feed(bytes)
}

/// Why a payload could not be decoded, or a value could not be accepted.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum PayloadError {
    /// The payload was not the length this message requires.
    ///
    /// The bytes are retained, not just their count: a reply with the right ID and the wrong length
    /// completes the transaction as a decode error, and the raw payload is the only thing left to
    /// diagnose it with.
    #[error("{msg} reply has {} payload byte(s), expected {want}: {}", payload.len(), hex::encode(payload))]
    WrongLength {
        /// Which message was being decoded.
        msg: MsgId,
        /// Length required (a minimum for messages specified as "at least N").
        want: usize,
        /// The payload exactly as received.
        payload: Vec<u8>,
    },
    /// A two-bit role or PLCA selector was given the reserved value 3, which the device rejects.
    #[error("{field} selector value 3 is reserved and is rejected by the device")]
    ReservedSelector {
        /// Which field: `"role"` or `"plca"`.
        field: &'static str,
    },
    /// The transmit-opportunity timer was below the device's accepted minimum.
    #[error(
        "TO timer {got} is below the device minimum of {min} BT; the device would reply ConfigurationError"
    )]
    ToTimerTooSmall {
        /// The rejected value.
        got: u8,
        /// Smallest value the device accepts.
        min: u8,
    },
}

/// Smallest transmit-opportunity timer the device accepts.
///
/// Established by an exhaustive 0-255 sweep: 0 through 28 are rejected with `ConfigurationError`,
/// 29 through 255 are accepted. This is the device's only numeric validation of any config field.
pub const TO_TIMER_MIN: u8 = 29;

/// Coordinator/follower selection, config `data[0]` bits 1:0. The discriminant is the wire value;
/// the serde spelling is the CLI's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize)]
#[repr(u8)]
pub enum Role {
    /// Defer to the dongle's physical switch.
    #[default]
    #[serde(rename = "switch")]
    SetBySwitch = 0,
    /// Act as a PLCA follower.
    #[serde(rename = "follower")]
    Follower = 1,
    /// Act as the PLCA coordinator (beacon source).
    #[serde(rename = "coordinator")]
    Coordinator = 2,
}

impl Role {
    /// Wire value for bits 1:0.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Decode from bits 1:0. Value 3 is reserved and has no meaning.
    #[must_use]
    pub const fn from_bits(v: u8) -> Option<Self> {
        Some(match v & 0x03 {
            0 => Role::SetBySwitch,
            1 => Role::Follower,
            2 => Role::Coordinator,
            _ => return None,
        })
    }

    /// Lowercase name as accepted and printed by the CLI.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Role::SetBySwitch => "switch",
            Role::Follower => "follower",
            Role::Coordinator => "coordinator",
        }
    }
}

impl fmt::Display for Role {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// PLCA enable selection, config `data[0]` bits 3:2. The discriminant is the wire value; the serde
/// spelling is the CLI's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize)]
#[repr(u8)]
pub enum PlcaMode {
    /// Defer to the dongle's physical switch.
    #[default]
    #[serde(rename = "switch")]
    SetBySwitch = 0,
    /// PLCA off; the segment falls back to plain CSMA/CD.
    #[serde(rename = "off")]
    Disabled = 1,
    /// PLCA on.
    #[serde(rename = "on")]
    Enabled = 2,
}

impl PlcaMode {
    /// Wire value for bits 3:2.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Decode from bits 3:2. Value 3 is reserved and has no meaning.
    #[must_use]
    pub const fn from_bits(v: u8) -> Option<Self> {
        Some(match v & 0x03 {
            0 => PlcaMode::SetBySwitch,
            1 => PlcaMode::Disabled,
            2 => PlcaMode::Enabled,
            _ => return None,
        })
    }

    /// Lowercase name as accepted by the CLI.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            PlcaMode::SetBySwitch => "switch",
            PlcaMode::Disabled => "off",
            PlcaMode::Enabled => "on",
        }
    }
}

impl fmt::Display for PlcaMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// The 6-byte configuration payload shared by `WriteConfig` (0x20) and `ReadConfig` (0x21).
///
/// Wire order is `[flags, node_id, node_count, max_burst_count, burst_timer, to_timer]`. Note that
/// the transmit-opportunity timer is the *last* byte, after the burst timer.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Config {
    /// Coordinator/follower selection (flags bits 1:0).
    pub role: Role,
    /// PLCA enable selection (flags bits 3:2).
    pub plca: PlcaMode,
    /// Local PLCA node ID, or [`Config::SET_BY_SWITCH`] to take it from the DIP switches. The
    /// coordinator is node 0, which the device derives from the coordinator role; followers use
    /// 1..=node_count-1.
    pub node_id: u8,
    /// Number of nodes in the PLCA cycle, or [`Config::SET_BY_SWITCH`] to take it from the DIP
    /// switches. Meaningful only on the coordinator.
    pub node_count: u8,
    /// Maximum additional frames a node may send per transmit opportunity. 0 disables bursting.
    pub max_burst_count: u8,
    /// Inter-burst timer, in bit times.
    pub burst_timer: u8,
    /// Transmit-opportunity timer, in bit times. The device requires at least [`TO_TIMER_MIN`].
    pub to_timer: u8,
}

/// The record `DefaultConfig` (0x24) leaves behind, `00 00 00 00 80 20`: every selector set by
/// switch, bursting off, burst timer 128 BT, TO timer 32 BT. Unlike a derived default it passes
/// [`Config::validate`].
impl Default for Config {
    fn default() -> Self {
        Self {
            role: Role::SetBySwitch,
            plca: PlcaMode::SetBySwitch,
            node_id: Self::SET_BY_SWITCH,
            node_count: Self::SET_BY_SWITCH,
            max_burst_count: 0,
            burst_timer: 128,
            to_timer: 32,
        }
    }
}

impl Config {
    /// Payload length of a configuration message.
    pub const LEN: usize = 6;

    /// The node ID / node count value meaning "take it from the DIP switches".
    ///
    /// The spec defines `0x00` this way for both fields; the switch-resolved value is what
    /// `ReadStatus` reports in effect. Firmware 1.2 honours it (measured); firmware 1.1 stored it
    /// and left the previous value in effect.
    pub const SET_BY_SWITCH: u8 = 0;

    /// Pack the flags byte. Bits 7:4 are reserved and written as zero.
    ///
    /// The device ignores bits 7:4 on write and reads them back as zero, so writing them as zero
    /// makes a written value round-trip identically.
    #[must_use]
    pub const fn flags_byte(&self) -> u8 {
        self.role.as_u8() | (self.plca.as_u8() << 2)
    }

    /// Encode to the 6-byte wire payload.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        vec![
            self.flags_byte(),
            self.node_id,
            self.node_count,
            self.max_burst_count,
            self.burst_timer,
            self.to_timer,
        ]
    }

    /// Decode from a 6-byte wire payload.
    ///
    /// A reserved selector value of 3 in either field cannot be produced by this device, but is
    /// reported rather than silently coerced if it ever appears.
    pub fn decode(p: &[u8]) -> Result<Self, PayloadError> {
        if p.len() != Self::LEN {
            return Err(PayloadError::WrongLength {
                msg: MsgId::ReadConfig,
                want: Self::LEN,
                payload: p.to_vec(),
            });
        }
        let flags = p[0];
        let role =
            Role::from_bits(flags).ok_or(PayloadError::ReservedSelector { field: "role" })?;
        let plca = PlcaMode::from_bits(flags >> 2)
            .ok_or(PayloadError::ReservedSelector { field: "plca" })?;
        Ok(Self {
            role,
            plca,
            node_id: p[1],
            node_count: p[2],
            max_burst_count: p[3],
            burst_timer: p[4],
            to_timer: p[5],
        })
    }

    /// Reject values the device is known to refuse, so a bad request is never sent.
    pub fn validate(&self) -> Result<(), PayloadError> {
        if self.to_timer < TO_TIMER_MIN {
            return Err(PayloadError::ToTimerTooSmall {
                got: self.to_timer,
                min: TO_TIMER_MIN,
            });
        }
        Ok(())
    }

    /// Conditions the device accepts but which make PLCA misbehave.
    ///
    /// Returned as warnings rather than errors precisely because the device does accept them: an
    /// exhaustive sweep showed `node_id` and `node_count` accept every value 0-255.
    #[must_use]
    pub fn warnings(&self) -> Vec<String> {
        let mut w = Vec::new();
        if self.node_id == 0xff {
            w.push(
                "node ID 255 means \"unconfigured\" in PLCA; the device accepts it but the PHY will not \
                 participate in a cycle"
                    .to_string(),
            );
        }
        // A switch-controlled role or PLCA setting reads back as "set by switch" rather than as the
        // value the switch resolved to, so the effective role is not knowable from the config record.
        // Warn unless the recorded values rule starvation out, and hedge the wording when they cannot
        // confirm it either. A switch-resolved node ID or node count cannot be judged at all.
        if self.role != Role::Coordinator
            && self.plca != PlcaMode::Disabled
            && self.node_id != 0xff
            && self.node_id != Self::SET_BY_SWITCH
            && self.node_count != Self::SET_BY_SWITCH
            && self.node_id >= self.node_count
        {
            let certain = self.role == Role::Follower && self.plca == PlcaMode::Enabled;
            let lead = if certain {
                format!(
                    "node ID {} is not below the node count {}",
                    self.node_id, self.node_count
                )
            } else {
                format!(
                    "node ID {} is not below the node count {}, and this configuration may resolve to a \
                     PLCA follower (role {}, plca {})",
                    self.node_id, self.node_count, self.role, self.plca
                )
            };
            w.push(format!(
                "{lead}: a follower outside the coordinator's cycle is silently starved of transmit \
                 opportunities (it still receives, and its host TX counter still increments, but nothing \
                 reaches the wire)"
            ));
        }
        // The coordinator is node 0 in 802.3cg; the role selector does not override that, so this
        // combination is a device that beacons nothing (measured with a far-end witness).
        if self.role == Role::Coordinator
            && self.plca == PlcaMode::Enabled
            && self.node_id != 0
            && self.node_id != 0xff
        {
            w.push(format!(
                "the coordinator role is selected with node ID {}, but the coordinator is node 0: \
                 this device will be a follower and will not beacon",
                self.node_id
            ));
        }
        w
    }

    /// Written fields the device did not apply, judged against a [`Status`] read after the write.
    ///
    /// Only node ID and node count are checkable, because they are the only configuration fields
    /// `ReadStatus` reports, and only when written as explicit nonzero values: a written
    /// [`Config::SET_BY_SWITCH`] asks for whatever the switches say, which the configuration record
    /// does not know, so nothing it leaves in effect can be called wrong here.
    #[must_use]
    pub fn unapplied(&self, status: &Status) -> Vec<Unapplied> {
        [
            (CfgField::NodeId, self.node_id, status.node_id),
            (CfgField::NodeCount, self.node_count, status.node_count),
        ]
        .into_iter()
        .filter(|&(_, wanted, in_effect)| wanted != Self::SET_BY_SWITCH && wanted != in_effect)
        .map(|(field, wanted, in_effect)| Unapplied {
            field,
            wanted,
            in_effect,
        })
        .collect()
    }
}

/// A configuration field that `ReadStatus` also reports, so its application can be verified.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CfgField {
    /// [`Config::node_id`] against [`Status::node_id`].
    NodeId,
    /// [`Config::node_count`] against [`Status::node_count`].
    NodeCount,
}

impl CfgField {
    /// Field name, spelled as the CLI spells it.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            CfgField::NodeId => "node ID",
            CfgField::NodeCount => "node count",
        }
    }
}

impl fmt::Display for CfgField {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

/// A written configuration field the device did not apply. See [`Config::unapplied`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct Unapplied {
    /// Which field.
    pub field: CfgField,
    /// The value that was written.
    pub wanted: u8,
    /// The value the device reports as being in effect instead.
    pub in_effect: u8,
}

impl fmt::Display for Unapplied {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} {} was not applied; {} is still in effect",
            self.field, self.wanted, self.in_effect
        )
    }
}

/// PHY test mode, `TestMode` (0x28) request payload and status `data[0]` bits 4:2. The discriminant
/// is the wire value.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize)]
#[repr(u8)]
pub enum TestMode {
    /// Normal operation.
    #[default]
    None = 0,
    /// Output voltage and jitter.
    Mode1 = 1,
    /// Output droop.
    Mode2 = 2,
    /// PSD mask.
    Mode3 = 3,
    /// High impedance.
    Mode4 = 4,
}

impl TestMode {
    /// Wire value.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }

    /// Decode a wire value; values 5-7 are undefined.
    #[must_use]
    pub const fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => TestMode::None,
            1 => TestMode::Mode1,
            2 => TestMode::Mode2,
            3 => TestMode::Mode3,
            4 => TestMode::Mode4,
            _ => return None,
        })
    }

    /// Description matching the specification's labels.
    #[must_use]
    pub const fn describe(self) -> &'static str {
        match self {
            TestMode::None => "no test mode",
            TestMode::Mode1 => "test mode 1 - output voltage/jitter",
            TestMode::Mode2 => "test mode 2 - output droop",
            TestMode::Mode3 => "test mode 3 - PSD mask",
            TestMode::Mode4 => "test mode 4 - high impedance",
        }
    }
}

impl fmt::Display for TestMode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.describe())
    }
}

/// Cable-test outcome, `CableTest` (0x27) response `data[0] & 0x07`, per the specification's table.
///
/// The specification lists the value `101` twice (fault open, then unknown); `110` is taken to be
/// unknown, which is the only value the bench has ever returned. Values 0 to 5 are **unverified**.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[repr(u8)]
pub enum CableTestResult {
    /// Open circuit.
    OpenCircuit = 0,
    /// Short circuit.
    ShortCircuit = 1,
    /// Correctly terminated.
    Termination = 2,
    /// Fault of unspecified kind.
    Fault = 3,
    /// Fault, short.
    FaultShort = 4,
    /// Fault, open.
    FaultOpen = 5,
    /// Result unknown. Observed on a healthy, terminated, actively linked cable.
    Unknown = 6,
    /// Value 7, which the specification does not name.
    Undefined = 7,
}

impl CableTestResult {
    /// Decode the low three bits of the response byte.
    #[must_use]
    pub const fn from_u8(v: u8) -> Self {
        match v & 0x07 {
            0 => CableTestResult::OpenCircuit,
            1 => CableTestResult::ShortCircuit,
            2 => CableTestResult::Termination,
            3 => CableTestResult::Fault,
            4 => CableTestResult::FaultShort,
            5 => CableTestResult::FaultOpen,
            6 => CableTestResult::Unknown,
            _ => CableTestResult::Undefined,
        }
    }

    /// Human-readable description.
    #[must_use]
    pub const fn describe(self) -> &'static str {
        match self {
            CableTestResult::OpenCircuit => "open circuit",
            CableTestResult::ShortCircuit => "short circuit",
            CableTestResult::Termination => "termination",
            CableTestResult::Fault => "fault",
            CableTestResult::FaultShort => "fault short",
            CableTestResult::FaultOpen => "fault open",
            CableTestResult::Unknown => "unknown",
            CableTestResult::Undefined => "undefined (wire value 7)",
        }
    }
}

impl fmt::Display for CableTestResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.describe())
    }
}

/// Decoded `ReadStatus` (0x25) payload.
///
/// **This message, not `ReadConfig`, is what reports reality.** `data[1]` and `data[2]` hold the
/// node ID and node count *in effect*, while `ReadConfig` (0x21) echoes the stored selectors. The
/// two legitimately differ whenever a selector is "set by switch": the record holds the sentinel,
/// status holds the switch-resolved value. (On firmware 1.1 they also differed because a written 0
/// was never applied.) Use
/// [`Config::unapplied`] after a write.
///
/// [`Status::raw`] is retained so the flag bytes can be read against the vendor's table directly.
///
/// **The diagnostic flags in `data[3]` and `data[4]` are read-to-clear**: they report events since
/// the previous read, not current state, so reading consumes the events another reader would see.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct Status {
    /// `data[0]` bit 0. The configured *selector*, not what the PHY is doing: it reads true whenever
    /// the coordinator role is selected, including when the role was never applied. On a LAN867x the
    /// coordinator is node 0, so [`Status::coordinator_in_effect`] is the question worth asking.
    pub coordinator_enabled: bool,
    /// `data[0]` bit 1. Also a selector echo, and not a liveness indicator: it reads true for a PHY
    /// that is enabled but has nothing to talk to. Only the far end can confirm PLCA is running.
    pub plca_enabled: bool,
    /// `data[0]` bits 4:2, or `None` if the field holds an undefined value.
    pub test_mode: Option<TestMode>,
    /// `data[1]`. The node ID **in effect**: the written value, or the DIP-switch value when
    /// [`Config::SET_BY_SWITCH`] was written (firmware 1.2), or 0 after `DefaultConfig` (0x24).
    pub node_id: u8,
    /// `data[2]`. The node count **in effect**, resolved the same way as [`Status::node_id`].
    /// `DefaultConfig` leaves 8 here, the LAN867x reset value.
    pub node_count: u8,
    /// `data[3]` bit 0. Never observed set.
    pub unexpected_carrier_sense: bool,
    /// `data[3]` bit 1. Never observed set.
    pub tx_collision_error: bool,
    /// `data[3]` bit 2. Never observed set.
    pub esda_error: bool,
    /// `data[3]` bit 3. A 4B/5B decode error. Observed both on a wedged data path and, occasionally,
    /// on a healthy PLCA link passing traffic with no packet loss, so an isolated occurrence is not
    /// by itself evidence of a fault.
    pub dec5b_error: bool,
    /// `data[3]` bit 4. Never observed set.
    pub tx_jabber_error: bool,
    /// `data[4]` bit 0. Never observed set.
    pub beacon_before_to: bool,
    /// `data[4]` bit 1. Observed set.
    pub unexpected_beacon_received: bool,
    /// `data[4]` bit 2. Observed set on a coordinator while a follower was transmitting in its
    /// transmit opportunity.
    pub rx_in_transmit_opportunity: bool,
    /// `data[4]` bit 3. Observed set on firmware 1.2 on a coordinator with PLCA just disabled while
    /// the follower was still transmitting.
    pub plca_symbols_detected: bool,
    /// The payload exactly as received.
    pub raw: Vec<u8>,
}

impl Status {
    /// Minimum payload length. The device is documented as "at least 5"; 5 is always observed.
    pub const MIN_LEN: usize = 5;

    /// Decode a status payload, accepting and preserving any trailing bytes.
    pub fn decode(p: &[u8]) -> Result<Self, PayloadError> {
        if p.len() < Self::MIN_LEN {
            return Err(PayloadError::WrongLength {
                msg: MsgId::ReadStatus,
                want: Self::MIN_LEN,
                payload: p.to_vec(),
            });
        }
        Ok(Self {
            coordinator_enabled: p[0] & 0x01 != 0,
            plca_enabled: p[0] & 0x02 != 0,
            test_mode: TestMode::from_u8((p[0] >> 2) & 0x07),
            node_id: p[1],
            node_count: p[2],
            unexpected_carrier_sense: p[3] & 0x01 != 0,
            tx_collision_error: p[3] & 0x02 != 0,
            esda_error: p[3] & 0x04 != 0,
            dec5b_error: p[3] & 0x08 != 0,
            tx_jabber_error: p[3] & 0x10 != 0,
            beacon_before_to: p[4] & 0x01 != 0,
            unexpected_beacon_received: p[4] & 0x02 != 0,
            rx_in_transmit_opportunity: p[4] & 0x04 != 0,
            plca_symbols_detected: p[4] & 0x08 != 0,
            raw: p.to_vec(),
        })
    }

    /// Whether this PHY is the PLCA coordinator, and therefore beaconing.
    ///
    /// The coordinator *is* node 0, so this is [`Status::node_id`] `== 0` rather than
    /// [`Status::coordinator_enabled`], which is only a selector echo. PLCA must also be enabled
    /// for beacons to appear; this answers only the node-ID half.
    #[must_use]
    pub const fn coordinator_in_effect(&self) -> bool {
        self.node_id == 0
    }

    /// Whether the device claims the coordinator role while a nonzero node ID is in effect.
    ///
    /// That combination means the PHY is a follower and is not beaconing, whatever the
    /// configuration says; the manual states the coordinator's node ID is always 0. It arises when a
    /// nonzero node ID is deliberately combined with the coordinator role, and on firmware 1.1 when
    /// a written node ID of 0 was ignored (recovery there: `DefaultConfig` then a fresh write).
    #[must_use]
    pub const fn coordinator_claim_unapplied(&self) -> bool {
        self.coordinator_enabled && !self.coordinator_in_effect()
    }

    /// Every diagnostic flag as `(name, value)`, in wire-bit order.
    #[must_use]
    pub fn flags(&self) -> [(&'static str, bool); 9] {
        [
            ("UnexpectedCarrierSense", self.unexpected_carrier_sense),
            ("TxCollisionError", self.tx_collision_error),
            ("ESDAError", self.esda_error),
            ("Dec5bError", self.dec5b_error),
            ("TxJabberError", self.tx_jabber_error),
            ("BeaconBeforeTO", self.beacon_before_to),
            ("UnexpectedBeaconReceived", self.unexpected_beacon_received),
            ("RxInTransmitOpportunity", self.rx_in_transmit_opportunity),
            ("PlcaSymbolsDetected", self.plca_symbols_detected),
        ]
    }
}

/// Device identity, assembled from the four product-information messages.
#[derive(Debug, Clone, PartialEq, Eq, Default, Serialize)]
pub struct DeviceInfo {
    /// Serial number as the vendor displays it: `data[3..0]` in reverse order, hex.
    pub serial: String,
    /// Hardware revision, `major.minor`.
    pub hardware: String,
    /// Firmware version, `major.minor`.
    pub firmware: String,
    /// MAC address, in wire order (no reversal).
    pub mac: [u8; 6],
    /// LAN8670 silicon revision code from `GetLan8670Rev` (0x29), or `None` when the firmware
    /// answers `UnknownMessageID` (firmware 1.1 predates the message). See [`phy_revision_name`].
    pub phy_revision: Option<u8>,
}

/// Decode `ReadSerialNumber` (0x11).
///
/// The vendor renders the four bytes in reverse order as hex; no arithmetic is ever done on the
/// value, so this is a display convention rather than an endianness claim.
pub fn decode_serial(p: &[u8]) -> Result<String, PayloadError> {
    if p.len() != 4 {
        return Err(PayloadError::WrongLength {
            msg: MsgId::ReadSerialNumber,
            want: 4,
            payload: p.to_vec(),
        });
    }
    Ok(format!("{:02X}{:02X}{:02X}{:02X}", p[3], p[2], p[1], p[0]))
}

/// Decode `ReadHwInfo` (0x12) as the `major.minor` string the vendor displays.
///
/// Only `data[5]` and `data[4]` contribute: payload `0c 00 02 00 02 00` displays as "0.2".
/// **Unverified:** the meaning of the other four bytes.
pub fn decode_hw_info(p: &[u8]) -> Result<String, PayloadError> {
    if p.len() != 6 {
        return Err(PayloadError::WrongLength {
            msg: MsgId::ReadHwInfo,
            want: 6,
            payload: p.to_vec(),
        });
    }
    Ok(format!("{}.{}", p[5], p[4]))
}

/// Firmware version as `ReadSwInfo` (0x13) reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FwVersion {
    /// Major version, `data[1]`.
    pub major: u8,
    /// Minor version, `data[0]`.
    pub minor: u8,
}

impl FwVersion {
    /// Firmware 1.1 stored a written node ID or node count of 0 without applying it, never applied
    /// the DIP switches, and lost its saved configuration on power cycle. Firmware 1.2 fixed all
    /// three.
    #[must_use]
    pub const fn has_1_1_faults(self) -> bool {
        self.major == 1 && self.minor == 1
    }
}

impl fmt::Display for FwVersion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}.{}", self.major, self.minor)
    }
}

/// Decode `ReadSwInfo` (0x13).
///
/// `data[0]` is the minor and `data[1]` the major, per the vendor's specification. Firmware 1.2
/// answers `02 01`, which confirms the order the 1.1 sample (`01 01`) could not.
pub fn decode_sw_version(p: &[u8]) -> Result<FwVersion, PayloadError> {
    match p {
        [minor, major] => Ok(FwVersion {
            major: *major,
            minor: *minor,
        }),
        _ => Err(PayloadError::WrongLength {
            msg: MsgId::ReadSwInfo,
            want: 2,
            payload: p.to_vec(),
        }),
    }
}

/// Decode `ReadMacAddress` (0x1B). Bytes are in wire order, not reversed.
pub fn decode_mac(p: &[u8]) -> Result<[u8; 6], PayloadError> {
    let arr: [u8; 6] = p.try_into().map_err(|_| PayloadError::WrongLength {
        msg: MsgId::ReadMacAddress,
        want: 6,
        payload: p.to_vec(),
    })?;
    Ok(arr)
}

/// Decode `GetLan8670Rev` (0x29): the one-byte silicon revision code. See [`phy_revision_name`].
pub fn decode_phy_revision(p: &[u8]) -> Result<u8, PayloadError> {
    match p {
        [code] => Ok(*code),
        _ => Err(PayloadError::WrongLength {
            msg: MsgId::GetLan8670Rev,
            want: 1,
            payload: p.to_vec(),
        }),
    }
}

/// The LAN8670 revision a `GetLan8670Rev` code names, per the vendor's table. Codes outside it
/// are reserved for other or future silicon.
#[must_use]
pub const fn phy_revision_name(code: u8) -> Option<&'static str> {
    Some(match code {
        0x02 => "B1",
        0x03 => "C0",
        0x04 => "C1",
        0x05 => "C2",
        0x06 => "D0",
        _ => return None,
    })
}

/// Format a MAC address in the conventional colon-separated lowercase form.
#[must_use]
pub fn format_mac(mac: &[u8; 6]) -> String {
    mac.chunks(1).map(hex::encode).collect::<Vec<_>>().join(":")
}

/// Device-reported error codes, carried in `GeneralError` (0xFF) `data[0]`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ErrorCode {
    /// 0xA0. Never observed on this product.
    IncorrectByte,
    /// 0xA1. Observed, by sending a deliberately wrong checksum.
    BadChecksum,
    /// 0xA2. Observed, by sending message ID 0x99.
    UnknownMessageId,
    /// 0xA3. Observed, via a truncated frame that corrupted the next frame's length.
    IncorrectDataLength,
    /// 0xA4. Never observed; out-of-range payload values yield `ConfigurationError` instead.
    InvalidData,
    /// 0xF0. Observed, from a TO timer below 29 and from an invalid test mode.
    ConfigurationError,
    /// 0xF1. No reachable trigger on this product.
    ChannelIsRunning,
    /// 0xF2. No reachable trigger on this product.
    InvalidChannel,
    /// 0xF3. No reachable trigger on this product.
    ChannelIsNotRunning,
    /// 0xF4. No reachable trigger on this product.
    HardwareFifoIsFull,
    /// A code not present in the specification's error table.
    Other(u8),
}

impl ErrorCode {
    /// Decode a code byte.
    #[must_use]
    pub const fn from_u8(v: u8) -> Self {
        match v {
            0xA0 => ErrorCode::IncorrectByte,
            0xA1 => ErrorCode::BadChecksum,
            0xA2 => ErrorCode::UnknownMessageId,
            0xA3 => ErrorCode::IncorrectDataLength,
            0xA4 => ErrorCode::InvalidData,
            0xF0 => ErrorCode::ConfigurationError,
            0xF1 => ErrorCode::ChannelIsRunning,
            0xF2 => ErrorCode::InvalidChannel,
            0xF3 => ErrorCode::ChannelIsNotRunning,
            0xF4 => ErrorCode::HardwareFifoIsFull,
            other => ErrorCode::Other(other),
        }
    }

    /// The code byte.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        match self {
            ErrorCode::IncorrectByte => 0xA0,
            ErrorCode::BadChecksum => 0xA1,
            ErrorCode::UnknownMessageId => 0xA2,
            ErrorCode::IncorrectDataLength => 0xA3,
            ErrorCode::InvalidData => 0xA4,
            ErrorCode::ConfigurationError => 0xF0,
            ErrorCode::ChannelIsRunning => 0xF1,
            ErrorCode::InvalidChannel => 0xF2,
            ErrorCode::ChannelIsNotRunning => 0xF3,
            ErrorCode::HardwareFifoIsFull => 0xF4,
            ErrorCode::Other(v) => v,
        }
    }

    /// Short name for the specification's error description.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            ErrorCode::IncorrectByte => "IncorrectByte",
            ErrorCode::BadChecksum => "BadChecksum",
            ErrorCode::UnknownMessageId => "UnknownMessageID",
            ErrorCode::IncorrectDataLength => "IncorrectDataLength",
            ErrorCode::InvalidData => "InvalidData",
            ErrorCode::ConfigurationError => "ConfigurationError",
            ErrorCode::ChannelIsRunning => "ChannelIsRunning",
            ErrorCode::InvalidChannel => "InvalidChannel",
            ErrorCode::ChannelIsNotRunning => "ChannelIsNotRunning",
            ErrorCode::HardwareFifoIsFull => "HardwareFIFOIsFull",
            ErrorCode::Other(_) => "unrecognised error code",
        }
    }
}

impl fmt::Display for ErrorCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} ({:#04x})", self.name(), self.as_u8())
    }
}

/// A decoded `GeneralError` reply.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
#[error("device reported {code} in reply to {}", request_name(*related_id))]
pub struct DeviceError {
    /// What went wrong.
    pub code: ErrorCode,
    /// The ID byte of the request that failed.
    pub related_id: u8,
}

impl DeviceError {
    /// Decode a `GeneralError` payload. Two bytes are always observed; extra bytes are ignored.
    pub fn decode(p: &[u8]) -> Result<Self, PayloadError> {
        if p.len() < 2 {
            return Err(PayloadError::WrongLength {
                msg: MsgId::GeneralError,
                want: 2,
                payload: p.to_vec(),
            });
        }
        Ok(Self {
            code: ErrorCode::from_u8(p[0]),
            related_id: p[1],
        })
    }
}

/// Render a request ID as its message name, falling back to hex for an unassigned value.
#[must_use]
pub fn request_name(id_raw: u8) -> String {
    MsgId::from_u8(id_raw).map_or_else(|| format!("{id_raw:#04x}"), |m| m.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Real frames captured from the device and from the vendor utility talking to it.
    /// Each entry is (description, whole frame bytes).
    const REAL_FRAMES: &[(&str, &[u8])] = &[
        ("ReadSWInfo request", &[0x02, 0x13, 0x00, 0x00, 0x13, 0x03]),
        (
            "ReadSWInfo reply 1.1",
            &[0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03],
        ),
        ("ReadCfg request", &[0x02, 0x21, 0x00, 0x00, 0x21, 0x03]),
        (
            "ReadCfg reply coordinator",
            &[
                0x02, 0x21, 0x06, 0x00, 0x0a, 0x00, 0x02, 0x00, 0x80, 0x20, 0xd3, 0x03,
            ],
        ),
        (
            "ReadCfg reply follower nid7",
            &[
                0x02, 0x21, 0x06, 0x00, 0x09, 0x07, 0x03, 0x00, 0x80, 0x20, 0xda, 0x03,
            ],
        ),
        (
            "ReadStatus reply flags clear",
            &[
                0x02, 0x25, 0x05, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x2f, 0x03,
            ],
        ),
        (
            "ReadStatus reply unexpected beacon",
            &[
                0x02, 0x25, 0x05, 0x00, 0x03, 0x01, 0x02, 0x00, 0x02, 0x32, 0x03,
            ],
        ),
        (
            "ReadStatus reply dec5b",
            &[
                0x02, 0x25, 0x05, 0x00, 0x03, 0x01, 0x02, 0x08, 0x00, 0x38, 0x03,
            ],
        ),
        (
            "ReadSerialNumber reply",
            &[0x02, 0x11, 0x04, 0x00, 0x04, 0x01, 0x02, 0x0c, 0x28, 0x03],
        ),
        (
            "ReadHWInfo reply",
            &[
                0x02, 0x12, 0x06, 0x00, 0x0c, 0x00, 0x02, 0x00, 0x02, 0x00, 0x28, 0x03,
            ],
        ),
        (
            "ReadMAC reply",
            &[
                0x02, 0x1b, 0x06, 0x00, 0xfc, 0x61, 0x79, 0x90, 0x04, 0x56, 0xe1, 0x03,
            ],
        ),
        (
            "ReadSQI reply 0",
            &[0x02, 0x26, 0x01, 0x00, 0x00, 0x27, 0x03],
        ),
        (
            "CableTest reply 6",
            &[0x02, 0x27, 0x01, 0x00, 0x06, 0x2e, 0x03],
        ),
        (
            "WriteCfg request stx/etx laden",
            &[
                0x02, 0x20, 0x06, 0x00, 0x09, 0x02, 0x03, 0x02, 0x03, 0x02, 0x3b, 0x03,
            ],
        ),
        ("WriteCfg ack", &[0x02, 0x20, 0x00, 0x00, 0x20, 0x03]),
        ("SaveCfg ack", &[0x02, 0x22, 0x00, 0x00, 0x22, 0x03]),
        (
            "GeneralError config error",
            &[0x02, 0xff, 0x02, 0x00, 0xf0, 0x20, 0x11, 0x03],
        ),
        (
            "GeneralError unknown id",
            &[0x02, 0xff, 0x02, 0x00, 0xa2, 0x99, 0x3c, 0x03],
        ),
    ];

    #[test]
    fn hand_picked_frames_validate_and_round_trip() {
        for (what, bytes) in REAL_FRAMES {
            let events = parse_all(bytes);
            assert_eq!(events.len(), 1, "{what}: expected exactly one event");
            let ParseEvent::Frame(f) = &events[0] else {
                panic!("{what}: did not parse as a valid frame: {:?}", events[0]);
            };
            assert_eq!(
                &f.encode(),
                bytes,
                "{what}: re-encode differs from the wire bytes"
            );
            assert_eq!(
                bytes.len(),
                f.payload.len() + FRAME_OVERHEAD,
                "{what}: total != len + 6"
            );
        }
    }

    #[test]
    fn checksum_is_id_plus_len_plus_payload() {
        // 0x21 reply: 0x21 + 6 + 0x0a + 0x00 + 0x02 + 0x00 + 0x80 + 0x20 = 0xd3.
        assert_eq!(checksum(0x21, &[0x0a, 0x00, 0x02, 0x00, 0x80, 0x20]), 0xd3);
        // 0x25 reply: 0x25 + 5 + 0x03 + 0x01 + 0x02 + 0x08 + 0x00 = 0x38.
        assert_eq!(checksum(0x25, &[0x03, 0x01, 0x02, 0x08, 0x00]), 0x38);
        // 0xff error: 0xff + 2 + 0xf0 + 0x20 = 0x211 -> 0x11.
        assert_eq!(checksum(0xff, &[0xf0, 0x20]), 0x11);
    }

    #[test]
    fn zero_length_frame_checksum_equals_id() {
        for id in [0x13u8, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25] {
            assert_eq!(checksum(id, &[]), id);
            let f = encode(id, &[]);
            assert_eq!(f, vec![STX, id, 0x00, 0x00, id, ETX]);
        }
    }

    #[test]
    fn summing_length_bytes_separately_would_differ_at_256_bytes() {
        // Both rules agree while len_hi is zero, which is why the captures cannot distinguish them.
        let payload = vec![0u8; 256];
        let correct = checksum(0x20, &payload);
        let wrong = {
            let len = payload.len();
            let lo = (len & 0xff) as u32;
            let hi = ((len >> 8) & 0xff) as u32;
            ((u32::from(0x20u8) + lo + hi + payload.iter().map(|&b| u32::from(b)).sum::<u32>())
                & 0xff) as u8
        };
        assert_ne!(
            correct, wrong,
            "the two rules must diverge at 256 payload bytes"
        );
    }

    #[test]
    fn payload_containing_stx_and_etx_round_trips() {
        let frame = &[
            0x02, 0x20, 0x06, 0x00, 0x09, 0x02, 0x03, 0x02, 0x03, 0x02, 0x3b, 0x03,
        ];
        let events = parse_all(frame);
        assert_eq!(events.len(), 1);
        let ParseEvent::Frame(f) = &events[0] else {
            panic!("expected a frame")
        };
        // The payload really does contain raw STX and ETX bytes; no unescaping happened.
        assert_eq!(f.payload, vec![0x09, 0x02, 0x03, 0x02, 0x03, 0x02]);
        assert!(f.payload.contains(&STX) && f.payload.contains(&ETX));
        assert_eq!(&f.encode(), frame);
    }

    #[test]
    fn two_concatenated_frames_yield_two_frames() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&[0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03]);
        buf.extend_from_slice(&[
            0x02, 0x25, 0x05, 0x00, 0x03, 0x00, 0x02, 0x00, 0x00, 0x2f, 0x03,
        ]);
        let events = parse_all(&buf);
        assert_eq!(events.len(), 2);
        let ids: Vec<_> = events
            .iter()
            .map(|e| match e {
                ParseEvent::Frame(f) => f.id_raw,
                other => panic!("unexpected {other:?}"),
            })
            .collect();
        assert_eq!(ids, vec![0x13, 0x25]);
    }

    #[test]
    fn frame_split_across_feeds_parses_when_remainder_arrives() {
        let whole = [
            0x02, 0x21, 0x06, 0x00, 0x0a, 0x00, 0x02, 0x00, 0x80, 0x20, 0xd3, 0x03,
        ];
        for split in 1..whole.len() {
            let mut p = Parser::new();
            assert!(
                p.feed(&whole[..split]).is_empty(),
                "premature event at split {split}"
            );
            let events = p.feed(&whole[split..]);
            assert_eq!(events.len(), 1, "no frame after remainder at split {split}");
            let ParseEvent::Frame(f) = &events[0] else {
                panic!("expected a frame")
            };
            assert_eq!(f.id_raw, 0x21);
        }
    }

    #[test]
    fn reply_dribbled_one_byte_at_a_time_parses() {
        let whole = [0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03];
        let mut p = Parser::new();
        let mut got = Vec::new();
        for b in whole {
            got.extend(p.feed(&[b]));
        }
        assert_eq!(got.len(), 1);
    }

    #[test]
    fn leading_garbage_is_skipped() {
        let mut buf = vec![0xde, 0xad, 0xbe, 0xef];
        buf.extend_from_slice(&[0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03]);
        let events = parse_all(&buf);
        assert_eq!(events.len(), 1);
        let ParseEvent::Frame(f) = &events[0] else {
            panic!("expected a frame")
        };
        assert_eq!(f.id_raw, 0x13);
        assert_eq!(f.payload, vec![0x01, 0x01]);
    }

    #[test]
    fn bad_checksum_is_reported_and_parser_resynchronises() {
        let mut buf = vec![0x02, 0x13, 0x00, 0x00, 0x00, 0x03]; // checksum should be 0x13
        buf.extend_from_slice(&[0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03]);
        let events = parse_all(&buf);
        assert!(
            matches!(
                events[0],
                ParseEvent::Invalid(FrameError::BadChecksum {
                    got: 0x00,
                    want: 0x13
                })
            ),
            "first event should be a checksum failure, got {:?}",
            events[0]
        );
        let last = events.last().expect("at least one event");
        assert!(
            matches!(last, ParseEvent::Frame(f) if f.id_raw == 0x13 && f.payload == vec![0x01, 0x01]),
            "the following good frame should still be found, got {last:?}"
        );
    }

    #[test]
    fn bad_end_byte_is_reported_and_parser_resynchronises() {
        // Checksum-valid but ETX-invalid: the ETX check is the last line of defence against a
        // false header that happens to checksum, so it must be exercised.
        let mut bad = vec![0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0xff];
        bad.extend_from_slice(&[0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03]);
        let events = parse_all(&bad);
        assert!(
            events
                .iter()
                .any(|e| matches!(e, ParseEvent::Invalid(FrameError::BadEtx { got: 0xff }))),
            "expected a BadEtx event, got {events:?}"
        );
        assert!(
            events
                .iter()
                .any(|e| matches!(e, ParseEvent::Frame(f) if f.payload == vec![0x01, 0x01])),
            "the following good frame should still be recovered, got {events:?}"
        );
    }

    #[test]
    fn starvation_warning_is_not_silent_when_the_role_is_switch_controlled() {
        // A switch-selected role reads back as SetBySwitch, so the effective role is unknowable
        // from the record. Staying silent here would hide the exact failure the warning exists for.
        let switched = Config {
            role: Role::SetBySwitch,
            plca: PlcaMode::SetBySwitch,
            node_id: 5,
            node_count: 2,
            ..Config::default()
        };
        let w = switched.warnings();
        assert!(
            w.iter().any(|m| m.contains("starved")),
            "a switch-controlled role must still warn, got {w:?}"
        );
        assert!(
            w.iter().any(|m| m.contains("may resolve")),
            "the wording should hedge when the role is not knowable, got {w:?}"
        );

        // node_count 0 is the spec's "set by DIP switch" sentinel, not a cycle of zero slots, so
        // there is no count to compare the node ID against.
        let zero_count = Config {
            role: Role::Follower,
            plca: PlcaMode::Enabled,
            node_id: 3,
            node_count: Config::SET_BY_SWITCH,
            ..Config::default()
        };
        assert!(
            !zero_count.warnings().iter().any(|m| m.contains("starved")),
            "a switch-resolved node count cannot be judged, got {:?}",
            zero_count.warnings()
        );

        // A coordinator is never starved, and an explicitly disabled PLCA cannot starve anyone.
        let coord = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_id: 0,
            node_count: 2,
            ..Config::default()
        };
        assert!(
            !coord.warnings().iter().any(|m| m.contains("starved")),
            "a coordinator should not get a starvation warning"
        );
        let off = Config {
            plca: PlcaMode::Disabled,
            node_id: 5,
            node_count: 2,
            ..Config::default()
        };
        assert!(
            !off.warnings().iter().any(|m| m.contains("starved")),
            "PLCA off cannot starve anyone"
        );
    }

    #[test]
    fn firmware_version_uses_the_documented_byte_order() {
        // data[0] is the minor and data[1] the major, per the specification. An asymmetric sample
        // is the only way to pin the order down; the real 01 01 capture cannot.
        assert_eq!(
            decode_sw_version(&[0x01, 0x02])
                .expect("decodes")
                .to_string(),
            "2.1"
        );
        // The real captured sample stays correct either way.
        assert_eq!(
            decode_sw_version(&[0x01, 0x01])
                .expect("decodes")
                .to_string(),
            "1.1"
        );
    }

    #[test]
    fn implausible_length_resynchronises_rather_than_stalling() {
        // A false start-of-frame claiming a 512-byte payload, then a real frame.
        let mut buf = vec![0x02, 0x13, 0x00, 0x02];
        buf.extend_from_slice(&[0x02, 0x13, 0x02, 0x00, 0x01, 0x01, 0x17, 0x03]);
        let events = parse_all(&buf);
        assert!(
            events.iter().any(|e| matches!(
                e,
                ParseEvent::Invalid(FrameError::LengthTooLarge { len: 512 })
            )),
            "expected a LengthTooLarge event, got {events:?}"
        );
        assert!(
            events
                .iter()
                .any(|e| matches!(e, ParseEvent::Frame(f) if f.id_raw == 0x13)),
            "expected the real frame to still be recovered, got {events:?}"
        );
    }

    #[test]
    fn reset_clears_a_partial_frame() {
        let mut p = Parser::new();
        assert!(p.feed(&[0x02, 0x21, 0x06, 0x00, 0x0a]).is_empty());
        assert!(p.buffered() > 0);
        p.reset();
        assert_eq!(p.buffered(), 0);
    }

    #[test]
    fn config_decodes_the_captured_coordinator_payload() {
        // Captured 0a 00 02 00 80 20, which the vendor utility decoded as
        // role=Coordinator plca=Enabled nodeId=0 nodeCount=2 toTimer=32 burstCount=0 burstTimer=128
        let c = Config::decode(&[0x0a, 0x00, 0x02, 0x00, 0x80, 0x20]).expect("decodes");
        assert_eq!(c.role, Role::Coordinator);
        assert_eq!(c.plca, PlcaMode::Enabled);
        assert_eq!(c.node_id, 0);
        assert_eq!(c.node_count, 2);
        assert_eq!(c.max_burst_count, 0);
        assert_eq!(c.burst_timer, 128);
        assert_eq!(c.to_timer, 32);
        assert_eq!(c.encode(), vec![0x0a, 0x00, 0x02, 0x00, 0x80, 0x20]);
    }

    #[test]
    fn default_config_is_the_device_reset_record_and_passes_validation() {
        // 0x24 leaves `00 00 00 00 80 20` (measured on firmware 1.2).
        let reset = Config::decode(&[0x00, 0x00, 0x00, 0x00, 0x80, 0x20]).expect("decodes");
        assert_eq!(Config::default(), reset);
        assert!(
            Config::default().validate().is_ok(),
            "a default must be sendable"
        );
    }

    #[test]
    fn role_and_plca_serialise_with_the_cli_spelling() {
        // One spelling per value, whether it comes out of `name()` or serde.
        for (r, want) in [
            (Role::SetBySwitch, "switch"),
            (Role::Follower, "follower"),
            (Role::Coordinator, "coordinator"),
        ] {
            assert_eq!(r.name(), want);
            assert_eq!(
                serde_json::to_string(&r).expect("json"),
                format!("\"{want}\"")
            );
        }
        for (m, want) in [
            (PlcaMode::SetBySwitch, "switch"),
            (PlcaMode::Disabled, "off"),
            (PlcaMode::Enabled, "on"),
        ] {
            assert_eq!(m.name(), want);
            assert_eq!(
                serde_json::to_string(&m).expect("json"),
                format!("\"{want}\"")
            );
        }
    }

    #[test]
    fn config_decodes_the_captured_follower_and_switch_payloads() {
        // session1: 09 07 03 00 80 20 -> Follower + Enabled, node 7 of 3, TO timer 32.
        let f = Config::decode(&[0x09, 0x07, 0x03, 0x00, 0x80, 0x20]).expect("decodes");
        assert_eq!((f.role, f.plca), (Role::Follower, PlcaMode::Enabled));
        assert_eq!((f.node_id, f.node_count, f.to_timer), (7, 3, 32));
        // session1: 00 01 02 01 40 28 -> both selectors SetBySwitch, burst count 1, TO timer 40.
        let s = Config::decode(&[0x00, 0x01, 0x02, 0x01, 0x40, 0x28]).expect("decodes");
        assert_eq!((s.role, s.plca), (Role::SetBySwitch, PlcaMode::SetBySwitch));
        assert_eq!(
            (s.max_burst_count, s.burst_timer, s.to_timer),
            (1, 0x40, 40)
        );
        // session1: 0a 00 05 03 c8 31 -> burst timer 200, TO timer 49 (TO timer really is last).
        let b = Config::decode(&[0x0a, 0x00, 0x05, 0x03, 0xc8, 0x31]).expect("decodes");
        assert_eq!((b.max_burst_count, b.burst_timer, b.to_timer), (3, 200, 49));
    }

    #[test]
    fn flags_reserved_nibble_is_written_as_zero_matching_device_readback() {
        // Measured: writing 0xfa reads back 0x0a, 0x55 reads back 0x05, 0xf0 reads 0x00.
        for (written, reads_back) in [(0xfau8, 0x0au8), (0x55, 0x05), (0xf0, 0x00), (0xaa, 0x0a)] {
            let c = Config::decode(&[written, 0, 2, 0, 128, 32]).expect("low nibble is valid");
            assert_eq!(
                c.flags_byte(),
                reads_back,
                "flags 0x{written:02x} should normalise to 0x{reads_back:02x}"
            );
        }
    }

    #[test]
    fn reserved_selector_value_three_is_rejected() {
        // The device answers GeneralError(ConfigurationError) for these, so refuse them locally.
        assert_eq!(
            Config::decode(&[0x03, 0, 2, 0, 128, 32]),
            Err(PayloadError::ReservedSelector { field: "role" })
        );
        assert_eq!(
            Config::decode(&[0x0c, 0, 2, 0, 128, 32]),
            Err(PayloadError::ReservedSelector { field: "plca" })
        );
        assert!(Role::from_bits(3).is_none());
        assert!(PlcaMode::from_bits(3).is_none());
    }

    #[test]
    fn to_timer_below_29_is_rejected_locally() {
        // Sweep result: 0-28 rejected by the device, 29-255 accepted.
        for bad in 0..TO_TIMER_MIN {
            let c = Config {
                to_timer: bad,
                ..Config::default()
            };
            assert_eq!(
                c.validate(),
                Err(PayloadError::ToTimerTooSmall {
                    got: bad,
                    min: TO_TIMER_MIN
                }),
                "TO timer {bad} should be refused"
            );
        }
        for ok in [TO_TIMER_MIN, 32, 200, 255] {
            let c = Config {
                to_timer: ok,
                ..Config::default()
            };
            assert!(c.validate().is_ok(), "TO timer {ok} should be accepted");
        }
    }

    #[test]
    fn coordinator_role_with_nonzero_node_id_warns() {
        // The coordinator is node 0; this combination is a measured non-beaconing state.
        let contradiction = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_id: 5,
            node_count: 8,
            ..Config::default()
        };
        let w = contradiction.warnings();
        assert!(
            w.iter().any(|m| m.contains("coordinator is node 0")),
            "coordinator + nonzero node ID should warn, got {w:?}"
        );

        // A real coordinator (node ID 0) is not a contradiction.
        let real = Config {
            node_id: 0,
            ..contradiction
        };
        assert!(
            !real
                .warnings()
                .iter()
                .any(|m| m.contains("coordinator is node 0")),
            "node ID 0 needs no contradiction warning"
        );

        // node ID 255 already has its own "unconfigured" warning; do not pile on.
        let unconfigured = Config {
            node_id: 0xff,
            ..contradiction
        };
        let w = unconfigured.warnings();
        assert!(w.iter().any(|m| m.contains("unconfigured")), "got {w:?}");
        assert!(
            !w.iter().any(|m| m.contains("coordinator is node 0")),
            "0xff is covered by the unconfigured warning, got {w:?}"
        );

        // With PLCA explicitly off nothing beacons anyway; stay quiet.
        let off = Config {
            plca: PlcaMode::Disabled,
            ..contradiction
        };
        assert!(
            !off.warnings()
                .iter()
                .any(|m| m.contains("coordinator is node 0")),
            "PLCA off has nothing to beacon"
        );
    }

    #[test]
    fn switch_resolved_node_count_on_a_coordinator_is_not_a_warning() {
        // Measured on firmware 1.2: coordinator,
        // PLCA on, node count 0 put the DIP count 3 in effect. Nothing to warn about.
        let c = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_id: 0,
            node_count: Config::SET_BY_SWITCH,
            ..Config::default()
        };
        assert_eq!(c.warnings(), Vec::<String>::new());
    }

    #[test]
    fn starved_follower_warns_without_being_rejected() {
        let c = Config {
            role: Role::Follower,
            plca: PlcaMode::Enabled,
            node_id: 3,
            node_count: 2,
            ..Config::default()
        };
        assert!(
            c.validate().is_ok(),
            "the device accepts this, so we must not reject it"
        );
        let w = c.warnings();
        assert!(
            w.iter().any(|m| m.contains("starved")),
            "expected a starvation warning, got {w:?}"
        );
    }

    #[test]
    fn status_decodes_captured_samples() {
        let clear = Status::decode(&[0x03, 0x00, 0x02, 0x00, 0x00]).expect("decodes");
        assert!(clear.coordinator_enabled && clear.plca_enabled);
        assert_eq!(clear.test_mode, Some(TestMode::None));
        assert_eq!((clear.node_id, clear.node_count), (0, 2));
        assert!(
            clear.flags().iter().all(|(_, v)| !v),
            "no flag should be set"
        );

        let beacon = Status::decode(&[0x03, 0x01, 0x02, 0x00, 0x02]).expect("decodes");
        assert!(beacon.unexpected_beacon_received);
        assert_eq!(
            beacon.flags().iter().filter(|(_, v)| *v).count(),
            1,
            "only UnexpectedBeaconReceived should be set"
        );

        let dec5b = Status::decode(&[0x03, 0x01, 0x02, 0x08, 0x00]).expect("decodes");
        assert!(dec5b.dec5b_error);
        assert_eq!(dec5b.flags().iter().filter(|(_, v)| *v).count(), 1);

        // A follower write cleared the coordinator bit: byte 0 went 0x03 -> 0x02.
        let follower = Status::decode(&[0x02, 0x01, 0x02, 0x00, 0x00]).expect("decodes");
        assert!(!follower.coordinator_enabled && follower.plca_enabled);

        // Raw bytes are always retained, since several field names are unverified.
        assert_eq!(follower.raw, vec![0x02, 0x01, 0x02, 0x00, 0x00]);
    }

    #[test]
    fn coordinator_in_effect_means_node_id_zero_not_the_coordinator_bit() {
        // Status bytes measured on firmware 1.1 hardware.
        let stuck = Status::decode(&[0x03, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        assert!(stuck.coordinator_enabled, "byte 0 claims the role");
        assert!(
            !stuck.coordinator_in_effect(),
            "node ID 5 is in effect, so this PHY is a follower and does not beacon"
        );
        assert!(stuck.coordinator_claim_unapplied());

        // Same configuration reached via DefaultConfig first, so the id really is 0 (step 2).
        let real = Status::decode(&[0x03, 0x00, 0x08, 0x00, 0x00]).expect("decodes");
        assert!(real.coordinator_in_effect());
        assert!(!real.coordinator_claim_unapplied());

        // A follower is not a coordinator whatever its node ID, so there is nothing to contradict.
        let follower = Status::decode(&[0x02, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        assert!(!follower.coordinator_claim_unapplied());
    }

    #[test]
    fn a_written_zero_is_switch_resolved_so_it_is_never_reported_unapplied() {
        assert_eq!(Config::SET_BY_SWITCH, 0, "the spec's sentinel is 0x00");
        let want_coordinator = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_id: Config::SET_BY_SWITCH,
            node_count: 8,
            max_burst_count: 0,
            burst_timer: 128,
            to_timer: 32,
        };
        // Whatever is in effect after a written 0 came from the switches (firmware 1.2) or was
        // left alone (firmware 1.1); either way it is not a value this write asked for.
        let five = Status::decode(&[0x03, 0x05, 0x08, 0x00, 0x00]).expect("decodes");
        assert_eq!(want_coordinator.unapplied(&five), vec![]);

        let want_zero_count = Config {
            node_count: Config::SET_BY_SWITCH,
            ..want_coordinator
        };
        let one = Status::decode(&[0x03, 0x00, 0x01, 0x00, 0x00]).expect("decodes");
        assert_eq!(want_zero_count.unapplied(&one), vec![]);
    }

    #[test]
    fn a_nonzero_value_that_did_not_land_is_reported_unapplied_in_wire_order() {
        let want = Config {
            role: Role::Follower,
            plca: PlcaMode::Enabled,
            node_id: 5,
            node_count: 6,
            max_burst_count: 0,
            burst_timer: 128,
            to_timer: 32,
        };
        let status = Status::decode(&[0x02, 0x03, 0x04, 0x00, 0x00]).expect("decodes");
        assert_eq!(
            want.unapplied(&status),
            vec![
                Unapplied {
                    field: CfgField::NodeId,
                    wanted: 5,
                    in_effect: 3,
                },
                Unapplied {
                    field: CfgField::NodeCount,
                    wanted: 6,
                    in_effect: 4,
                },
            ]
        );
    }

    #[test]
    fn a_fully_applied_write_reports_nothing_unapplied() {
        let applied = Config {
            role: Role::Coordinator,
            plca: PlcaMode::Enabled,
            node_id: 0,
            node_count: 8,
            max_burst_count: 0,
            burst_timer: 128,
            to_timer: 32,
        };
        let status = Status::decode(&[0x03, 0x00, 0x08, 0x00, 0x00]).expect("decodes");
        assert!(applied.unapplied(&status).is_empty());

        // Nonzero values are applied even with PLCA disabled, so that must not be flagged either:
        // measured with flags 0x05, node ID 4, node count 6 (status 00 04 06).
        let plca_off = Config {
            role: Role::Follower,
            plca: PlcaMode::Disabled,
            node_id: 4,
            node_count: 6,
            ..applied
        };
        let off_status = Status::decode(&[0x00, 0x04, 0x06, 0x00, 0x00]).expect("decodes");
        assert!(plca_off.unapplied(&off_status).is_empty());
    }

    #[test]
    fn status_accepts_trailing_bytes_but_rejects_short_payloads() {
        assert!(Status::decode(&[0x03, 0x00, 0x02, 0x00, 0x00, 0x99]).is_ok());
        assert_eq!(
            Status::decode(&[0x03, 0x00, 0x02, 0x00]),
            Err(PayloadError::WrongLength {
                msg: MsgId::ReadStatus,
                want: 5,
                payload: vec![0x03, 0x00, 0x02, 0x00],
            })
        );
    }

    #[test]
    fn general_error_decodes_captured_replies() {
        let e = DeviceError::decode(&[0xf0, 0x20]).expect("decodes");
        assert_eq!(e.code, ErrorCode::ConfigurationError);
        assert_eq!(e.related_id, MsgId::WriteConfig.as_u8());
        assert!(e.to_string().contains("WriteDeviceConfiguration"));

        assert_eq!(
            DeviceError::decode(&[0xa2, 0x99]).expect("decodes").code,
            ErrorCode::UnknownMessageId
        );
        assert_eq!(
            DeviceError::decode(&[0xa1, 0x13]).expect("decodes").code,
            ErrorCode::BadChecksum
        );
        assert_eq!(
            DeviceError::decode(&[0xa3, 0x13]).expect("decodes").code,
            ErrorCode::IncorrectDataLength
        );
    }

    #[test]
    fn product_info_decodes_captured_payloads() {
        assert_eq!(
            decode_serial(&[0x04, 0x01, 0x02, 0x0c]).expect("decodes"),
            "0C020104"
        );
        assert_eq!(
            decode_sw_version(&[0x01, 0x01])
                .expect("decodes")
                .to_string(),
            "1.1"
        );
        let hw = decode_hw_info(&[0x0c, 0x00, 0x02, 0x00, 0x02, 0x00]).expect("decodes");
        assert_eq!(hw, "0.2");
        let mac = decode_mac(&[0xfc, 0x61, 0x79, 0x90, 0x04, 0x56]).expect("decodes");
        assert_eq!(format_mac(&mac), "fc:61:79:90:04:56");
    }

    #[test]
    fn wrong_length_payloads_are_errors_not_panics() {
        assert!(decode_serial(&[0x01]).is_err());
        assert!(decode_sw_version(&[]).is_err());
        assert!(decode_hw_info(&[0x00; 5]).is_err());
        assert!(decode_mac(&[0x00; 7]).is_err());
        assert!(Config::decode(&[0x0a, 0x00]).is_err());
        assert!(DeviceError::decode(&[0xf0]).is_err());
    }

    #[test]
    fn message_ids_round_trip() {
        for &id in MsgId::ALL {
            assert_eq!(MsgId::from_u8(id.as_u8()), Some(id));
        }
        assert_eq!(MsgId::from_u8(0x99), None, "0x99 is unassigned");
        // The 2026-10-06 spec assigns these; firmware 1.2 answers 0x29 and names 0x30/0x31 as
        // UnknownMessageID on pre-D0 silicon, and 0xFE is confirmed as RESTART_BOOT.
        assert_eq!(MsgId::from_u8(0x29), Some(MsgId::GetLan8670Rev));
        assert_eq!(MsgId::from_u8(0x30), Some(MsgId::ReadDcqSqi));
        assert_eq!(MsgId::from_u8(0x31), Some(MsgId::ReadDcqSqiPlus));
        assert_eq!(MsgId::from_u8(0xfd), Some(MsgId::Restart));
        assert_eq!(MsgId::from_u8(0xfe), Some(MsgId::RestartBoot));
        assert_eq!(MsgId::ALL.len(), 20);
    }

    #[test]
    fn can_sized_payloads_parse_as_frames_not_desync() {
        // The spec's CAN side emits unsolicited CAN_RECEIVED_MESSAGE frames of up to 79 payload
        // bytes over the same VCP. With the old cap of 8 these were LengthTooLarge events.
        let payload = vec![0x5a; 79];
        let frame = encode(0x6b, &payload);
        let events = parse_all(&frame);
        assert_eq!(events.len(), 1, "got {events:?}");
        assert!(
            matches!(&events[0], ParseEvent::Frame(f) if f.id_raw == 0x6b && f.payload == payload),
            "a 79-byte CAN frame must parse, got {:?}",
            events[0]
        );
    }

    #[test]
    fn phy_revision_decodes_the_spec_table() {
        // Firmware 1.2 on the bench answered 0x29 with 0x05, which the spec names C2.
        assert_eq!(decode_phy_revision(&[0x05]).expect("decodes"), 0x05);
        for (code, name) in [
            (0x02u8, "B1"),
            (0x03, "C0"),
            (0x04, "C1"),
            (0x05, "C2"),
            (0x06, "D0"),
        ] {
            assert_eq!(phy_revision_name(code), Some(name), "code {code:#04x}");
        }
        assert_eq!(phy_revision_name(0x07), None, "unlisted codes are reserved");
        assert!(decode_phy_revision(&[]).is_err());
        assert!(decode_phy_revision(&[0x05, 0x00]).is_err());
    }

    #[test]
    fn cable_test_and_test_mode_wire_values() {
        assert_eq!(CableTestResult::from_u8(0x06), CableTestResult::Unknown);
        assert_eq!(CableTestResult::from_u8(0x02), CableTestResult::Termination);
        // Only the low three bits are significant.
        assert_eq!(CableTestResult::from_u8(0xf6), CableTestResult::Unknown);
        for (v, m) in [
            (0u8, TestMode::None),
            (1, TestMode::Mode1),
            (4, TestMode::Mode4),
        ] {
            assert_eq!(TestMode::from_u8(v), Some(m));
            assert_eq!(m.as_u8(), v);
        }
        assert_eq!(
            TestMode::from_u8(0x7f),
            None,
            "the device rejects mode 0x7f"
        );
    }
}
