//! Talk to a MACH SYSTEMS 10BASET1S-USB-IF dongle over its STM32 CDC-ACM control channel.
//!
//! This crate is both a library and the [`t1s-cli`] command-line tool. The protocol it speaks is the
//! vendor's published specification, checked against wire captures and hardware probing of a real
//! dongle.
//!
//! Two layers:
//!
//! - [`proto`] is pure and IO-free: framing, checksums, an incremental length-driven parser, and
//!   every payload type. Usable for offline decoding of captures.
//! - [`device`] adds the serial transport: finding the dongle, opening the port, and running
//!   request/response transactions with the robustness the device demands.
//!
//! ```no_run
//! use std::time::Duration;
//! use mach_10base_t1s_cli::{device, proto::{Config, PlcaMode, Role}};
//!
//! let path = device::resolve_port(None)?;
//! let mut dev = device::open(&path, Duration::from_millis(1000))?;
//!
//! // Make this dongle the PLCA coordinator of a two-node segment.
//! let mut cfg = dev.read_config()?;
//! cfg.role = Role::Coordinator;
//! cfg.plca = PlcaMode::Enabled;
//! cfg.node_id = 0;
//! cfg.node_count = 2;
//! dev.write_config(&cfg)?;
//! # Ok::<(), device::Error>(())
//! ```
//!
//! [`t1s-cli`]: https://github.com/stewartallen/mach-10base-t1s-cli

pub mod device;
pub mod proto;
