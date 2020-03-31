#![no_std]

#[macro_use]
extern crate derive_try_from_primitive;

use core::num::NonZeroUsize;
use core::ops::Deref;

#[repr(u8)]
#[allow(non_camel_case_types)]
#[derive(TryFromPrimitive, Copy, Clone)]
pub enum Command {
    // General Commands
    /// Get Information about CMSIS-DAP Debug Unit.
    DAP_Info            = 0x00,
    /// Sent status information of the debugger to Debug Unit.
    DAP_HostStatus      = 0x01,
    /// Connect to Device and selected DAP mode.
    DAP_Connect         = 0x02,
    /// Disconnect from active Debug Port.
    DAP_Disconnect      = 0x03,
    /// Write ABORT Register.
    DAP_WriteABORT      = 0x08,
    /// Wait for specified delay.
    DAP_Delay           = 0x09,
    /// Reset Target with Device specific sequence.
    DAP_ResetTarget     = 0x0A,

    // Common SWD/JTAG Commands
    /// Control and monitor SWD/JTAG Pins.
    DAP_SWJ_Pins        = 0x10,
    /// Select SWD/JTAG Clock.
    DAP_SWJ_Clock       = 0x11,
    /// Generate SWJ sequence SWDIO/TMS @SWCLK/TCK.
    DAP_SWJ_Sequence    = 0x12,

    // SWD Commands
    /// Configure SWD Protocol.
    DAP_SWD_Configure   = 0x13,
    /// Generate SWD sequence and output on SWDIO or capture input from SWDIO data.
    DAP_SWD_Sequence    = 0x1D,

    // TODO: SWO Commands
    // TODO: JTAG Commands
    // TODO: Transfer Commands
    // TODO: Atomic Commands

    // DAP Vendor Command IDs
    DAP_VendorFirst     = 0x80,
    DAP_VendorLast      = 0x9F,

    // DAP Extended range of Vendor Command IDs
    DAP_VendorExFirst   = 0xA0,
    DAP_VendorExLast    = 0xFE,

    // Invalid command
    DAP_Invalid         = 0xFF,
}

#[repr(u8)]
#[allow(non_camel_case_types)]
pub enum ResponseStatus {
    DAP_OK      = 0x00,
    DAP_ERROR   = 0xFF,
}

pub fn request_length(command: Command, request: &[u8]) -> Option<NonZeroUsize> {
    assert!(!request.is_empty());
    assert_eq!(request[0], command as u8);

    use Command::*;
    let len = match command {
        DAP_Info => 2,
        DAP_HostStatus => 3,
        DAP_Connect => 2,
        DAP_Disconnect => 1,
        DAP_WriteABORT => 6,
        DAP_Delay => 3,
        DAP_ResetTarget => 1,
        DAP_SWJ_Pins => 7,
        DAP_SWJ_Clock => 5,
        DAP_SWJ_Sequence => {
            if request.len() > 2 {
                let bit_count = match request[1] {
                    0 => 256,
                    x => x as usize,
                };
                2 + (bit_count + 7) / 8
            } else {
                return None;
            }
        },
        DAP_SWD_Configure => 2,
        DAP_SWD_Sequence => {
            if request.len() >= 2 {
                let sequence_count = request[1];
                let mut offset = 2;
                for _ in 0..sequence_count {
                    if offset >= request.len() {
                        return None;
                    }
                    let sequence_info = request[offset];
                    offset += 1;

                    if sequence_info & 0x80 == 0 {
                        // Output mode
                        let cycle_count = match sequence_info & 0x2f {
                            0 => 64,
                            x => x as usize,
                        };
                        offset += (cycle_count + 7) / 8;
                    }
                }
                if offset > request.len() {
                    return None;
                }
                offset
            } else {
                return None;
            }
        },
        _ => return None,
    };
    Some(unsafe { NonZeroUsize::new_unchecked(len) })
}

/// Enum for the DAP_Info command parameter
#[repr(u8)]
#[derive(TryFromPrimitive, Copy, Clone)]
pub enum InfoId {
    /// Get the Vendor ID (string)
    VendorId            = 0x01,
    /// Get the Product ID (string)
    ProductId           = 0x02,
    /// Get the Serial Number (string)
    SerialNumber        = 0x03,
    /// Get the CMSIS-DAP Firmware Version (string)
    FirmwareVersion     = 0x04,
    /// Get the Target Device Vendor (string)
    TargetDeviceVendor  = 0x05,
    /// Get the Target Device Name (string)
    TargetDeviceName    = 0x06,
    /// Get information about the Capabilities (BYTE) of the Debug Unit
    Capabilities        = 0xF0,
    /// Get the Test Domain Timer parameter information
    TestDomainTimer     = 0xF1,
    /// Get the SWO Trace Buffer Size (WORD)
    SwoTraceBufferSize  = 0xFD,
    /// Get the maximum Packet Count (BYTE)
    PacketCount         = 0xFE,
    /// Get the maximum Packet Size (SHORT)
    PacketSize          = 0xFF,
}

pub trait ProbeInterface {
    fn dap_info(&mut self, id: InfoId);
}

pub trait Responder {
    fn reject_command(&mut self);
}

pub struct DapCommand<'a> {
    command: Command,
    payload: &'a [u8],
}

impl DapCommand<'_> {
    pub fn parse(buf: &[u8]) -> Option<DapCommand> {
        if buf.is_empty() {
            return None;
        }
        let command = Command::try_from(buf[0])?;
        let len = request_length(command, buf)?.get();
        Some(DapCommand {
            command,
            payload: &buf[1..len]
        })
    }

    pub fn command(&self) -> Command {
        self.command
    }
}

impl Deref for DapCommand<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.payload
    }
}

pub struct DapResponse<'a> {
    buffer: &'a mut [u8],
    payload_len: usize,
}

impl DapResponse<'_> {
    pub fn new(command: Command, buffer: &mut [u8]) -> DapResponse {
        assert!(!buffer.is_empty());
        buffer[0] = command as u8;
        DapResponse {
            buffer,
            payload_len: 0
        }
    }

    pub fn reject(&mut self) {
        self.buffer[0] = Command::DAP_Invalid as u8;
        self.payload_len = 0;
    }
}



// fn process_command(buf: &[u8]) {
//     if buf.is_empty() {
//         return;
//     }
//
//     let request_length;
//     if let Some(len) = request_length(buf) {
//         request_length = len.get();
//     } else {
//         return;
//     }
// }
