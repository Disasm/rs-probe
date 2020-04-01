#![no_std]

#[macro_use]
extern crate derive_try_from_primitive;

use core::num::NonZeroUsize;
use core::ops::Deref;
use core::convert::TryInto;

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

    // SWO Commands
    /// Set SWO transport mode.
    DAP_SWO_Transport   = 0x17,
    /// Set SWO capture mode.
    DAP_SWO_Mode        = 0x18,
    ///Set SWO baudrate.
    DAP_SWO_Baudrate    = 0x19,
    /// Control SWO trace data capture.
    DAP_SWO_Control     = 0x1A,
    /// Read SWO trace status.
    DAP_SWO_Status      = 0x1B,
    /// Read SWO trace extended status.
    DAP_SWO_ExtendedStatus = 0x1E,
    /// Read SWO trace data.
    DAP_SWO_Data        = 0x1C,

    // JTAG Commands
    /// Generate JTAG sequence TMS, TDI and capture TDO.
    DAP_JTAG_Sequence   = 0x14,
    /// Configure JTAG Chain.
    DAP_JTAG_Configure  = 0x15,
    /// Read JTAG IDCODE.
    DAP_JTAG_IDCODE     = 0x16,

    // Transfer Commands
    /// Configure Transfers.
    DAP_TransferConfigure = 0x04,
    /// Read/write single and multiple registers.
    DAP_Transfer        = 0x05,
    /// Read/Write a block of data from/to a single register.
    DAP_TransferBlock   = 0x06,
    /// Abort current Transfer.
    DAP_TransferAbort   = 0x07,

    // Atomic Commands
    /// Execute multiple DAP commands from a single packet.
    DAP_ExecuteCommands = 0x7F,
    /// Queue multiple DAP commands provided in a multiple packets.
    DAP_QueueCommands   = 0x7E,

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
                offset
            } else {
                return None;
            }
        },
        DAP_SWO_Transport => 2,
        DAP_SWO_Mode => 2,
        DAP_SWO_Baudrate => 5,
        DAP_SWO_Control => 2,
        DAP_SWO_Status => 1,
        DAP_SWO_ExtendedStatus => 2,
        DAP_SWO_Data => 3,

        DAP_JTAG_Sequence => {
            if request.len() >= 2 {
                let sequence_count = request[1];
                let mut offset = 2;
                for _ in 0..sequence_count {
                    if offset >= request.len() {
                        return None;
                    }
                    let sequence_info = request[offset];
                    offset += 1;

                    let cycle_count = match sequence_info & 0x2f {
                        0 => 64,
                        x => x as usize,
                    };
                    offset += (cycle_count + 7) / 8;
                }
                offset
            } else {
                return None;
            }
        }
        DAP_JTAG_Configure => {
            if request.len() >= 2 {
                let count = request[1] as usize;
                2 + count
            } else {
                return None;
            }
        }
        DAP_JTAG_IDCODE => 2,

        DAP_TransferConfigure => 6,
        DAP_Transfer => {
            if request.len() >= 3 {
                let transfer_count = request[2] as usize;
                let mut offset = 3;
                for _ in 0..transfer_count {
                    if offset >= request.len() {
                        return None;
                    }
                    let req = request[offset];
                    offset += 1;

                    if (req & 0b10 == 0) || (req & 0b00110000 != 0) {
                        offset += 4;
                    }
                }
                offset
            } else {
                return None;
            }
        },
        DAP_TransferBlock => {
            if request.len() >= 5 {
                let transfer_count = u16::from_le_bytes(request[2..4].try_into().unwrap());
                let req = request[5];

                if req & 0b10 == 0 {
                    5 + 4 * (transfer_count as usize)
                } else {
                    5
                }
            } else {
                return None;
            }
        },
        DAP_TransferAbort => 1,

        DAP_ExecuteCommands | DAP_QueueCommands => {
            if request.len() >= 2 {
                let num_cmd = request[1] as usize;
                let mut offset = 2;
                for _ in 0..num_cmd {
                    if offset >= request.len() {
                        return None;
                    }
                    let command = Command::try_from(request[offset])?;
                    let len = request_length(command, &request[offset..])?;
                    offset += len.get();
                }
                offset
            } else {
                return None;
            }
        }

        _ => return None,
    };

    if len > request.len() {
        return None;
    }
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

    pub fn read_byte(&mut self) -> u8 {
        let value = self.payload[0];
        self.payload = &self.payload[1..];
        value
    }

    pub fn read_short(&mut self) -> u16 {
        let value = u16::from_le_bytes(self.payload[..2].try_into().unwrap());
        self.payload = &self.payload[2..];
        value
    }

    pub fn read_word(&mut self) -> u32 {
        let value = u32::from_le_bytes(self.payload[..4].try_into().unwrap());
        self.payload = &self.payload[4..];
        value
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

    #[inline(always)]
    fn buf(&mut self) -> &mut [u8] {
        &mut self.buffer[1 + self.payload_len..]
    }

    pub fn write_byte(&mut self, value: u8) {
        self.buf()[0] = value;
        self.payload_len += 1;
    }

    pub fn write_short(&mut self, value: u16) {
        self.buf()[..2].copy_from_slice(&value.to_le_bytes());
        self.payload_len += 2;
    }

    pub fn write_word(&mut self, value: u32) {
        self.buf()[..4].copy_from_slice(&value.to_le_bytes());
        self.payload_len += 4;
    }
}

impl Deref for DapResponse<'_> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        &self.buffer[..self.payload_len+1]
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
