use bxcan::{Frame, Data, StandardId, ExtendedId};

use stm32f1xx_hal::time::{Bps, U32Ext};


pub enum SlcanCmdSetupType {
    StandardBitrate(Bps),
    BTR{
        btr0: u8,
        btr1: u8
    },  
}

pub enum SlcanCmd {
    Setup(SlcanCmdSetupType),
    Open,
    Close,
    CanFrame(Frame),
    ReadStatus,
    SetAcceptanceCode(u32),
    SetAcceptanceMask(u32),
    GetVersion,
    GetSerialNumber,
    SetTimestamp(bool)
}

pub mod slcan_responses {
    use std::io::Read;

    use bxcan::Frame;

    const ASCII_CR: u8 = 13;
    const ASCII_BELL: u8 = 7;

    pub const SLCAN_SUCCESS: &[u8] = &[ASCII_CR];
    pub const SLCAN_TX_SUCCESS: &[u8] = &[b'z', ASCII_CR];
    pub const SLCAN_FAILURE: &[u8] = &[ASCII_BELL];
    pub const SLCAN_VERSION: &[u8] = &[b'V', b'0', b'0', b'0', b'0', ASCII_CR];
    pub const SLCAN_SERIAL_NO: &[u8] = &[b'N', b'B', b'R', b'2', b'3', ASCII_CR];
    
    fn nibble_to_hexchar (b: u8) -> u8 {
        match b {
            0..=9 => b'0' + b,
            10..=15 => b'A' + b - 10,
            _ => b'X',
        }
    }
    
    fn u8_to_hex(b: u8) -> [u8; 2] {
        let upper = nibble_to_hexchar((b & 0xf0) >> 4);
        let lower = nibble_to_hexchar(b & 0x0f);
        [upper, lower]
    }
    
    fn std_id_to_hex(id: u16) -> [u8; 3] {
        let upper: u8 = nibble_to_hexchar(((id & 0xf00) >> 8) as u8);
        let mid = nibble_to_hexchar(((id & 0x0f0) >> 4) as u8);
        let lower = nibble_to_hexchar((id & 0x00f) as u8);
        [upper, mid, lower]
    }

    fn ext_id_to_hex(id: u32) -> [u8; 8] {
        let mut out_buf = [0u8; 8];

        for i in 0..8 {
            out_buf[7 - i] = nibble_to_hexchar((id & (0xf << (4 * i)) >> (4 * i)) as u8);
        }

        out_buf
    }

    pub fn slcan_status_flags(rx_fifo_full: bool, tx_fifo_full: bool, data_overrun: bool) -> [u8; 4] {
        let mut status_byte: u8 = 0;
        if rx_fifo_full {
            status_byte |= (0b1 << 0);
        }
        if tx_fifo_full {
            status_byte |= (0b1 << 1);
        }
        if data_overrun {
            status_byte |= (0b1 << 3);
        }
        let status_byte = u8_to_hex(status_byte);
    
        [b'F', status_byte[0], status_byte[1], ASCII_CR]
    }

    pub fn slcan_frame(frame: Frame) -> ([u8; 20], usize) {
        let mut out_buf = [0u8; 20];

        match frame.id() {
            bxcan::Id::Standard(id) => {
                out_buf[0] = b't';
                out_buf[1..=3].swap_with_slice(&mut std_id_to_hex(id.as_raw()));
                out_buf[4] = nibble_to_hexchar(frame.dlc());
                if let Some(&data) = frame.data() {
                    let d = data;
                }
                for i in 0..frame.dlc() {
                    
                }
                (out_buf, 8)
            },
            bxcan::Id::Extended(id) => {
                
            },
        }
    }
}

fn parse_hex_char(c: u8) -> Option<u8> {
    match c {
        b'0'..=b'9' => Some(c - b'0'),
        b'a'..=b'f' => Some(c - b'a' + 10),
        b'A'..=b'F' => Some(c - b'A' + 10),
        _ => None
    }
}

fn parse_hex_byte(chars: &[u8]) -> Option<u8> {
    if chars.len() < 2 { return None }
    let lower_nibble = parse_hex_char(chars[1])?;
    let upper_nibble = parse_hex_char(chars[0])?;
    Some(lower_nibble + 16 * upper_nibble)
}

fn parse_hex_int(chars: &[u8]) -> Option<u32> {
    let mut result = 0u32;
    for c in chars {
        result <<= 4;
        result += parse_hex_char(*c)? as u32;
    }
    Some(result)
}

fn parse_btr(chars: &[u8]) -> Option<SlcanCmd> {
    let btr0 = parse_hex_byte(&chars[1..=2])?;
    let btr1 = parse_hex_byte(&chars[3..=4
        ])?;
    Some(SlcanCmd::Setup(SlcanCmdSetupType::BTR{btr0, btr1}))
}

fn parse_standard_frame(data: &[u8]) -> Option<Frame> {
    // Make sure we at least have enough for a 0-byte frame
    if data.len() <= 5 {return None}
    
    // Parse CAN ID and dtl
    let id = parse_hex_int(&data[1..=3])? as u16;
    let dtl = parse_hex_char(data[4])? as usize;

    // Ensure there's enough hex-encoded data for the specified length
    if dtl > 8 || data.len() <= 5 + (2 * dtl) {return None}

    // Create an array to store the parsed data
    let mut frame_data = [0u8; 8];

    // Parse byte-by-byte
    for i in 0..dtl {
        let hex_byte_start = 5 + (2 * i);
        let hex_byte = &data[hex_byte_start..=hex_byte_start + 1];
        frame_data[i] = parse_hex_byte(hex_byte)?;
    }

    // Create the frame
    let frame_id = StandardId::new(id)?;
    let frame_data = Data::new(&frame_data[0..dtl])?;
    Some(Frame::new_data(frame_id, frame_data))
}

fn parse_standard_empty_frame(data: &[u8]) -> Option<Frame> {
    // Make sure we at least have enough for a 0-byte frame
    if data.len() <= 5 {return None}

    // Parse CAN ID and dtl
    let id = parse_hex_int(&data[1..=3])? as u16;
    let dtl = parse_hex_char(data[4])?;

    if dtl > 8 { return None }

    // Create the frame
    let frame_id = StandardId::new(id)?;
    Some(Frame::new_remote(frame_id, dtl))
}


fn parse_extended_frame(data: &[u8]) -> Option<Frame> {
    // Make sure we at least have enough for a 0-byte frame
    if data.len() <= 10 {return None}

    // Parse CAN ID and dtl
    let id = parse_hex_int(&data[1..=8])?;
    let dtl = parse_hex_char(data[4])? as usize;

    if dtl > 8 || data.len() <= 10 + (2 * dtl) { return None }

    // Create an array to store the parsed data
    let mut frame_data = [0u8; 8];

    // Parse byte-by-byte
    for i in 0..dtl {
        let hex_byte_start = 10 + (2 * i);
        let hex_byte = &data[hex_byte_start..=hex_byte_start + 1];
        frame_data[i] = parse_hex_byte(hex_byte)?;
    }

    // Create the frame
    let frame_id = ExtendedId::new(id)?;
    let frame_data = Data::new(&frame_data[0..dtl])?;
    Some(Frame::new_data(frame_id, frame_data))
}

fn parse_extended_empty_frame(data: &[u8]) -> Option<Frame> {
    // Make sure we at least have enough for a 0-byte frame
    if data.len() <= 10 {return None}

    // Parse CAN ID and dtl
    let id = parse_hex_int(&data[1..=8])?;
    let dtl = parse_hex_char(data[4])?;

    if dtl > 8 { return None }

    // Create the frame
    let frame_id = ExtendedId::new(id)?;
    Some(Frame::new_remote(frame_id, dtl))
}

pub fn parse_slcan_cmd(data: &[u8]) -> Option<SlcanCmd> {
    if data.len() < 2 { // Must have at least two characters (Command + CR)
        return None
    }
    match data[0] {
        b'S' => {
            match data[1] {
                b'0'..=b'8' => Some(SlcanCmd::Setup(SlcanCmdSetupType::StandardBitrate(
                    match data[1] {
                        b'0' => 10_000.bps(),
                        b'1' => 20_000.bps(),
                        b'2' => 50_000.bps(),
                        b'3' => 100_000.bps(),
                        b'4' => 125_000.bps(),
                        b'5' => 250_000.bps(),
                        b'6' => 500_000.bps(),
                        b'7' => 800_000.bps(),
                        b'8' => 1_000_000.bps(),
                        _ => panic!() // shouldn't happen
                    }
                ))),
                _ => None
            }
        },
        b's' => parse_btr(data),
        b'O' => Some(SlcanCmd::Open),
        b'C' => Some(SlcanCmd::Close),

        // CAN frame with data
        b't' => parse_standard_frame(data).map(SlcanCmd::CanFrame),
        b'T' => parse_extended_frame(data).map(SlcanCmd::CanFrame),

        // Empty CAN frame
        b'r' => parse_standard_empty_frame(data).map(SlcanCmd::CanFrame),
        b'R' => parse_extended_empty_frame(data).map(SlcanCmd::CanFrame),

        b'M' => Some(SlcanCmd::SetAcceptanceCode(0)), // Acceptance mask/code not implemented
        b'm' => Some(SlcanCmd::SetAcceptanceMask(0)),

        b'F' => Some(SlcanCmd::ReadStatus),

        b'V' => Some(SlcanCmd::GetVersion),
        b'N' => Some(SlcanCmd::GetSerialNumber),

        b'Z' => {
            match data[1] {
                b'0' => Some(SlcanCmd::SetTimestamp(false)),
                b'1' => Some(SlcanCmd::SetTimestamp(true)),
                _ => None
            }
        },
        _ => None
    }

}
