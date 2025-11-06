//! Constants and configuration for the OV5640
const SENSOR_H: usize = 2608;
const SENSOR_V: usize = 1952;
const H_OFFSET: usize = 16;
const V_OFFSET: usize = 4;

const X_START: u16 = (SENSOR_H - (super::H_RES * super::IMAGE_SCALE + 2 * H_OFFSET)) as u16 / 2;
const X_END: u16 = SENSOR_H as u16 - X_START;
const Y_START: u16 = (SENSOR_V - (super::V_RES * super::IMAGE_SCALE + 2 * V_OFFSET)) as u16 / 2;
const Y_END: u16 = SENSOR_V as u16 - Y_START;
const _: () = assert!(
    (super::H_RES * super::IMAGE_SCALE) as u16 == X_END - X_START - 2 * H_OFFSET as u16,
    "Configured window is not the correct width"
);
const _: () = assert!(
    (super::V_RES * super::IMAGE_SCALE) as u16 == Y_END - Y_START - 2 * V_OFFSET as u16,
    "Configured window is not the correct width"
);

pub const REG_DLY: u16 = 0xFFFF;
// 7-bit I2C address (0x78 >> 1)
pub const CAM_ADDR: u8 = 0x3C;
pub const INIT_REGS: [(u16, u8); 139] = [
    (SYSTEM_CTRL0, 0x82), // software reset
    (REG_DLY, 10),        // delay 10ms
    (SYSTEM_CTRL0, 0x42), // power down
    // === CLOCK CONFIG ===
    (0x3103, 0x11), // System clock from pad, PLL bit
    (0x3039, 0x80), // Bypass PLL
    (0x3108, 0x00), // PCLK root divider = /8 from input clock
    // === IO DIRECTION ===
    (0x3017, 0xFF), // io direction
    (0x3018, 0xFF), // io direction
    (0x302C, 0xC3), // Drive capability
    (ISP_CONTROL_01, 0x83),
    // Reset
    (0x3000, 0x20), // reset MCU
    (REG_DLY, 10),  // delay 10ms
    (0x3002, 0x1C), // Reset ISPs
    (0x3004, 0xFF), // Enable all peripheral clocks
    (0x3005, 0xF7), // Enable all clocks except MIPI
    (0x3006, 0xC3), // Enable ISP clocks
    // ISP Control
    (0x5000, 0xB3), // Enable: LENC (bit 7=0), BPC (bit 2), WPC (bit 1), CIP (bit 0)
    (ISP_CONTROL_01, 0xA3), // Enable: scaling (bit 5), color matrix (bit 1), AWB (bit 0)
    (0x5002, 0x41), // Disable AWB gain for now (can enable later with 0x41)
    (0x5003, 0x08), // Buffer enable
    // Magic debug nonsense
    (0x370C, 0x02),
    (0x3634, 0x40),
    // === FORMAT CONTROL - RGB888 OUTPUT ===
    (FORMAT_CTRL, 0x01),   // ISP format: RGB
    //(FORMAT_CTRL00, 0x23), // RGB888, RGBRGB... sequence (0x2 = RGB888, 0x3 = RGBRGB order)
    (FORMAT_CTRL00, 0x61), // RGB565, BGR... sequence
    (TIMING_TC_REG21, 0x01), // Horizontal binning
    // AEC/AGC
    (0x3A02, 0x03),
    (0x3A03, 0xD8),
    (0x3A08, 0x01),
    (0x3A09, 0x27),
    (0x3A0A, 0x00),
    (0x3A0B, 0xF6),
    (0x3A0D, 0x04),
    (0x3A0E, 0x03),
    (0x3A0F, 0x30), // ae_level
    (0x3A10, 0x28), // ae_level
    (0x3A11, 0x60), // ae_level
    (0x3A13, 0x43),
    (0x3A14, 0x03),
    (0x3A15, 0xD8),
    (0x3A18, 0x00), // gainceiling
    (0x3A19, 0xF8), // gainceiling
    (0x3A1B, 0x30), // ae_level
    (0x3A1E, 0x26), // ae_level
    (0x3A1F, 0x14), // ae_level
    (0x3600, 0x08), // VCM debug
    (0x3601, 0x33), // VCM debug
    (0x4001, 0x02), // BLC
    (0x4004, 0x02), // BLC
    // AWB
    (0x5180, 0xFF),
    (0x5181, 0xF2),
    (0x5182, 0x00),
    (0x5183, 0x14),
    (0x5184, 0x25),
    (0x5185, 0x24),
    (0x5186, 0x09),
    (0x5187, 0x09),
    (0x5188, 0x09),
    (0x5189, 0x75),
    (0x518A, 0x54),
    (0x518B, 0xE0),
    (0x518C, 0xB2),
    (0x518D, 0x42),
    (0x518E, 0x3D),
    (0x518F, 0x56),
    (0x5190, 0x46),
    (0x5191, 0xF8),
    (0x5192, 0x04),
    (0x5193, 0x70),
    (0x5194, 0xF0),
    (0x5195, 0xF0),
    (0x5196, 0x03),
    (0x5197, 0x01),
    (0x5198, 0x04),
    (0x5199, 0x12),
    (0x519A, 0x04),
    (0x519B, 0x00),
    (0x519C, 0x06),
    (0x519D, 0x82),
    (0x519E, 0x38),
    //CIP control (Sharpness)
    (0x5300, 0x10), //sharpness
    (0x5301, 0x10), //sharpness
    (0x5302, 0x18), //sharpness
    (0x5303, 0x19), //sharpness
    (0x5304, 0x10),
    (0x5305, 0x10),
    (0x5306, 0x08), //denoise
    (0x5307, 0x16),
    (0x5308, 0x40),
    (0x5309, 0x10), //sharpness
    (0x530A, 0x10), //sharpness
    (0x530B, 0x04), //sharpness
    (0x530C, 0x06), //sharpness
    // color matrix (Saturation)
    (0x5381, 0x1E),
    (0x5382, 0x5B),
    (0x5383, 0x08),
    (0x5384, 0x0A),
    (0x5385, 0x7E),
    (0x5386, 0x88),
    (0x5387, 0x7C),
    (0x5388, 0x6C),
    (0x5389, 0x10),
    (0x538A, 0x01),
    (0x538B, 0x98),
    // GAMMA
    (0x5480, 0x01),
    (0x5481, 0x00),
    (0x5482, 0x1E),
    (0x5483, 0x3B),
    (0x5484, 0x58),
    (0x5485, 0x66),
    (0x5486, 0x71),
    (0x5487, 0x7D),
    (0x5488, 0x83),
    (0x5489, 0x8F),
    (0x548A, 0x98),
    (0x548B, 0xA6),
    (0x548C, 0xB8),
    (0x548D, 0xCA),
    (0x548E, 0xD7),
    (0x548F, 0xE3),
    (0x5490, 0x1D),
    //(0x3503, 0x00), // Disable Auto Gain and Exposure
    // === TIMING CONTROL (160x120 from centered 1280x960) ===
    (0x3800, (X_START >> 8) as u8),        // X start high
    (0x3801, (X_START & 0xFF) as u8),      // X start low
    (0x3802, (Y_START >> 8) as u8),        // Y start high
    (0x3803, (Y_START & 0xFF) as u8),      // Y start low
    (0x3804, (X_END >> 8) as u8),          // X end high
    (0x3805, (X_END & 0xFF) as u8),        // X end low
    (0x3806, (Y_END >> 8) as u8),          // Y end high
    (0x3807, (Y_END & 0xFF) as u8),        // Y end low
    (0x3808, (super::H_RES >> 8) as u8),   // H output high (160)
    (0x3809, (super::H_RES & 0xFF) as u8), // H output low
    (0x380A, (super::V_RES >> 8) as u8),   // V output high (120)
    (0x380B, (super::V_RES & 0xFF) as u8), // V output low
    // === ISP OFFSETS ===
    (0x3810, (H_OFFSET >> 8) as u8),   // H offset high
    (0x3811, (H_OFFSET & 0xFF) as u8), // H offset low
    (0x3812, (V_OFFSET >> 8) as u8),   // V offset high
    (0x3813, (V_OFFSET & 0xFF) as u8), // V offset low
    // === DVP CONTROL ===
    (0x4740, 0x2D), // PCLK active high (bit 5), HREF/VSYNC active low
    //
    (SYSTEM_CTRL0, 0x02), // Power up
    (REG_DLY, 100),
];

// OV5640 register definitions.

/* system control registers */
pub const SYSTEM_CTRL0: u16 = 0x3008; // Bit[7]: Software reset
// Bit[6]: Software power down
// Bit[5]: Reserved
// Bit[4]: SRB clock SYNC enable
// Bit[3]: Isolation suspend select
// Bit[2:0]: Not used

pub const DRIVE_CAPABILITY: u16 = 0x302C; // Bit[7:6]:
//          00: 1x
//          01: 2x
//          10: 3x
//          11: 4x

pub const SC_PLLS_CTRL0: u16 = 0x303A; // Bit[7]: PLLS bypass
pub const SC_PLLS_CTRL1: u16 = 0x303B; // Bit[4:0]: PLLS multiplier
pub const SC_PLLS_CTRL2: u16 = 0x303C; // Bit[6:4]: PLLS charge pump control
// Bit[3:0]: PLLS system divider
pub const SC_PLLS_CTRL3: u16 = 0x303D; // Bit[5:4]: PLLS pre-divider
//          00: 1
//          01: 1.5
//          10: 2
//          11: 3
// Bit[2]: PLLS root-divider - 1
// Bit[1:0]: PLLS seld5
//          00: 1
//          01: 1
//          10: 2
//          11: 2.5

/* AEC/AGC control functions */
pub const AEC_PK_MANUAL: u16 = 0x3503; // AEC Manual Mode Control
// Bit[7:6]: Reserved
// Bit[5]: Gain delay option
//         Valid when 0x3503[4]=1’b0
//         0: Delay one frame latch
//         1: One frame latch
// Bit[4:2]: Reserved
// Bit[1]: AGC manual
//         0: Auto enable
//         1: Manual enable
// Bit[0]: AEC manual
//         0: Auto enable
//         1: Manual enable

//gain = {0x350A[1:0], 0x350B[7:0]} / 16

pub const X_ADDR_ST_H: u16 = 0x3800; //Bit[3:0]: X address start[11:8]
pub const X_ADDR_ST_L: u16 = 0x3801; //Bit[7:0]: X address start[7:0]
pub const Y_ADDR_ST_H: u16 = 0x3802; //Bit[2:0]: Y address start[10:8]
pub const Y_ADDR_ST_L: u16 = 0x3803; //Bit[7:0]: Y address start[7:0]
pub const X_ADDR_END_H: u16 = 0x3804; //Bit[3:0]: X address end[11:8]
pub const X_ADDR_END_L: u16 = 0x3805; //Bit[7:0]:
pub const Y_ADDR_END_H: u16 = 0x3806; //Bit[2:0]: Y address end[10:8]
pub const Y_ADDR_END_L: u16 = 0x3807; //Bit[7:0]:
// Size after scaling
pub const X_OUTPUT_SIZE_H: u16 = 0x3808; //Bit[3:0]: DVP output horizontal width[11:8]
pub const X_OUTPUT_SIZE_L: u16 = 0x3809; //Bit[7:0]:
pub const Y_OUTPUT_SIZE_H: u16 = 0x380A; //Bit[2:0]: DVP output vertical height[10:8]
pub const Y_OUTPUT_SIZE_L: u16 = 0x380B; //Bit[7:0]:
pub const X_TOTAL_SIZE_H: u16 = 0x380C; //Bit[3:0]: Total horizontal size[11:8]
pub const X_TOTAL_SIZE_L: u16 = 0x380D; //Bit[7:0]:
pub const Y_TOTAL_SIZE_H: u16 = 0x380E; //Bit[7:0]: Total vertical size[15:8]
pub const Y_TOTAL_SIZE_L: u16 = 0x380F; //Bit[7:0]:
pub const X_OFFSET_H: u16 = 0x3810; //Bit[3:0]: ISP horizontal offset[11:8]
pub const X_OFFSET_L: u16 = 0x3811; //Bit[7:0]:
pub const Y_OFFSET_H: u16 = 0x3812; //Bit[2:0]: ISP vertical offset[10:8]
pub const Y_OFFSET_L: u16 = 0x3813; //Bit[7:0]:
pub const X_INCREMENT: u16 = 0x3814; //Bit[7:4]: Horizontal odd subsample increment
//Bit[3:0]: Horizontal even subsample increment
pub const Y_INCREMENT: u16 = 0x3815; //Bit[7:4]: Vertical odd subsample increment
//Bit[3:0]: Vertical even subsample increment
// Size before scaling
//pub const X_INPUT_SIZE: u16 = ;(X_ADDR_END - X_ADDR_ST + 1 - (2 * X_OFFSET))
//pub const Y_INPUT_SIZE: u16 = ;(Y_ADDR_END - Y_ADDR_ST + 1 - (2 * Y_OFFSET))

/* mirror and flip registers */
pub const TIMING_TC_REG20: u16 = 0x3820; // Timing Control Register
// Bit[2:1]: Vertical flip enable
//         00: Normal
//         11: Vertical flip
// Bit[0]: Vertical binning enable
pub const TIMING_TC_REG21: u16 = 0x3821; // Timing Control Register
// Bit[5]: Compression Enable
// Bit[2:1]: Horizontal mirror enable
//         00: Normal
//         11: Horizontal mirror
// Bit[0]: Horizontal binning enable

pub const PCLK_RATIO: u16 = 0x3824; // Bit[4:0]: PCLK ratio manual

/* frame control registers */
pub const FRAME_CTRL01: u16 = 0x4201; // Control Passed Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
// Bit[7:4]: Not used
// Bit[3:0]: Frame ON number
pub const FRAME_CTRL02: u16 = 0x4202; // Control Masked Frame Number When both ON and OFF number set to 0x00,frame control is in bypass mode
// Bit[7:4]: Not used
// BIT[3:0]: Frame OFF number

/* format control registers */
pub const FORMAT_CTRL00: u16 = 0x4300;

pub const CLOCK_POL_CONTROL: u16 = 0x4740; // Bit[5]: PCLK polarity 0: active low
//          1: active high
// Bit[3]: Gate PCLK under VSYNC
// Bit[2]: Gate PCLK under HREF
// Bit[1]: HREF polarity
//          0: active low
//          1: active high
// Bit[0] VSYNC polarity
//          0: active low
//          1: active high

pub const ISP_CONTROL_01: u16 = 0x5001; // Bit[5]: Scale enable
//          0: Disable
//          1: Enable

/* output format control registers */
pub const FORMAT_CTRL: u16 = 0x501F; // Format select
// Bit[2:0]:
//  000: YUV422
//  001: RGB
//  010: Dither
//  011: RAW after DPC
//  101: RAW after CIP

/* ISP top control registers */
pub const PRE_ISP_TEST_SETTING_1: u16 = 0x503D; // Bit[7]: Test enable
//         0: Test disable
//         1: Color bar enable
// Bit[6]: Rolling
// Bit[5]: Transparent
// Bit[4]: Square black and white
// Bit[3:2]: Color bar style
//         00: Standard 8 color bar
//         01: Gradual change at vertical mode 1
//         10: Gradual change at horizontal
//         11: Gradual change at vertical mode 2
// Bit[1:0]: Test select
//         00: Color bar
//         01: Random data
//         10: Square data
//         11: Black image

//exposure = {0x3500[3:0], 0x3501[7:0], 0x3502[7:0]} / 16 × tROW

pub const SCALE_CTRL_1: u16 = 0x5601; // Bit[6:4]: HDIV RW
//          DCW scale times
//          000: DCW 1 time
//          001: DCW 2 times
//          010: DCW 4 times
//          100: DCW 8 times
//          101: DCW 16 times
//          Others: DCW 16 times
// Bit[2:0]: VDIV RW
//          DCW scale times
//          000: DCW 1 time
//          001: DCW 2 times
//          010: DCW 4 times
//          100: DCW 8 times
//          101: DCW 16 times
//          Others: DCW 16 times

pub const SCALE_CTRL_2: u16 = 0x5602; // X_SCALE High Bits
pub const SCALE_CTRL_3: u16 = 0x5603; // X_SCALE Low Bits
pub const SCALE_CTRL_4: u16 = 0x5604; // Y_SCALE High Bits
pub const SCALE_CTRL_5: u16 = 0x5605; // Y_SCALE Low Bits
pub const SCALE_CTRL_6: u16 = 0x5606; // Bit[3:0]: V Offset

pub const VFIFO_CTRL0C: u16 = 0x460C; // Bit[1]: PCLK manual enable
//          0: Auto
//          1: Manual by PCLK_RATIO

pub const VFIFO_X_SIZE_H: u16 = 0x4602;
pub const VFIFO_X_SIZE_L: u16 = 0x4603;
pub const VFIFO_Y_SIZE_H: u16 = 0x4604;
pub const VFIFO_Y_SIZE_L: u16 = 0x4605;

pub const COMPRESSION_CTRL00: u16 = 0x4400; //
pub const COMPRESSION_CTRL01: u16 = 0x4401; //
pub const COMPRESSION_CTRL02: u16 = 0x4402; //
pub const COMPRESSION_CTRL03: u16 = 0x4403; //
pub const COMPRESSION_CTRL04: u16 = 0x4404; //
pub const COMPRESSION_CTRL05: u16 = 0x4405; //
pub const COMPRESSION_CTRL06: u16 = 0x4406; //
pub const COMPRESSION_CTRL07: u16 = 0x4407; // Bit[5:0]: QS
pub const COMPRESSION_ISI_CTRL: u16 = 0x4408; //
pub const COMPRESSION_CTRL09: u16 = 0x4409; //
pub const COMPRESSION_CTRL0A: u16 = 0x440A; //
pub const COMPRESSION_CTRL0B: u16 = 0x440B; //
pub const COMPRESSION_CTRL0C: u16 = 0x440C; //
pub const COMPRESSION_CTRL0D: u16 = 0x440D; //
pub const COMPRESSION_CTRL0E: u16 = 0x440E; //
