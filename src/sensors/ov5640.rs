//! Constants and configuration for the OV5640

pub const SENSOR_H: usize = 2608;
pub const SENSOR_V: usize = 1952;
pub const H_OFFSET: usize = 16;
pub const V_OFFSET: usize = 4;

pub const REG_DLY: u16 = 0xFFFF;
// 7-bit I2C address (0x78 >> 1)
pub const CAM_ADDR: u8 = 0x3C;

pub const INIT_START: [(u16, u8); 27] = [
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
    (FORMAT_CTRL, 0x01),     // ISP format: RGB
    (FORMAT_CTRL00, 0x61),   // RGB565, BGR... sequence
    (TIMING_TC_REG21, 0x01), // Horizontal binning
    // === DVP CONTROL ===
    (0x4740, 0x29), // PCLK active high (bit 5), Gate PCLK under VSYNC, HREF/VSYNC active high
    (0x471D, 0x00), // VSYNC Pulse at frame start
];

pub const INIT_END: [(u16, u8); 2] = [
    (SYSTEM_CTRL0, 0x02), // Power up
    (REG_DLY, 100),
];
pub enum InitLen {}
/// Returns a "default" set of init regs for a centered window based on your resolution/scale.
#[allow(non_local_definitions)]
pub const fn init_regs(
    h_res: usize,
    v_res: usize,
    image_scale: usize,
) -> [(u16, u8); InitLen::LEN] {
    let resolution = resolution_regs(h_res, v_res, image_scale);

    arrcat::concat_arrays!(
        length_type = InitLen;
        INIT_START,
        resolution: [_; 16],
        INIT_END
    )
}

// TODO: HTS and VTS
pub const fn resolution_regs(h_res: usize, v_res: usize, image_scale: usize) -> [(u16, u8); 16] {
    let x_start: u16 = (SENSOR_H - (h_res * image_scale + 2 * H_OFFSET)) as u16 / 2;
    let x_end: u16 = SENSOR_H as u16 - x_start;
    let y_start: u16 = (SENSOR_V - (v_res * image_scale + 2 * V_OFFSET)) as u16 / 2;
    let y_end: u16 = SENSOR_V as u16 - y_start;

    assert!(
        (h_res * image_scale) as u16 == x_end - x_start - 2 * H_OFFSET as u16,
        "Configured window is not the correct width"
    );
    assert!(
        (v_res * image_scale) as u16 == y_end - y_start - 2 * V_OFFSET as u16,
        "Configured window is not the correct height"
    );
    assert!(
        (h_res * image_scale) <= SENSOR_H - 2 * H_OFFSET,
        "Source frame too wide, reduce scale or output resolution"
    );
    assert!(
        (v_res * image_scale) <= SENSOR_V - 2 * V_OFFSET,
        "Source frame too tall, reduce scale or output resolution"
    );

    [
        // === TIMING CONTROL (160x120 from centered 1280x960) ===
        (0x3800, (x_start >> 8) as u8),   // X start high
        (0x3801, (x_start & 0xFF) as u8), // X start low
        (0x3802, (y_start >> 8) as u8),   // Y start high
        (0x3803, (y_start & 0xFF) as u8), // Y start low
        (0x3804, (x_end >> 8) as u8),     // X end high
        (0x3805, (x_end & 0xFF) as u8),   // X end low
        (0x3806, (y_end >> 8) as u8),     // Y end high
        (0x3807, (y_end & 0xFF) as u8),   // Y end low
        (0x3808, (h_res >> 8) as u8),     // H output high (160)
        (0x3809, (h_res & 0xFF) as u8),   // H output low
        (0x380A, (v_res >> 8) as u8),     // V output high (120)
        (0x380B, (v_res & 0xFF) as u8),   // V output low
        // === ISP OFFSETS ===
        (0x3810, (H_OFFSET >> 8) as u8),   // H offset high
        (0x3811, (H_OFFSET & 0xFF) as u8), // H offset low
        (0x3812, (V_OFFSET >> 8) as u8),   // V offset high
        (0x3813, (V_OFFSET & 0xFF) as u8), // V offset low
    ]
}
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
