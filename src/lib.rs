#![no_std]

use core::marker::PhantomData;

use atsamd51_pcc::{self as pcc, Pcc, ReadablePin as _, SyncPins};

use atsamd_hal::{
    clock::v2::gclk::GclkOut,
    dmac::{self, Busy, Channel, PriorityLevel},
    fugit::{self, ExtU64 as _, Rate},
    gpio::{PA15, PB15, Pin, PushPullOutput},
};
use defmt::Format;
use embedded_hal::digital::OutputPin;

mod macros;
mod safe_dma;
pub mod sensors;

use rtic_time::Monotonic;
use sensors::sensor::*;

use crate::safe_dma::SafeTransfer;
use crate::sensors::{H_RES, V_RES};

type PccMode = pcc::EightBit1data;

mod seal {
    pub(super) trait Sealed {}
}

pub enum Enabled {}
pub enum Disabled {}

trait State: seal::Sealed {}
impl State for Enabled {}
impl seal::Sealed for Enabled {}
impl State for Disabled {}
impl seal::Sealed for Disabled {}

#[allow(private_bounds)]
/// Camera Driver.
///
/// The `const fn` [fb_size] is provded for convenience, recommend using this to set the
/// generic param:
/// ```rust
/// const H_RES: usize = 160;
/// const V_RES: usize = 120;
/// const FB_SIZE = fb_size(H_RES, V_RES);
/// let cam: Camera<_, _, _, _,FB_SIZE> = Cam::new(..);
/// let cam = cam.init(&init_regs(H_RES, V_RES, 8), PriotityLevel::Lvl3);
/// ```
/// Also, It's probably a good idea to Type-Alias your camera config
/// ```
/// pub type Cam = Camera<'static, Channel::Ch0, I2C, Delay, Enabled, FB_SIZE>;
/// let cam: Cam = Camera::new(..);
/// ```
pub struct Camera<
    'framebuf,
    C: dmac::AnyChannel<Status = Busy>,
    I2C,
    D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
    S: State,
    const FB_SIZE: usize,
> {
    pcc_xfer_handle: SafeTransfer<C::Id, Pcc<PccMode>, &'framebuf mut FrameBuf<FB_SIZE>>,
    i2c: I2C,
    cam_rst: Pin<PA15, PushPullOutput>,
    cam_sync: pcc::SyncPins,
    cam_clk: GclkOut<PB15>,
    inactive_buf: Option<&'framebuf mut FrameBuf<FB_SIZE>>,
    delay: D,
    _en: PhantomData<S>,
}

pub type FrameBuf<const FB_SIZE: usize> = [u8; FB_SIZE];
pub const fn fb_size(h_res: usize, v_res: usize) -> usize {
    let fb_size = h_res * v_res * 2;
    let _ = assert!(fb_size <= 65535, "Frame Buffer too large for single DMA");
    fb_size
}

unsafe impl<'framebuf, Id, I2C, D, S, const FB_SIZE: usize> Send
    for Camera<'framebuf, Channel<Id, dmac::Busy>, I2C, D, S, FB_SIZE>
where
    Id: dmac::ChId,
    D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
    S: State,
{
}

impl<'framebuf, Id, I2C, D, const FB_SIZE: usize>
    Camera<'framebuf, Channel<Id, dmac::Busy>, I2C, D, Disabled, FB_SIZE>
where
    Id: dmac::ChId,
    D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
{
    pub fn new<R: safe_dma::Uninit>(
        mut pcc: Pcc<PccMode>,
        channel: Channel<Id, R>,
        i2c: I2C,
        mut pa15: Pin<PA15, PushPullOutput>,
        pb19: GclkOut<PB15>,
        delay: D,
        fb1: &'framebuf mut FrameBuf<FB_SIZE>,
        fb2: &'framebuf mut FrameBuf<FB_SIZE>,
    ) -> Self {
        pa15.set_low().ok();

        let cam_sync = pcc.take_sync_pins().expect("Sync pins have gone missing!");
        pcc.configure(|pcc| {
            pcc.mr().modify(|_, w| {
                // Configure Clear If Disabled on VSYNC falling edge
                unsafe { w.cid().bits(1) }
            });
        });

        // Default to lowest `PriorityLevel` pre-init, user-supplied priority comes later
        let pcc_xfer_handle = SafeTransfer::new(channel, pcc, fb1, PriorityLevel::Lvl0);

        Self {
            pcc_xfer_handle,
            i2c,
            cam_rst: pa15,
            cam_sync,
            cam_clk: pb19,
            inactive_buf: Some(fb2),
            delay,
            _en: PhantomData,
        }
    }
}

impl<'framebuf, Id, I2C, D, const FB_SIZE: usize>
    Camera<'framebuf, Channel<Id, dmac::Busy>, I2C, D, Disabled, FB_SIZE>
where
    Id: dmac::ChId,
    I2C: embedded_hal_async::i2c::I2c,
    <I2C as embedded_hal_async::i2c::ErrorType>::Error: Format,
    D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
{
    pub async fn init(
        self,
        init_regs: &[(u16, u8)],
        dma_priority: PriorityLevel,
    ) -> Result<Camera<'framebuf, Channel<Id, dmac::Busy>, I2C, D, Enabled, FB_SIZE>, I2C::Error>
    {
        let mut cam = Camera {
            pcc_xfer_handle: self.pcc_xfer_handle,
            i2c: self.i2c,
            cam_rst: self.cam_rst,
            cam_sync: self.cam_sync,
            cam_clk: self.cam_clk,
            inactive_buf: self.inactive_buf,
            delay: self.delay,
            _en: PhantomData,
        };

        // Hardware reset sequence
        cam.cam_rst.set_low().ok();
        D::delay(5.millis()).await;
        cam.cam_rst.set_high().ok();
        D::delay(20.millis()).await;

        // Init Regs
        cam.write_regs(init_regs.iter().copied()).await?;
        D::delay(100.millis()).await;

        cam.write_reg(0x3008, 0x02).await?; // Exit software power down
        let id = cam.get_sensor_id().await?;
        log_debug!("Sensor ID: {:#X}", id);
        assert_eq!(id, 0x5640);

        cam.resync_framebuf(Some(dma_priority));
        Ok(cam)
    }
}

impl<'framebuffer, Id, I2C, D, const FB_SIZE: usize>
    Camera<'framebuffer, Channel<Id, dmac::Busy>, I2C, D, Enabled, FB_SIZE>
where
    Id: dmac::ChId,
    I2C: embedded_hal_async::i2c::I2c,
    <I2C as embedded_hal_async::i2c::ErrorType>::Error: Format,
    D: Monotonic<Duration = fugit::Duration<u64, 1, 32768>>,
{
    /// Writes an iterator of registers to the cam, Optionally power cycling:
    ///
    /// Some(true) => Hard reset
    /// Some(false) => Soft reset
    /// None => No reset
    ///
    /// # Errors
    ///
    /// This function will return an error if I2C Fails
    pub async fn reconfigure(
        &mut self,
        regs: impl Iterator<Item = (u16, u8)>,
        power_cycle: Option<bool>,
    ) -> Result<(), I2C::Error> {
        Self::vsync(&self.cam_sync);
        if let Some(hard) = power_cycle {
            if hard {
                self.cam_rst.set_low().ok();
                D::delay(5.millis()).await;
                self.cam_rst.set_high().ok();
                D::delay(20.millis()).await;
            }
            self.write_reg(0x3008, 0x82).await?; // SW Power down
            D::delay(10.millis()).await;
        }

        self.write_regs(regs).await?;

        if let Some(_) = power_cycle {
            D::delay(100.millis()).await;
            self.write_reg(0x3008, 0x02).await?; // SW Power up
        }

        // Wait for VSYNC and restart the DMA to prevent frame-tear after a reconfigure
        self.resync_framebuf(None);

        Ok(())
    }

    /// Spin-Wait for VSYNC falling edge
    /// PERF: Make Async with ExtInt
    fn vsync(sync: &SyncPins) {
        log_debug!("Wait for VSYNC Falling edge");
        while sync.den1.is_low() {}
        while sync.den1.is_high() {}
    }

    /// Stop transfer, Wait for frame boundary, then restart transfer to re-align buffer
    pub fn resync_framebuf(&mut self, priority: Option<PriorityLevel>) {
        let waitfn = Some(|| {
            Self::vsync(&self.cam_sync);
        });
        self.pcc_xfer_handle.restart(priority, waitfn);
    }

    async fn write_reg(&mut self, reg: u16, val: u8) -> Result<(), I2C::Error> {
        log_debug!("Writing {:#X} to {:#X}", val, reg);
        self.i2c
            .write(CAM_ADDR, &[(reg >> 8) as u8, reg as u8, val])
            .await?;
        Ok(())
    }

    async fn write_regs(
        &mut self,
        regs: impl Iterator<Item = (u16, u8)>,
    ) -> Result<(), I2C::Error> {
        for (reg, val) in regs {
            if reg == REG_DLY {
                D::delay((val as u64).millis()).await;
            } else {
                self.write_reg(reg, val).await?;
            }
        }
        Ok(())
    }

    pub async fn read_reg(&mut self, reg: u16, buf: &mut [u8]) -> Result<(), I2C::Error> {
        log_debug!("Reading from {:#X}", reg);
        self.i2c
            .write_read(CAM_ADDR, &[(reg >> 8) as u8, reg as u8], buf)
            .await?;
        Ok(())
    }

    /// Set PLL configuration and print resulting clocks
    ///
    /// # Arguments
    /// * `xclk_mhz` - Input clock frequency (typically 24MHz)
    /// * `pll_bypass` - true to bypass PLL and use xclk directly
    /// * `pll_multiplier` - PLL multiplier (4-252, must be even if >127)
    /// * `pll_sys_div` - System clock divider (0-15, 0 means 1)
    /// * `pre_div` - Pre-divider index (0-8, maps to [1, 1, 2, 3, 4, 1.5, 6, 2.5, 8])
    /// * `root_2x` - Enable root 2x divider
    /// * `pclk_root_div` - PCLK root divider (0-3, maps to [1, 2, 4, 8])
    /// * `pclk_manual` - Enable manual PCLK divider
    /// * `pclk_div` - PCLK divider value (0-31)
    /// * `bit_mode` - Data bit width (8 or 10)
    pub async fn set_pll(
        &mut self,
        xclk_mhz: Rate<u32, 1, 1>,
        pll_bypass: bool,
        pll_multiplier: u8,
        pll_sys_div: u8,
        pre_div: u8,
        root_2x: bool,
        pclk_root_div: u8,
        pclk_manual: bool,
        pclk_div: u8,
        bit_mode: u8,
    ) -> Result<(), I2C::Error> {
        assert!(pll_multiplier >= 4 && pll_multiplier <= 252);
        assert!(pll_sys_div <= 15);
        assert!(pre_div <= 8);
        assert!(pclk_div <= 31);
        assert!(pclk_root_div <= 3);
        assert!(bit_mode == 8 || bit_mode == 10);

        // If multiplier > 127, must be even
        let multiplier = if pll_multiplier > 127 {
            pll_multiplier & 0xFE
        } else {
            pll_multiplier
        };

        // Generate register values
        let regs: [(u16, u8); 9] = [
            (0x3039, if pll_bypass { 0x80 } else { 0x00 }),
            (0x3034, if bit_mode == 10 { 0x1A } else { 0x18 }),
            (0x3035, 0x01 | ((pll_sys_div & 0x0F) << 4)),
            (0x3036, multiplier),
            (
                0x3037,
                (pre_div & 0x0F) | (if root_2x { 0x10 } else { 0x00 }),
            ),
            (0x3108, ((pclk_root_div & 0x03) << 4) | 0x06),
            (0x3824, pclk_div & 0x1F),
            (0x460C, if pclk_manual { 0x22 } else { 0x20 }),
            // Enable PLL
            (0x3103, 0x11),
        ];

        #[cfg(debug_assertions)]
        calc_and_print_clocks(
            xclk_mhz,
            pll_bypass,
            pll_multiplier,
            pll_sys_div,
            pre_div,
            root_2x,
            pclk_root_div,
            pclk_manual,
            pclk_div,
            bit_mode,
        );

        self.write_regs(regs.iter().copied()).await?;
        Ok(())
    }

    /// Read a complete frame from the camera
    pub fn read_frame(&mut self) -> Option<&mut FrameBuf<FB_SIZE>> {
        Self::vsync(&self.cam_sync);
        let full_buf = self
            .pcc_xfer_handle
            .swap(self.inactive_buf.take().expect("Framebuffer missing"));
        self.inactive_buf.replace(full_buf);
        self.inactive_buf.as_deref_mut()
    }

    /// Configure window position, locked to the contant resolution
    pub async fn set_window(&mut self, x_start: u16, y_start: u16) -> Result<(), I2C::Error> {
        let x_end = x_start + H_RES as u16 - 1;
        let y_end = y_start + V_RES as u16 - 1;

        if x_end > SENSOR_H as u16 || y_end > SENSOR_V as u16 {
            defmt::warn!("Window overlaaps edge of sensor, aborting");
            return Ok(());
        }

        // Set window coordinates
        self.write_reg(0x3800, (x_start >> 8) as u8).await?; // X start high
        self.write_reg(0x3801, x_start as u8).await?; // X start low
        self.write_reg(0x3802, (y_start >> 8) as u8).await?; // Y start high
        self.write_reg(0x3803, y_start as u8).await?; // Y start low
        self.write_reg(0x3804, (x_end >> 8) as u8).await?; // X end high
        self.write_reg(0x3805, x_end as u8).await?; // X end low
        self.write_reg(0x3806, (y_end >> 8) as u8).await?; // Y end high
        self.write_reg(0x3807, y_end as u8).await?; // Y end low

        // Set output size to match window
        self.write_reg(0x3808, (H_RES >> 8) as u8).await?; // Output width high
        self.write_reg(0x3809, H_RES as u8).await?; // Output width low
        self.write_reg(0x380A, (V_RES >> 8) as u8).await?; // Output height high
        self.write_reg(0x380B, V_RES as u8).await?; // Output height low

        Ok(())
    }

    /// Set manual exposure
    pub async fn set_exposure(&mut self, exposure: u32) -> Result<(), I2C::Error> {
        self.write_reg(0x3500, (exposure >> 12) as u8).await?; // Exposure[19:16]
        self.write_reg(0x3501, (exposure >> 4) as u8).await?; // Exposure[15:8]
        self.write_reg(0x3502, (exposure << 4) as u8).await?; // Exposure[7:0]
        Ok(())
    }

    /// Set manual gain
    pub async fn set_gain(&mut self, gain: u16) -> Result<(), I2C::Error> {
        self.write_reg(0x350A, (gain >> 8) as u8).await?; // Gain[9:8]
        self.write_reg(0x350B, gain as u8).await?; // Gain[7:0]
        Ok(())
    }

    /// Enable/disable test pattern for debugging
    pub async fn test_pattern(&mut self, on: bool) -> Result<(), I2C::Error> {
        if on {
            self.write_reg(0x503D, 0x80).await?;
            self.write_reg(0x4741, 0x00).await?;
        } else {
            self.write_reg(0x503D, 0x00).await?; // Test pattern disable
        }
        Ok(())
    }

    /// Verify sensor is responding with correct chip ID
    pub async fn get_sensor_id(&mut self) -> Result<u16, I2C::Error> {
        let mut id = [0u8; 2];
        self.read_reg(0x300A, &mut id[0..1]).await?; // Chip ID high byte
        self.read_reg(0x300B, &mut id[1..2]).await?; // Chip ID low byte
        Ok(u16::from_be_bytes(id))
    }
}

/// Calculate OV5640 clock tree based on register settings.
/// Matches the esp32-camera calc_sysclk function.
#[cfg(debug_assertions)]
pub fn calc_and_print_clocks(
    xclk: Rate<u32, 1, 1>,
    pll_bypass: bool,
    pll_multiplier: u8,
    pll_sys_div: u8,
    pre_div: u8,
    root_2x: bool,
    pclk_root_div: u8,
    pclk_manual: bool,
    pclk_div: u8,
    bit_mode: u8,
) {
    const PLL_PRE_DIV2X_MAP: [f32; 9] = [1.0, 1.0, 2.0, 3.0, 4.0, 1.5, 6.0, 2.5, 8.0];
    const PLL_PCLK_ROOT_DIV_MAP: [u32; 4] = [1, 2, 4, 8];

    let pll_sys_div = if pll_sys_div == 0 { 1 } else { pll_sys_div };

    let pll_pre_div_val = PLL_PRE_DIV2X_MAP[pre_div as usize];
    let root_2x_div = if root_2x { 2 } else { 1 };
    let pll_pclk_root_div_val = PLL_PCLK_ROOT_DIV_MAP[pclk_root_div as usize];

    // Clock calculations
    let refin = xclk * 2 / ((pll_pre_div_val * 2.0) as u32);
    let vco = refin * pll_multiplier as u32 / root_2x_div;

    // bit_mode affects the final divisor: 10-bit mode = /5, 8-bit mode = /4
    let bit_divisor = if bit_mode == 10 { 5 } else { 4 };

    let pll_clk = if pll_bypass {
        xclk
    } else {
        vco / pll_sys_div as u32 * 2 / bit_divisor
    };

    let pclk_divisor = if pclk_manual && pclk_div != 0 {
        pclk_div as u32
    } else {
        2
    };
    let pclk = pll_clk / pll_pclk_root_div_val / pclk_divisor;
    let sysclk = pll_clk / 4;

    defmt::println!("OV5640 Clock Configuration:");
    defmt::println!("  XVCLK:   {}  (input)", xclk);
    defmt::println!("  REFIN:   {}  (XVCLK / pre_div)", refin);
    defmt::println!("  VCO:     {}  (REFIN * multiplier / root_2x_div)", vco);
    defmt::println!(
        "  PLL_CLK: {}  (VCO / sys_div * 2 / {})",
        pll_clk,
        bit_divisor
    );
    defmt::println!("  SYSCLK:  {}  (PLL_CLK / 4)", sysclk);
    defmt::println!("  PCLK:    {}  (PLL_CLK / pclk_root_div / pclk_div)", pclk);
    defmt::println!("  SYSCLK/PCLK ratio: {}:1", sysclk / pclk);
}
