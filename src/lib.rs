#![no_std]

use atsamd51_pcc::{self as pcc, Pcc};

#[cfg(feature="adafruit-branch")]
use atsamd_hal_git as atsamd_hal;

use atsamd_hal::{clock::v2::gclk::GclkOut, dmac::{self, BufferPair, Channel, Transfer, TriggerSource}, fugit::{self, ExtU32 as _}, gpio::{Pin, PushPullOutput, PA15, PB15}};
use defmt::{self as log, Format};
use embedded_hal::{delay::DelayNs, digital::OutputPin};

pub mod sensors;

use sensors::sensor::*;

use crate::sensors::{H_RES, V_RES};

type PccMode = pcc::EightBit1data;


pub struct Camera<C: dmac::AnyChannel, I2C, D: embedded_hal::delay::DelayNs> {
	pcc_xfer_handle: Option<Transfer<C, BufferPair<Pcc<PccMode>, &'static mut FrameBuf>>>,
	i2c: I2C,
	cam_rst: Pin<PA15, PushPullOutput>,
	cam_sync: pcc::SyncPins,
	cam_clk: GclkOut<PB15>,
	pub other_buffer: Option<*mut FrameBuf>,
    delay: D
}

#[derive(Debug, Default, Clone, Copy, defmt::Format)]
#[repr(C)]
pub struct Pixel {
	pub r: u8,
	pub g: u8,
	pub b: u8,
}

impl Pixel {
	pub fn as_RGB888_bytes(&self) -> [&u8; 3] {
		[&self.r, &self.g, &self.b]
	}

	fn rgb888_to_rgb565(pixel: &Pixel) -> u16 {
		let r5: u16 = (pixel.r as u16 & 0xF8) << 8;
		let g6: u16 = (pixel.g as u16 & 0xFC) << 3;
		let b5: u16 = (pixel.b as u16) >> 3;

		// Pack into RGB565: RRRRR GGGGGG BBBBB
		let _565: u16 = r5 | g6 | b5;
		//log::info!("{:?}, {:#X}", pixel, _565);
		_565
	}
}


pub type Frame = [[Pixel; H_RES]; V_RES];
// 565
pub type FrameBuf = [u8; H_RES * V_RES * 2];
// 888
//pub type FrameBuf = [u8; H_RES * V_RES * 3];

pub fn flatten_frame_bytes_iter(frame: &Frame) -> impl Iterator<Item = &u8> {
	frame.into_iter().flatten().flat_map(Pixel::as_RGB888_bytes)
}

pub fn flatten_frame_bytes_magic(frame: &Frame) -> &[u8; 3 * H_RES * V_RES] {
	// A Frame is just [[[u8; 3]; H_RES]; V_RES] in memory, and arrays are flat in in memory
	unsafe { core::mem::transmute(frame) }
}

// 565
static mut FRAMEBUFFER: FrameBuf = [0; H_RES * V_RES * 2];
static mut FRAMEBUFFER_2: FrameBuf = [0; H_RES * V_RES * 2];
// 888
//static mut FRAMEBUFFER: FrameBuf = [0; H_RES * V_RES * 3];
//static mut FRAMEBUFFER_2: FrameBuf = [0; H_RES * V_RES * 3];

impl<Id, I2C, D> Camera<Channel<Id, dmac::Busy>, I2C, D>
where 
    Id: dmac::ChId,
    D: embedded_hal::delay::DelayNs
{
	fn sleep(&mut self, duration: fugit::MillisDurationU32) {
		self.delay.delay_ms(duration.to_millis());
	}
}

impl<Id, I2C, D> Camera<Channel<Id, dmac::Busy>, I2C, D>
where 
    Id: dmac::ChId,
    I2C: embedded_hal::i2c::I2c, 
    <I2C as embedded_hal::i2c::ErrorType>::Error: Format,
    D: embedded_hal::delay::DelayNs
{	
    pub fn new(
		mut pcc: Pcc<PccMode>,
		channel: Channel<Id, dmac::Ready>,
		i2c: I2C,
		mut pa15: Pin<PA15, PushPullOutput>,
		pb19: GclkOut<PB15>,
        delay: D
	) -> Self {
		pa15.set_low().ok();

		#[allow(static_mut_refs)]
		let framebuffer = unsafe { &mut FRAMEBUFFER };
		let cam_sync = pcc.take_sync_pins().expect("Sync pins have gone missing!");
		pcc.configure(|pcc| {
			pcc.mr().write(|w|
                // Configure Clear If Disabled of VSYNC falling edge
                unsafe { w.cid().bits(1) });
		});
		let pcc_xfer_handle = Some(
			Transfer::new(channel, pcc, framebuffer, false)
				.expect("DMA Transfer INIT FAIL")
				.begin(TriggerSource::PccRx, dmac::TriggerAction::Burst),
		);

		let other_buffer = unsafe { Some(&raw mut FRAMEBUFFER_2) };
		let mut camera = Self {
			pcc_xfer_handle,
			i2c,
			cam_rst: pa15,
			cam_sync,
			cam_clk: pb19,
			other_buffer,
            delay
		};
		match camera.init() {
			Ok(_) => log::info!("Camera Init Success"),
			Err(e) => log::error!("Camera Init fail: {:?}", e),
		}
		camera
	}



	fn init(&mut self) -> Result<(), I2C::Error> {
		// Hardware reset sequence
		self.cam_rst.set_low().ok();
		self.sleep(5.millis());
		self.cam_rst.set_high().ok();
		self.sleep(20.millis());

		// Init Regs
		self.write_regs(&INIT_REGS)?;
		self.sleep(100.millis());

		self.test_pattern(false)?;

		// self.write_reg(0x4202, 0xFF)?;
		self.write_reg(0x3008, 0x02)?; // Exit software power down
		let id = self.get_sensor_id()?;
		log::debug!("Sensor ID: {:#X}", id);
		assert_eq!(id, 0x5640);

		let mut buf = [0u8; 1];
		self.read_reg(0x3039, &mut buf)?;
		log::info!("PLL bypass: {:#x}", buf[0]);

		Ok(())
	}

	fn write_reg(&mut self, reg: u16, val: u8) -> Result<(), I2C::Error> {
		log::info!("Writing {:#X} to {:#X}", val, reg);
		self.i2c
			.write(CAM_ADDR, &[(reg >> 8) as u8, reg as u8, val])?;
		Ok(())
	}

	fn write_regs(&mut self, regs: &[(u16, u8)]) -> Result<(), I2C::Error> {
		for (reg, val) in regs {
			if *reg == REG_DLY {
				self.sleep((*val as u32).millis());
			} else {
				self.write_reg(*reg, *val)?;
			}
		}
		Ok(())
	}

	fn read_reg(&mut self, reg: u16, buf: &mut [u8]) -> Result<(), I2C::Error> {
		log::info!("Reading from {:#X}", reg);
		self.i2c
			.write_read(CAM_ADDR, &[(reg >> 8) as u8, reg as u8], buf)?;
		Ok(())
	}

	/// Read a complete frame from the camera
	/// This will hold the mutable reference to self until the frame is processed
	/// The returned reference looks into a static buffer, be aware that this maye cause Strange
	/// Issues.
	pub fn read_frame(&mut self) -> Result<&'static mut FrameBuf, I2C::Error> {
		//self.write_reg(0x4202, 0x00)?;
		//self.write_reg(0x3008, 0x02)?; // Exit software power down
		let xfer = self.pcc_xfer_handle.as_mut().unwrap();
		// log::info!("Waiting for VSync");
		// while self.cam_sync.den1.is_high() {};
		let other_buf = self.other_buffer.take().map(|b| unsafe{ b.as_mut() }).flatten().unwrap();
		while !xfer.complete() {}
		let framebuffer = xfer.recycle_source(other_buf).unwrap();

        self.other_buffer.replace(framebuffer);
		//if pcc.read_flags().ovre().bit_is_set() {
		// 	log::error!("PCC Overrun!")
		//  }
		// 	pcc.configure(|_| ()); // Off and on again
		let frame: &Frame = unsafe { core::mem::transmute(&mut *framebuffer) };
		//let frame: &Frame = &[[Pixel::default(); H_RES]; V_RES];
		// let frame_u16: &[u16; H_RES * V_RES] = unsafe {core::mem::transmute(&mut *framebuffer)};
		// Stop frame capture
		//self.write_reg(0x3008, 0x42)?; // Enter software power down
		// self.write_reg(0x4202, 0xFF)?;

		// self.write_reg(0x4202, 0xFF)?; // Mask Frames

		//for read in framebuffer.iter_mut() {
		//    *read = read.to_be();
		//}

		//log::info!("{:?}", framebuffer);


		// self.write_reg(0x4202, 0x00)?; // unmask frames
		//delay().delay_ms(100);
		//	log::debug!("Frame Mangling done");
		Ok(framebuffer)
	}

	/// Configure windowing
	pub fn set_window(
		&mut self,
		x_start: u16,
		y_start: u16,
		width: u16,
		height: u16,
	) -> Result<(), I2C::Error> {
		let x_end = x_start + width - 1;
		let y_end = y_start + height - 1;

		// Set window coordinates
		self.write_reg(0x3800, (x_start >> 8) as u8)?; // X start high
		self.write_reg(0x3801, x_start as u8)?; // X start low
		self.write_reg(0x3802, (y_start >> 8) as u8)?; // Y start high
		self.write_reg(0x3803, y_start as u8)?; // Y start low
		self.write_reg(0x3804, (x_end >> 8) as u8)?; // X end high
		self.write_reg(0x3805, x_end as u8)?; // X end low
		self.write_reg(0x3806, (y_end >> 8) as u8)?; // Y end high
		self.write_reg(0x3807, y_end as u8)?; // Y end low

		// Set output size to match window
		self.write_reg(0x3808, (width >> 8) as u8)?; // Output width high
		self.write_reg(0x3809, width as u8)?; // Output width low
		self.write_reg(0x380A, (height >> 8) as u8)?; // Output height high
		self.write_reg(0x380B, height as u8)?; // Output height low

		Ok(())
	}

	/// Util for region -> window
	pub fn set_window_region(
		&mut self,
		region: (usize, usize, usize, usize),
	) -> Result<(), I2C::Error> {
		self.set_window(
			region.0 as u16,
			region.1 as u16,
			region.2 as u16,
			region.3 as u16,
		)
	}

	/// Set manual exposure
	pub fn set_exposure(&mut self, exposure: u32) -> Result<(), I2C::Error> {
		self.write_reg(0x3500, (exposure >> 12) as u8)?; // Exposure[19:16]
		self.write_reg(0x3501, (exposure >> 4) as u8)?; // Exposure[15:8]
		self.write_reg(0x3502, (exposure << 4) as u8)?; // Exposure[7:0]
		Ok(())
	}

	/// Set manual gain
	pub fn set_gain(&mut self, gain: u16) -> Result<(), I2C::Error> {
		self.write_reg(0x350A, (gain >> 8) as u8)?; // Gain[9:8]
		self.write_reg(0x350B, gain as u8)?; // Gain[7:0]
		Ok(())
	}

	/// Enable/disable test pattern for debugging
	pub fn test_pattern(&mut self, on: bool) -> Result<(), I2C::Error> {
		if on {
			self.write_reg(0x503D, 0x80)?;
			self.write_reg(0x4741, 0x00)?;
		} else {
			self.write_reg(0x503D, 0x00)?; // Test pattern disable
		}
		Ok(())
	}

	/// Verify sensor is responding with correct chip ID
	pub fn get_sensor_id(&mut self) -> Result<u16, I2C::Error> {
		let mut id = [0u8; 2];

		self.read_reg(0x300A, &mut id[0..1])?; // Chip ID high byte
		self.read_reg(0x300B, &mut id[1..2])?; // Chip ID low byte

		Ok(u16::from_be_bytes(id))
	}
}

impl<Id, I2C, D> Camera<Channel<Id, dmac::Busy>, I2C, D>
where 
    Id: dmac::ChId,
    I2C: embedded_hal_async::i2c::I2c, 
    <I2C as embedded_hal_async::i2c::ErrorType>::Error: Format,
    D: embedded_hal::delay::DelayNs
{	
    pub async fn new_async(
		mut pcc: Pcc<PccMode>,
		channel: Channel<Id, dmac::Ready>,
		i2c: I2C,
		mut pa15: Pin<PA15, PushPullOutput>,
		pb19: GclkOut<PB15>,
        delay: D
	) -> Self {
		pa15.set_low().ok();

		#[allow(static_mut_refs)]
		let framebuffer = unsafe { &mut FRAMEBUFFER };
		let cam_sync = pcc.take_sync_pins().expect("Sync pins have gone missing!");
		pcc.configure(|pcc| {
			pcc.mr().write(|w|
                // Configure Clear If Disabled of VSYNC falling edge
                unsafe { w.cid().bits(1) });
		});
		let pcc_xfer_handle = Some(
			Transfer::new(channel, pcc, framebuffer, false)
				.expect("DMA Transfer INIT FAIL")
				.begin(TriggerSource::PccRx, dmac::TriggerAction::Burst),
		);

		let other_buffer = unsafe { Some(&raw mut FRAMEBUFFER_2) };
		let mut camera = Self {
			pcc_xfer_handle,
			i2c,
			cam_rst: pa15,
			cam_sync,
			cam_clk: pb19,
			other_buffer,
            delay
		};
		match camera.init_async().await {
			Ok(_) => log::info!("Camera Init Success"),
			Err(e) => log::error!("Camera Init fail: {:?}", e),
		}
		camera
	}

	async fn init_async(&mut self) -> Result<(), I2C::Error> {
		// Hardware reset sequence
		self.cam_rst.set_low().ok();
		self.sleep(5.millis());
		self.cam_rst.set_high().ok();
		self.sleep(20.millis());

		// Init Regs
		self.write_regs_async(&INIT_REGS).await?;
		self.sleep(100.millis());

		self.test_pattern_async(false).await?;

		// self.write_reg(0x4202, 0xFF)?;
		self.write_reg_async(0x3008, 0x02).await?; // Exit software power down
		let id = self.get_sensor_id_async().await?;
		log::debug!("Sensor ID: {:#X}", id);
		assert_eq!(id, 0x5640);

		let mut buf = [0u8; 1];
		self.read_reg_async(0x3039, &mut buf).await?;
		log::info!("PLL bypass: {:#x}", buf[0]);

		Ok(())
	}

	async fn write_reg_async(&mut self, reg: u16, val: u8) -> Result<(), I2C::Error> {
		log::info!("Writing {:#X} to {:#X}", val, reg);
		self.i2c
			.write(CAM_ADDR, &[(reg >> 8) as u8, reg as u8, val]).await?;
		Ok(())
	}

	async fn write_regs_async(&mut self, regs: &[(u16, u8)]) -> Result<(), I2C::Error> {
		for (reg, val) in regs {
			if *reg == REG_DLY {
				self.sleep((*val as u32).millis());
			} else {
				self.write_reg_async(*reg, *val).await?;
			}
		}
		Ok(())
	}

	async fn read_reg_async(&mut self, reg: u16, buf: &mut [u8]) -> Result<(), I2C::Error> {
		log::info!("Reading from {:#X}", reg);
		self.i2c
			.write_read(CAM_ADDR, &[(reg >> 8) as u8, reg as u8], buf).await?;
		Ok(())
	}

	/// Read a complete frame from the camera
	/// This will hold the mutable reference to self until the frame is processed
	/// The returned reference looks into a static buffer, be aware that this maye cause Strange
	/// Issues.
	pub async fn read_frame_async(&mut self) -> Result<&'static mut FrameBuf, I2C::Error> {
		//self.write_reg(0x4202, 0x00).await?;
		//self.write_reg(0x3008, 0x02).await?; // Exit software power down
		let xfer = self.pcc_xfer_handle.as_mut().unwrap();
		// log::info!("Waiting for VSync");
		// while self.cam_sync.den1.is_high() {};
		let other_buf = self.other_buffer.take().map(|b| unsafe{ b.as_mut() }).flatten().unwrap();
		while !xfer.complete() {}
		let framebuffer = xfer.recycle_source(other_buf).unwrap();

        self.other_buffer.replace(framebuffer);
		//if pcc.read_flags().ovre().bit_is_set() {
		// 	log::error!("PCC Overrun!")
		//  }
		// 	pcc.configure(|_| ()); // Off and on again
		let frame: &Frame = unsafe { core::mem::transmute(&mut *framebuffer) };
		//let frame: &Frame = &[[Pixel::default(); H_RES]; V_RES];
		// let frame_u16: &[u16; H_RES * V_RES] = unsafe {core::mem::transmute(&mut *framebuffer)};
		// Stop frame capture
		//self.write_reg(0x3008, 0x42).await?; // Enter software power down
		// self.write_reg(0x4202, 0xFF).await?;

		// self.write_reg(0x4202, 0xFF).await?; // Mask Frames

		//for read in framebuffer.iter_mut() {
		//    *read = read.to_be();
		//}

		//log::info!("{:?}", framebuffer);


		// self.write_reg(0x4202, 0x00).await?; // unmask frames
		//delay().delay_ms(100);
		//	log::debug!("Frame Mangling done");
		Ok(framebuffer)
	}

	/// Configure windowing
	pub async fn set_window_async(
		&mut self,
		x_start: u16,
		y_start: u16,
		width: u16,
		height: u16,
	) -> Result<(), I2C::Error> {
		let x_end = x_start + width - 1;
		let y_end = y_start + height - 1;

		// Set window coordinates
		self.write_reg_async(0x3800, (x_start >> 8) as u8).await?; // X start high
		self.write_reg_async(0x3801, x_start as u8).await?; // X start low
		self.write_reg_async(0x3802, (y_start >> 8) as u8).await?; // Y start high
		self.write_reg_async(0x3803, y_start as u8).await?; // Y start low
		self.write_reg_async(0x3804, (x_end >> 8) as u8).await?; // X end high
		self.write_reg_async(0x3805, x_end as u8).await?; // X end low
		self.write_reg_async(0x3806, (y_end >> 8) as u8).await?; // Y end high
		self.write_reg_async(0x3807, y_end as u8).await?; // Y end low

		// Set output size to match window
		self.write_reg_async(0x3808, (width >> 8) as u8).await?; // Output width high
		self.write_reg_async(0x3809, width as u8).await?; // Output width low
		self.write_reg_async(0x380A, (height >> 8) as u8).await?; // Output height high
		self.write_reg_async(0x380B, height as u8).await?; // Output height low

		Ok(())
	}

	/// Util for region -> window
	pub async fn set_window_region_async(
		&mut self,
		region: (usize, usize, usize, usize),
	) -> Result<(), I2C::Error> {
		self.set_window_async(
			region.0 as u16,
			region.1 as u16,
			region.2 as u16,
			region.3 as u16,
		).await
	}

	/// Set manual exposure
	pub async fn set_exposure_async(&mut self, exposure: u32) -> Result<(), I2C::Error> {
		self.write_reg_async(0x3500, (exposure >> 12) as u8).await?; // Exposure[19:16]
		self.write_reg_async(0x3501, (exposure >> 4) as u8).await?; // Exposure[15:8]
		self.write_reg_async(0x3502, (exposure << 4) as u8).await?; // Exposure[7:0]
		Ok(())
	}

	/// Set manual gain
	pub async fn set_gain_async(&mut self, gain: u16) -> Result<(), I2C::Error> {
		self.write_reg_async(0x350A, (gain >> 8) as u8).await?; // Gain[9:8]
		self.write_reg_async(0x350B, gain as u8).await?; // Gain[7:0]
		Ok(())
	}

	/// Enable/disable test pattern for debugging
	pub async fn test_pattern_async(&mut self, on: bool) -> Result<(), I2C::Error> {
		if on {
			self.write_reg_async(0x503D, 0x80).await?;
			self.write_reg_async(0x4741, 0x00).await?;
		} else {
			self.write_reg_async(0x503D, 0x00).await?; // Test pattern disable
		}
		Ok(())
	}

	/// Verify sensor is responding with correct chip ID
	pub async fn get_sensor_id_async(&mut self) -> Result<u16, I2C::Error> {
		let mut id = [0u8; 2];

		self.read_reg_async(0x300A, &mut id[0..1]).await?; // Chip ID high byte
		self.read_reg_async(0x300B, &mut id[1..2]).await?; // Chip ID low byte

		Ok(u16::from_be_bytes(id))
	}
}
pub struct FilterRegions {
	pub filter_1: (usize, usize, usize, usize), // (x, y, width, height)
	pub filter_2: (usize, usize, usize, usize),
	pub filter_3: (usize, usize, usize, usize),
	pub filter_4: (usize, usize, usize, usize),
}

impl Default for FilterRegions {
	fn default() -> Self {
		Self {
			filter_1: (0, 0, H_RES / 2, V_RES / 2), // Top-left quadrant
			filter_2: (H_RES / 2, 0, H_RES / 2, V_RES / 2), // Top-right quadrant
			filter_3: (0, V_RES / 2, H_RES / 2, V_RES / 2), // Bottom-left quadrant
			filter_4: (H_RES / 2, V_RES / 2, H_RES / 2, V_RES / 2), // Bottom-right quadrant
		}
	}
}
