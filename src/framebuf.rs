use core::ops::{Deref, DerefMut};

use atsamd_hal::dmac::Buffer;

#[repr(C, align(2))]
pub struct FrameBuf<const SIZE: usize>([u8; SIZE]);
impl<const SIZE: usize> FrameBuf<SIZE> {
    pub const fn fb_size(h_res: usize, v_res: usize) -> usize {
        let fb_size = h_res * v_res * 2;
        let _ = assert!(fb_size <= 65535, "Frame Buffer too large for single DMA");
        fb_size
    }

    pub const fn new() -> Self {
        let _ = assert!(SIZE <= 65535, "Frame Buffer too large for single DMA");
        Self([0u8; SIZE])
    }

    /// View the buffer as a slice of u16
    ///
    /// # Panics
    ///
    /// Panics if the buffer is not correctly aligned for access as u16
    pub fn as_u16_array(&self) -> &[u16] {
        // SAFETY: Framebuf is `repr(C, align(2))`, so is also a valid array of u16
        let (pre, buf, post) = unsafe { self.0.align_to::<u16>() };
        debug_assert!(pre.is_empty());
        debug_assert!(post.is_empty());
        buf
    }

    /// Reinterpret the buffer as a H_RES*V_RES slice of u16
    ///
    /// # Panics
    ///
    /// Panics if the buffer is not correctly aligned for access as u16
    pub fn as_lines<const H_RES: usize>(&self) -> &[[u16; H_RES]] {
        self.as_u16_array().as_chunks::<{ H_RES }>().0
    }
}

unsafe impl<const SIZE: usize> Buffer for &mut FrameBuf<SIZE> {
    type Beat = u8;

    fn dma_ptr(&mut self) -> *mut Self::Beat {
        (self as &mut [u8; SIZE]).dma_ptr()
    }

    fn incrementing(&self) -> bool {
        true
    }

    fn buffer_len(&self) -> usize {
        SIZE
    }
}

impl<const SIZE: usize> Deref for FrameBuf<SIZE> {
    type Target = [u8; SIZE];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}
impl<const SIZE: usize> DerefMut for FrameBuf<SIZE> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
