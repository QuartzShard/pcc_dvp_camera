use core::mem::ManuallyDrop;

use atsamd_hal::dmac::{
    Buffer, BufferPair, BurstLength, Busy, ChId, Channel, FifoThreshold, PriorityLevel, Ready,
    Status, Transfer, TriggerAction, TriggerSource, Uninitialized, UninitializedFuture,
};

pub trait Uninit: Status {}
impl Uninit for Uninitialized {}
impl Uninit for UninitializedFuture {}

enum Xfer<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> {
    Running(ManuallyDrop<Transfer<Channel<C, Busy>, BufferPair<S, D>>>),
    Stopped(ManuallyDrop<Transfer<Channel<C, Ready>, BufferPair<S, D>>>),
}

impl<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> Xfer<C, S, D> {
    pub fn start_reinit(&mut self, priority: PriorityLevel) {
        if let Xfer::Stopped(xfer) = self {
            let (channel, source, dest) = unsafe { ManuallyDrop::take(xfer) }.free();
            let channel = channel.reset();
            *self = Xfer::Running(ManuallyDrop::new(
                unsafe { Self::make_transfer_reinit(channel, source, dest, priority) }
                    .begin(TriggerSource::PccRx, TriggerAction::Block),
            ))
        }
    }
    pub fn start(&mut self) {
        if let Xfer::Stopped(xfer) = self {
            let (channel, source, dest) = unsafe { ManuallyDrop::take(xfer) }.free();
            *self = Xfer::Running(ManuallyDrop::new(
                unsafe { Self::make_transfer(channel, source, dest) }
                    .begin(TriggerSource::PccRx, TriggerAction::Block),
            ))
        }
    }
    pub fn stop(&mut self) {
        if let Xfer::Running(xfer) = self {
            let (channel, source, dest) = unsafe { ManuallyDrop::take(xfer) }.stop();
            *self = Xfer::Stopped(ManuallyDrop::new(unsafe {
                Self::make_transfer(channel, source, dest)
            }));
        }
    }

    unsafe fn make_transfer(
        channel: Channel<C, Ready>,
        source: S,
        dest: D,
    ) -> Transfer<Channel<C, Ready>, BufferPair<S, D>> {
        unsafe { Transfer::new_unchecked(channel, source, dest, true) }
    }

    unsafe fn make_transfer_reinit<R: Uninit>(
        channel: Channel<C, R>,
        source: S,
        dest: D,
        priority: PriorityLevel,
    ) -> Transfer<Channel<C, Ready>, BufferPair<S, D>> {
        unsafe {
            // SAFETY: This is valid as the only difference is PhantomData, so long as we don't
            // break contract by enabling the TCMPL interrupt or coercing a busy/ready channel to
            // uninit (enforced by R: Uninit)
            let channel: Channel<C, Uninitialized> = core::mem::transmute(channel);
            let mut channel = channel.init(priority);
            channel.burst_length(BurstLength::Single);
            channel.fifo_threshold(FifoThreshold::_8beats);
            Self::make_transfer(channel, source, dest)
        }
    }

    pub fn swap(&mut self, buf1: D) -> D {
        let (channel, source, buf2) = match self {
            Xfer::Running(xfer) => unsafe { ManuallyDrop::take(xfer) }.stop(),
            Xfer::Stopped(xfer) => unsafe { ManuallyDrop::take(xfer) }.free(),
        };
        *self = Xfer::Running(ManuallyDrop::new(
            unsafe { Self::make_transfer(channel, source, buf1) }
                .begin(TriggerSource::PccRx, TriggerAction::Block),
        ));
        buf2
    }

    /// Stop the transfer (if running), and call a closure with handles to the source and
    /// destination
    pub unsafe fn with_inner(&mut self, f: impl FnOnce(&mut S, &mut D)) {
        self.stop();
        // Always true
        let Self::Stopped(xfer) = self else { return };
        let (channel, mut source, mut dest) = unsafe { ManuallyDrop::take(xfer) }.free();
        f(&mut source, &mut dest);
        *self = Xfer::Stopped(ManuallyDrop::new(unsafe {
            Self::make_transfer(channel, source, dest)
        }));
    }
}

pub(crate) struct SafeTransfer<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> {
    inner: Xfer<C, S, D>,
    priority: PriorityLevel,
}

impl<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> SafeTransfer<C, S, D> {
    /// Construct a DMA transfer for non `'static` buffers.
    ///
    /// # Panics
    /// Will panic if contructed with an invalid BufferPair, where two buffers of len > 1 do not
    /// match in length
    pub fn new<R: Uninit>(
        channel: Channel<C, R>,
        source: S,
        dest: D,
        priority: PriorityLevel,
    ) -> Self {
        assert!(
            source.buffer_len() == dest.buffer_len()
                || source.buffer_len() == 1
                || dest.buffer_len() == 1
        );

        // SAFETY: We just asserted that the Bufferpair is valid - and the Drop impl ensures the
        // transfer is stopped before the `SafeTransfer` is destroyed
        let transfer = Xfer::Stopped(ManuallyDrop::new(unsafe {
            Xfer::make_transfer_reinit(channel, source, dest, priority)
        }));

        Self {
            inner: transfer,
            priority,
        }
    }

    pub fn start(&mut self) {
        self.inner.start()
    }

    pub fn start_reinit(&mut self, priority: PriorityLevel) {
        self.inner.start_reinit(priority)
    }

    pub fn stop(&mut self) {
        self.inner.stop()
    }

    pub fn swap(&mut self, buf1: D) -> D {
        self.inner.swap(buf1)
    }

    /// Restart transfer, optionally at a new priority level,
    pub fn restart(&mut self, priority: Option<PriorityLevel>) {
        self.priority = priority.unwrap_or(self.priority);
        self.stop();
        self.start_reinit(self.priority)
    }

    /// Stop the transfer (if running), and call a closure with handles to the source and
    /// destination. Ensure you don't modify the length of anything
    pub unsafe fn with_buffers(&mut self, f: impl FnOnce(&mut S, &mut D)) {
        unsafe { self.inner.with_inner(f) }
    }
}

impl<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> Drop for SafeTransfer<C, S, D> {
    fn drop(&mut self) {
        // SAFETY: We're in drop - the transfer is to be stopped and then dropped. Nothing can
        // reference this again after this
        unsafe {
            match &mut self.inner {
                Xfer::Running(transfer) => &mut ManuallyDrop::take(transfer).stop(),
                Xfer::Stopped(transfer) => &mut ManuallyDrop::take(transfer).free(),
            }
        };
    }
}
