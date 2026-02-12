use core::mem::ManuallyDrop;

use atsamd_hal::dmac::{
    Buffer, BufferPair, BurstLength, Busy, ChId, Channel, FifoThreshold, PriorityLevel, Status,
    Transfer, TriggerAction, TriggerSource, Uninitialized, UninitializedFuture,
};

type BusyChannel<Id> = Channel<Id, Busy>;

pub trait Uninit: Status {}
impl Uninit for Uninitialized {}
impl Uninit for UninitializedFuture {}

pub(crate) struct SafeTransfer<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> {
    inner: ManuallyDrop<Transfer<BusyChannel<C>, BufferPair<S, D>>>,
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
        let transfer = unsafe { Self::start_transfer_reinit(channel, source, dest, priority) };

        Self {
            inner: transfer,
            priority,
        }
    }

    unsafe fn start_transfer_reinit<R: Uninit>(
        channel: Channel<C, R>,
        source: S,
        dest: D,
        priority: PriorityLevel,
    ) -> ManuallyDrop<Transfer<Channel<C, Busy>, BufferPair<S, D>>> {
        ManuallyDrop::new(unsafe {
            // SAFETY: This is valid as the only difference is PhantomData, so long as we don't
            // break contract by enabling the TCMPL interrupt or coercing a busy/ready channel to
            // uninit (enforced by R: Uninit)
            let channel: Channel<C, Uninitialized> = core::mem::transmute(channel);
            let mut channel = channel.init(priority);
            channel.burst_length(BurstLength::Single);
            channel.fifo_threshold(FifoThreshold::_8beats);
            Transfer::new_unchecked(channel, source, dest, true)
                .begin(TriggerSource::PccRx, TriggerAction::Burst)
        })
    }

    pub fn swap(&mut self, buf1: D) -> D {
        // SAFETY: Nothing here can panic, and it's back in place before return. Do not read
        // self.inner until reassigned
        let xfer = unsafe { ManuallyDrop::take(&mut self.inner) };
        let (ch, pcc, buf2) = xfer.stop();
        // SAFETY: Buf1 is of same type as Buf2, so swapping is valid as long as the transfer was
        // initially
        self.inner = ManuallyDrop::new(unsafe {
            Transfer::new_unchecked(ch, pcc, buf1, true)
                .begin(TriggerSource::PccRx, TriggerAction::Burst)
        });
        buf2
    }

    /// Restart transfer, optionally at a new priority level, and optionally waiting for some sync
    /// condition
    pub fn restart(&mut self, priority: Option<PriorityLevel>) {
        self.priority = priority.unwrap_or(self.priority);
        // SAFETY: Nothing here can panic, and it's back in place before return. Do not read
        // self.inner until reassigned
        let xfer = unsafe { ManuallyDrop::take(&mut self.inner) };
        let (ch, pcc, fb) = xfer.stop();
        let ch = ch.reset();
        // SAFETY: If it was valid before, it's still valid
        self.inner = unsafe { Self::start_transfer_reinit(ch, pcc, fb, self.priority) };
    }
}

impl<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> Drop for SafeTransfer<C, S, D> {
    fn drop(&mut self) {
        // SAFETY: We're in drop - the transfer is to be stopped and then dropped. Nothing can
        // reference this again after this
        let xfer = unsafe { ManuallyDrop::take(&mut self.inner) };
        xfer.stop();
    }
}
