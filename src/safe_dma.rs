use core::mem::ManuallyDrop;

use atsamd_hal::dmac::{
    Buffer, BufferPair, Busy, ChId, Channel, Ready, Transfer, TriggerAction, TriggerSource,
};

type ReadyChannel<Id> = Channel<Id, Ready>;
type BusyChannel<Id> = Channel<Id, Busy>;

pub(crate) struct SafeTransfer<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> {
    inner: ManuallyDrop<Transfer<BusyChannel<C>, BufferPair<S, D>>>,
}

impl<C: ChId, S: Buffer<Beat = u8>, D: Buffer<Beat = u8>> SafeTransfer<C, S, D> {
    /// Construct a DMA transfer for non `'static` buffers.
    ///
    /// # Panics
    /// Will panic if contructed with an invalid BufferPair, where two buffers of len > 1 do not
    /// match in length
    pub fn new(channel: ReadyChannel<C>, source: S, dest: D) -> Self {
        assert!(
            source.buffer_len() == dest.buffer_len()
                || source.buffer_len() == 1
                || dest.buffer_len() == 1
        );

        // SAFETY: We just asserted that the Bufferpair is valid - and the Drop impl ensures the
        // transfer is stopped before the `SafeTransfer` is destroyed
        let transfer = unsafe {
            Transfer::new_unchecked(channel, source, dest, false)
                .begin(TriggerSource::PccRx, TriggerAction::Burst)
        };

        Self {
            inner: ManuallyDrop::new(transfer),
        }
    }

    pub fn xfer(&mut self) -> &mut Transfer<BusyChannel<C>, BufferPair<S, D>> {
        &mut self.inner
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
