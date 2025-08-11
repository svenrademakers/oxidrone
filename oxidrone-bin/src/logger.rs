use core::{
    cell::UnsafeCell,
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, Ordering},
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    zerocopy_channel::{Channel, Receiver, Sender},
};
use static_cell::StaticCell;

pub const LOG_BUF_SIZE: usize = 64;
pub const LOG_LENGTH: usize = 8;
type LogStorageType = heapless::Vec<u8, LOG_BUF_SIZE>;
type LogSender = Sender<'static, CriticalSectionRawMutex, LogStorageType>;
type LogChannel = Channel<'static, CriticalSectionRawMutex, LogStorageType>;

static LOG_BUFFER: StaticCell<[MaybeUninit<LogStorageType>; LOG_LENGTH]> = StaticCell::new();
static CHANNEL: StaticCell<LogChannel> = StaticCell::new();

static ENCODER: BufferedEncoder = BufferedEncoder::new();

struct BufferedEncoder {
    /// A boolean lock
    ///
    /// Is `true` when `acquire` has been called and we have exclusive access to
    /// the rest of this structure.
    taken: AtomicBool,
    /// We need to remember this to exit a critical section
    cs_restore: UnsafeCell<critical_section::RestoreState>,
    buffer: UnsafeCell<LogStorageType>,
    encoder: UnsafeCell<defmt::Encoder>,
    log_sender: UnsafeCell<Option<LogSender>>,
}

impl BufferedEncoder {
    const fn new() -> Self {
        Self {
            taken: AtomicBool::new(false),
            cs_restore: UnsafeCell::new(critical_section::RestoreState::invalid()),
            buffer: UnsafeCell::new(LogStorageType::new()),
            encoder: UnsafeCell::new(defmt::Encoder::new()),
            log_sender: UnsafeCell::new(None),
        }
    }

    fn set_logger(&self, logger: LogSender) {
        unsafe {
            let restore = critical_section::acquire();
            *self.log_sender.get() = Some(logger);
            critical_section::release(restore);
        }
    }

    fn acquire(&self) {
        // safety: Must be paired with corresponding call to release(), see below
        let restore = unsafe { critical_section::acquire() };

        // NB: You can re-enter critical sections but we need to make sure
        // no-one does that.
        if self.taken.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }

        // no need for CAS because we are in a critical section
        self.taken.store(true, Ordering::Relaxed);

        // safety: accessing the cell is OK because we have acquired a critical
        // section.
        unsafe {
            self.cs_restore.get().write(restore);
            let encoder: &mut defmt::Encoder = &mut *self.encoder.get();
            let buffer: &mut LogStorageType = &mut *self.buffer.get();

            buffer.clear();
            encoder.start_frame(|b| {
                let _ = buffer.extend_from_slice(b);
            })
        }
    }

    fn write(&self, bytes: &[u8]) {
        if !self.taken.load(Ordering::Relaxed) {
            panic!("defmt write out of context")
        }

        // safety: accessing the cell is OK because we have acquired a critical
        // section.
        unsafe {
            let encoder: &mut defmt::Encoder = &mut *self.encoder.get();
            let buffer: &mut LogStorageType = &mut *self.buffer.get();
            encoder.write(bytes, |b| {
                let _ = buffer.extend_from_slice(b);
            });
        }
    }

    fn release(&self) {
        if !self.taken.load(Ordering::Relaxed) {
            panic!("defmt release out of context")
        }

        // safety: accessing the cell is OK because we have acquired a critical
        // section.
        unsafe {
            let encoder: &mut defmt::Encoder = &mut *self.encoder.get();
            let buffer: &mut LogStorageType = &mut *self.buffer.get();
            let log_sender: &mut Option<LogSender> = &mut *self.log_sender.get();

            encoder.end_frame(|b| {
                let _ = buffer.extend_from_slice(b);
            });

            if let Some(sender) = log_sender.as_mut() {
                // drop all pending items in the channel, ideally we only want to
                // drop the oldest item, however we dont have an api for this on
                // the sending side
                if sender.is_full() {
                    sender.clear();
                }

                if let Some(slot) = sender.try_send() {
                    slot.clear();
                    let _ = slot.extend_from_slice(&buffer);
                    sender.send_done();
                }
            }

            let restore = self.cs_restore.get().read();
            self.taken.store(false, Ordering::Relaxed);

            // paired with exactly one acquire call
            critical_section::release(restore);
        }
    }
}

unsafe impl Sync for BufferedEncoder {}

#[defmt::global_logger]
struct Logger;

unsafe impl defmt::Logger for Logger {
    fn acquire() {
        ENCODER.acquire();
    }

    unsafe fn release() {
        ENCODER.release();
    }

    unsafe fn write(bytes: &[u8]) {
        ENCODER.write(bytes);
    }

    unsafe fn flush() {}
}

#[unsafe(no_mangle)]
fn _defmt_timestamp() -> u64 {
    0
}

pub fn init_logger() -> Receiver<'static, CriticalSectionRawMutex, LogStorageType> {
    let buf = LOG_BUFFER.init([const { MaybeUninit::zeroed() }; LOG_LENGTH]);
    // safety: we zeroed the buffer in the previous call + the LogStorageType is a POD type, which are safe for zeroing.
    let storage =
        unsafe { core::mem::transmute::<_, &'static mut [LogStorageType; LOG_LENGTH]>(buf) };
    let channel = CHANNEL.init(Channel::new(storage));
    let (sender, receiver) = channel.split();
    ENCODER.set_logger(sender);

    receiver
}
