
#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {
        #[cfg(feature = "debug_log")]
        {
            ::defmt::debug!($($arg)*)
        }
    };
}
