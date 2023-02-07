//! Internal helpers for working with FFIs.

use std::time::Duration;

use log::warn;

/// Extension trait for [Duration], mainly to help passing strongly-typed timeout values to the
/// OS-expected `u32` or `i32` millisecond values with warnings for truncation.
pub(crate) trait DurationExt<N> {
    /// Converts the duration to milliseconds, truncating to the target type if necessary and
    /// logging a warning (with [log::warn], and tracking the caller of this function) if so.
    #[track_caller]
    fn as_millis_truncated(&self) -> N;
}

impl DurationExt<i32> for Duration {
    /// Converts the duration to milliseconds, truncating to `i32::MAX` if necessary and
    /// logging a warning (with [log::warn], and tracking the caller of this function) if so.
    #[track_caller]
    fn as_millis_truncated(&self) -> i32 {
        if self.as_millis() > i32::MAX as u128 {
            // Use the caller of this function as the log target, as that will be far more
            // relevant to the user.
            let caller = std::panic::Location::caller().file();
            warn!(
                target: caller,
                "A wildly long timeout ({}s) was truncated to i32::MAX ({}s)",
                self.as_secs_f64(),
                i32::MAX,
            );
            i32::MAX
        } else {
            self.as_millis() as i32
        }
    }
}

impl DurationExt<u32> for Duration {
    /// Converts the duration to milliseconds, truncating to `u32::MAX` if necessary and
    /// logging a warning (with [log::warn], and tracking the caller of this function) if so.
    #[track_caller]
    fn as_millis_truncated(&self) -> u32 {
        if self.as_millis() > u32::MAX as u128 {
            // Use the caller of this function as the log target, as that will be far more
            // relevant to the user.
            let caller = std::panic::Location::caller().file();
            warn!(
                target: caller,
                "A wildly long timeout ({}s) was truncated to u32::MAX ({}s)",
                self.as_secs_f64(),
                u32::MAX,
            );
            u32::MAX
        } else {
            self.as_millis() as u32
        }
    }
}

/// Extension trait for [Option<Duration>], mainly to help passing strongly-typed timeout values to the
/// OS-expected `u32` or `i32` millisecond values with warnings for truncation, defaulting to the
/// caller-provided sentinel if the timeout was specified as `None`.
pub trait OptDurationExt<N> {
    #[track_caller]
    fn as_millis_truncated_or(self, optb: N) -> N;
}

impl OptDurationExt<i32> for Option<Duration> {
    /// Converts a `Some` duration to milliseconds, truncating to `i32::MAX` if necessary and
    /// logging a warning (with [log::warn], and tracking the caller of this function) if so, and
    /// converts a `None` duration to the provided sentinel value.
    #[track_caller]
    fn as_millis_truncated_or(self, optb: i32) -> i32 {
        // Because [DurationExt::as_millis_truncated] uses `track_caller`, we have to reimplement
        // its functional instead of delegating. That's fine; it's pretty small.

        match self {
            Some(duration) => {
                if duration.as_millis() > i32::MAX as u128 {
                    // Use the caller of this function as the log target, as that will be far more
                    // relevant to the user.
                    let caller = std::panic::Location::caller().file();
                    warn!(
                        target: caller,
                        "A wildly long timeout ({}s) was truncated to i32::MAX ({}s)",
                        duration.as_secs_f64(),
                        i32::MAX,
                    );
                    i32::MAX
                } else {
                    i32::MAX
                }
            }
            None => optb,
        }
    }
}

impl OptDurationExt<u32> for Option<Duration> {
    /// Converts a `Some` duration to milliseconds, truncating to `u32::MAX` if necessary and
    /// logging a warning (with [log::warn], and tracking the caller of this function) if so, and
    /// converts a `None` duration to the provided sentinel value.
    #[track_caller]
    fn as_millis_truncated_or(self, optb: u32) -> u32 {
        // Because [DurationExt::as_millis_truncated] uses `track_caller`, we have to reimplement
        // its functional instead of delegating. That's fine; it's pretty small.

        match self {
            Some(duration) => {
                if duration.as_millis() > u32::MAX as u128 {
                    // Use the caller of this function as the log target, as that will be far more
                    // relevant to the user.
                    let caller = std::panic::Location::caller().file();
                    warn!(
                        target: caller,
                        "A wildly long timeout ({}s) was truncated to u32::MAX ({}s)",
                        duration.as_secs_f64(),
                        u32::MAX,
                    );
                    u32::MAX
                } else {
                    u32::MAX
                }
            }
            None => optb,
        }
    }
}
