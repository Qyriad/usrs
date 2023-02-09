//! Convenience functions to make working with the library easier.

use std::{sync::Arc, sync::RwLock};

use crate::ReadBuffer;

/// Convenience function that creates a read buffer suitable for use with our async functions.
pub fn create_read_buffer(size: usize) -> ReadBuffer {
    Arc::new(RwLock::new(vec![0; size]))
}

/// Format a byte slice but elide repeats over 10. Output looks like this:
/// `[80, 06, 00, 01, 00, 00, ff, 255, 00, <65534 elided 0s>]`.
#[doc(hidden)]
pub fn format_bytes_with_elision(data: &[u8]) -> String {
    #[derive(Debug, Copy, Clone, PartialEq, Default)]
    struct Repeat {
        value: u8,
        count: usize,
    }

    fn maybe_elide(s: &mut String, repeat: Repeat, suffix: &str) {
        if repeat.count > 10 {
            s.push_str(&format!(
                "<{} elided {}s>{}",
                repeat.count, repeat.value, suffix
            ));
        } else {
            for _ in 0..repeat.count {
                s.push_str(&format!("{:02x}{}", repeat.value, suffix));
            }
        }
    }

    let mut out_str = String::new();
    let mut repeat = Repeat::default();
    out_str.push('[');

    for byte in data.iter().copied().take(data.len() - 1) {
        if byte == repeat.value {
            repeat.count += 1;
        } else {
            maybe_elide(&mut out_str, repeat, ", ");
            out_str.push_str(&format!("{:02x}, ", byte));
            repeat = Repeat {
                value: byte,
                count: 0,
            };
        }
    }

    let last = data.last().unwrap();

    if *last == repeat.value {
        repeat.count += 1;
        maybe_elide(&mut out_str, repeat, "]");
    } else {
        out_str.push_str(&format!("{:02x}]", last));
    }

    out_str
}
