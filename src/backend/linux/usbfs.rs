use std::os::raw::c_void;

use nix::{ioctl_read, ioctl_readwrite, ioctl_write_ptr};

use super::usbfs_c::{usbdevfs_bulktransfer, usbdevfs_ctrltransfer, usbdevfs_urb};

ioctl_readwrite!(usbdevfs_control, b'U', 0, usbdevfs_ctrltransfer);
ioctl_readwrite!(usbdevfs_bulk, b'U', 2, usbdevfs_bulktransfer);
ioctl_read!(usbdevfs_submiturb, b'U', 10, usbdevfs_urb);
ioctl_write_ptr!(usbdevfs_reapurb, b'U', 12, *mut c_void);
ioctl_write_ptr!(usbdevfs_reapurbndelay, b'U', 13, *mut c_void);

/// Performs an IOCTL and logs the result code with [log::debug] or a specified [log::Level],
/// specified with `ioctl_log!(ioctl_func, fd, arg, loglevel: log::Level::Warn)`.
macro_rules! ioctl_log {
    ($ioctl_func:expr, $fd:expr, $arg:expr, loglevel: $loglevel:expr) => {{
        let res = $ioctl_func($fd, $arg);
        let ret_code = $crate::backend::linux::nix_result_to_code(&res);
        let short_ioctl_name = core::stringify!($ioctl_func);
        // HACK: strip the module name this will usually be qualified with :)
        let short_ioctl_name = match short_ioctl_name.strip_prefix("usbfs::") {
            Some(s) => s.to_ascii_uppercase(),
            None => short_ioctl_name.to_ascii_uppercase(),
        };
        log::log!($loglevel, "{} ioctl ret = {}", short_ioctl_name, ret_code);
        res
    }};
    ($ioctl_func:expr, $fd:expr, $arg:expr) => {{
        $crate::backend::linux::usbfs::ioctl_log!(
            $ioctl_func,
            $fd,
            $arg,
            loglevel: log::Level::Debug
        )
    }};
}

pub(crate) use ioctl_log;
