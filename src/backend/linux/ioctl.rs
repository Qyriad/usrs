use std::os::raw::c_void;

use nix::{ioctl_read, ioctl_readwrite, ioctl_write_ptr};

use super::ioctl_c::{usbdevfs_ctrltransfer, usbdevfs_urb};

ioctl_readwrite!(usbdevfs_control, b'U', 0, usbdevfs_ctrltransfer);
ioctl_read!(usbdevfs_submiturb, b'U', 10, usbdevfs_urb);
ioctl_write_ptr!(usbdevfs_reapurb, b'U', 12, *mut c_void);
