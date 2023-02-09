use std::{any::Any, fs::File, os::fd::AsRawFd, ptr, time::Duration};

use libc::c_void;
use log::{debug, error, log_enabled, trace, Level};
use nix::poll::{PollFd, PollFlags};

use crate::{
    backend::linux::{
        usbfs::{self, ioctl_log},
        usbfs_c::{
            __IncompleteArrayField, usbdevfs_bulktransfer, usbdevfs_ctrltransfer, usbdevfs_urb,
            usbdevfs_urb__bindgen_ty_1, USBDEVFS_URB_TYPE_CONTROL,
        },
    },
    ffi::OptDurationExt,
    request::Direction,
    Error, UsbResult,
};

use super::BackendDevice;

/// Internal type storing the state for a raw USB device on Linux.
#[derive(Debug)]
pub(crate) struct LinuxDevice {
    /// The "everything is a file" handle to a USB device on Linux.
    pub(crate) file: File,
}

impl LinuxDevice {
    /// Performs an asynchronous control transfer, then blocks until it completes.
    /// Requires an additional allocation compared to [control_sync].
    ///
    /// Safety: `data` is not written to if the transfer is an OUT transfer.
    /// It is the caller's responsibility to ensure that the supplied `request_type` does not
    /// indicate to the kernel that this is an IN transfer if `data` comes from an immutable
    /// reference.
    pub unsafe fn control_async_blocking(
        &self,
        request_type: u8,
        request_number: u8,
        value: u16,
        index: u16,
        data: *mut [u8],
        timeout: Option<Duration>,
        direction: Direction,
    ) -> UsbResult<usize> {
        let fd = self.file.as_raw_fd();

        // Unlike [usbfs_control_read_true_sync], this function can be used to make control
        // transfers larger than PAGE_SIZE, but with an additional limitation. USBDEVFS_SUBMITURB
        // with USBDEVFS_URB_TYPE_CONTROL requires the setup data to be the first 8 bytes of the
        // buffer. This means we can't simply use the buffer provided by the user, so unfortunately
        // we'll have to reallocate to account for the extra 8 bytes of setup data, as pass that to
        // the kernel.

        let len = unsafe { (&*data).len() };

        // If the data is too long for a control request, error out.
        if len > u16::MAX as usize {
            return Err(Error::Overrun);
        }

        let mut buffer_for_urb: Vec<u8> = vec![0; len + 8];

        let req_len = &(len as u16).to_le_bytes();

        // Copy the setup data into the beginning of the buffer for the URB.
        buffer_for_urb[0] = request_type;
        buffer_for_urb[1] = request_number;
        buffer_for_urb[2..4].copy_from_slice(&value.to_le_bytes());
        buffer_for_urb[4..6].copy_from_slice(&index.to_le_bytes());
        buffer_for_urb[6..8].copy_from_slice(req_len);
        // Then copy the rest of the data into this buffer.
        unsafe { buffer_for_urb[8..].copy_from_slice(&*data) };

        // Now build the USB request block used for usbfs IOCTLs, pointing it to our buffer.
        let mut urb = usbdevfs_urb {
            type_: USBDEVFS_URB_TYPE_CONTROL as u8,
            endpoint: 0,
            status: 0, // Out.
            flags: 0,
            buffer: buffer_for_urb.as_mut_ptr() as *mut c_void,
            // Truncation okay as wLength can't be higher than i16::MAX anyway.
            buffer_length: buffer_for_urb.len() as i32,
            actual_length: 0, // Out.
            start_frame: 0,
            // Unnamed struct fields are not yet implemented in Rust:
            // https://github.com/rust-lang/rust/issues/49804.
            __bindgen_anon_1: usbdevfs_urb__bindgen_ty_1 { stream_id: 0 }, // OUT.
            error_count: 0,                                                // Out.
            signr: 0,                                                      // No idea what this is.
            usercontext: ptr::null_mut(),
            iso_frame_desc: __IncompleteArrayField::new(),
        };

        debug!(
            "Submitting asynchronous control transfer: \
            bmRequestType=0x{:02x} bRequest=0x{:02x} wValue=0x{:04x} wIndex=0x{:04x} wLength=0x{:04x}",
            request_type, request_number, value, index, len,
        );

        if log_enabled!(Level::Trace) {
            trace!(
                "Buffer data for submit: {}",
                crate::convenience::format_bytes_with_elision(&buffer_for_urb)
            );
        }

        // With the URB populated, submit it to the kernel.
        let submit_urb_res = unsafe { ioctl_log!(usbfs::usbdevfs_submiturb, fd, &mut urb) };
        match submit_urb_res {
            Ok(_) => (),
            Err(nix::Error::ENOENT) => return Err(Error::DeviceNotFound),
            Err(nix::Error::EACCES) => return Err(Error::PermissionDenied),
            Err(code) => return Err(Error::OsError(code as i64)),
        }

        // With the URB submitted, we can now poll() for events to be ready, with our timeout.

        let poll_flags = PollFlags::POLLOUT | PollFlags::POLLERR | PollFlags::POLLHUP;
        let poll_fd = PollFd::new(fd, poll_flags);
        // poll() uses -1 as its sentinel value for no timeout.
        let poll_timeout: i32 = timeout.as_millis_truncated_or(-1);

        // poll() returns errors for some cases where we just want to try again,
        // notably EAGAIN or EINTR, so let's loop until it returns anything that's not that.
        loop {
            let poll_res = nix::poll::poll(&mut [poll_fd], poll_timeout);
            match poll_res {
                // We don't actually care which poll flag was triggered.
                Ok(_) => break,
                Err(nix::Error::EAGAIN | nix::Error::EINTR) => continue,
                Err(nix::Error::ETIMEDOUT) => return Err(Error::TimedOut),
                Err(other) => {
                    error!("poll() on device returned an unusual error: {}", other);
                    return Err(Error::OsError(other as i64));
                }
            }
        }

        // With poll() done, we know that some events are ready for us. Now ask the kernel to
        // "reap" URBs for this device, which will update our userland URB struct with the new
        // data. It'll also re-give us a pointer to that data, so we'll use that to soundness
        // check to make sure we didn't just complete some other URB.
        let mut out_urb_ptr: *mut c_void = ptr::null_mut();
        let reapurb_res = unsafe {
            ioctl_log!(
                usbfs::usbdevfs_reapurbndelay,
                fd,
                &mut out_urb_ptr as *mut *mut c_void
            )
        };
        match reapurb_res {
            Ok(_) => (),
            Err(nix::Error::ENOENT) => return Err(Error::DeviceNotFound),
            Err(nix::Error::EACCES) => return Err(Error::PermissionDenied),
            Err(code) => return Err(Error::OsError(code as i64)),
        }

        // Soundness check: to USBDEVFS_REAPURBNDELAY ioctl *should* give us a pointer to the exact
        // same URB we just submitted. Let's make double sure.
        // FIXME: check if this is actually possible.
        assert!(
            ptr::eq(ptr::addr_of_mut!(urb), out_urb_ptr as *mut usbdevfs_urb),
            "Linux completed a different URB than we expected; this is a bug in usrs",
        );

        debug!("urb status = {}", urb.status);

        if log_enabled!(Level::Trace) {
            trace!(
                "Completed buffer data: {}",
                crate::convenience::format_bytes_with_elision(&buffer_for_urb)
            );
        }

        // Now handle any error indicated in the URB's status.
        if urb.status < 0 {
            match nix::Error::from_i32(-urb.status) {
                // FIXME: figure out what other errors should be hand-translated.
                nix::Error::EPIPE => return Err(Error::Stalled),
                _ => return Err(Error::OsError(-urb.status as i64)),
            }
        }

        // If this was an IN transfer, we need to copy the data the kernel gave us back to the
        // caller -- without the 8 bytes of setup data.
        if direction == Direction::In {
            // Safety: it's up to the caller to ensure `data` did not come from an immutable
            // reference.
            unsafe {
                (&mut *data).copy_from_slice(&buffer_for_urb[8..]);
            }
        }

        Ok(urb.actual_length as usize)
    }

    /// Synchronously performs a control request. Can only be used when data length < PAGE_SIZE
    /// (usually 4096).
    ///
    /// Safety: `data` is not written to if the transfer is an OUT transfer.
    /// It is the caller's responsibility to ensure that the supplied `request_type` does not
    /// indicate to the kernel that this is an IN transfer if `data` comes from an immutable
    /// reference.
    pub unsafe fn control_sync(
        &self,
        request_type: u8,
        request_number: u8,
        value: u16,
        index: u16,
        data: *mut [u8],
        timeout: Option<Duration>,
    ) -> UsbResult<usize> {
        let fd = self.file.as_raw_fd();

        let len = unsafe { (&*data).len() };

        // USBDEVFS_CONTROL ioctl uses 0 as its sentinel for no timeout.
        let transfer_timeout: u32 = timeout.as_millis_truncated_or(0);

        let mut control_data = usbdevfs_ctrltransfer {
            bRequestType: request_type,
            bRequest: request_number,
            wValue: value,
            wIndex: index,
            wLength: len as u16,
            timeout: transfer_timeout,
            data: data as *mut c_void,
        };

        debug!(
            "Performing true-synchronous control transfer: \
            bmRequestType=0x{:02x} bRequest=0x{:02x} wValue=0x{:04x} wIndex=0x{:04x} wLength=0x{:04x}",
            request_type, request_number, value, index, len,
        );

        let control_res = unsafe { ioctl_log!(usbfs::usbdevfs_control, fd, &mut control_data) };
        let len_transferred = match control_res {
            Ok(actual_len) => actual_len,
            // FIXME: figure out what other errors should be hand-translated.
            Err(nix::Error::ENOENT) => return Err(Error::DeviceNotFound),
            Err(nix::Error::EACCES) => return Err(Error::PermissionDenied),
            Err(nix::Error::EPIPE) => return Err(Error::Stalled),
            Err(nix::Error::ETIMEDOUT) => return Err(Error::TimedOut),
            Err(other) => return Err(Error::OsError(other as i64)),
        };

        Ok(len_transferred as usize)
    }

    /// Performs a bulk transfer.
    pub unsafe fn bulk(
        &self,
        endpoint: u8,
        data: *mut [u8],
        timeout: Option<Duration>,
    ) -> UsbResult<usize> {
        let fd = self.file.as_raw_fd();

        let len = unsafe { (&*data).len() };

        // USBDEVFS_BULK ioctl uses 0 as its sentinel for no timeout.
        let timeout_for_transfer: u32 = timeout.as_millis_truncated_or(0);

        let mut transfer = usbdevfs_bulktransfer {
            ep: endpoint as u32,
            len: len as u32,
            timeout: timeout_for_transfer,
            data: data as *mut c_void,
        };

        debug!("Performing synchronous bulk transfer of length {}", len);

        let bulk_res = unsafe { ioctl_log!(usbfs::usbdevfs_bulk, fd, &mut transfer) };
        let len = match bulk_res {
            Ok(len) => len,
            // FIXME: figure out what other errors should be hand-translated.
            Err(nix::Error::ENOENT) => return Err(Error::DeviceNotFound),
            Err(nix::Error::EACCES) => return Err(Error::PermissionDenied),
            Err(nix::Error::ETIMEDOUT) => return Err(Error::TimedOut),
            Err(other) => return Err(Error::OsError(other as i64)),
        };

        Ok(len as usize)
    }
}

impl BackendDevice for LinuxDevice {
    fn as_mut_any(&mut self) -> &mut dyn Any {
        self
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}
