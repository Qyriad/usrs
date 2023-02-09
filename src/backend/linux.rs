//! Core, low-level functionality for Linux.

use std::{
    fs::{self, File},
    path::{Path, PathBuf},
    time::Duration,
};

use log::{error, trace, warn};
use nix::unistd::SysconfVar;

use crate::{
    backend::linux::device::LinuxDevice, device::Device, request::Direction, DeviceInformation,
    Error, UsbResult,
};

use super::{Backend, BackendDevice};

mod device;
mod enumeration;
mod usbfs;
mod usbfs_c;

/// Per-OS data for the Linux backend.
#[derive(Debug)]
pub struct LinuxBackend {
    sysfs_root: PathBuf,
    devfs_root: PathBuf,
    page_size: usize,
}

impl LinuxBackend {
    const DEFAULT_SYSFS_ROOT: &'static str = "/sys";
    const DEFAULT_DEVFS_ROOT: &'static str = "/dev";

    /// Initializes a Linux backend using sensible defaults. Use [with_paths] if you need more
    /// granularity.
    pub fn new() -> UsbResult<Self> {
        let page_size = nix::unistd::sysconf(SysconfVar::PAGE_SIZE)
            .ok()
            .flatten()
            .unwrap_or(4096) as usize;

        Ok(LinuxBackend {
            sysfs_root: PathBuf::from(Self::DEFAULT_SYSFS_ROOT),
            devfs_root: PathBuf::from(Self::DEFAULT_DEVFS_ROOT),
            page_size,
        })
    }

    /// Initialize a Linux backend and override the sysfs and devfs root paths (/sys and /dev
    /// by default).
    // FIXME: This constructor is not accessible currently as the entire module is not.
    pub fn with_paths(sysfs_root: Option<PathBuf>, devfs_root: Option<PathBuf>) -> UsbResult<Self> {
        let page_size = nix::unistd::sysconf(SysconfVar::PAGE_SIZE)
            .ok()
            .flatten()
            .unwrap_or(4096) as usize;
        Ok(LinuxBackend {
            sysfs_root: sysfs_root.unwrap_or_else(|| PathBuf::from(Self::DEFAULT_SYSFS_ROOT)),
            devfs_root: devfs_root.unwrap_or_else(|| PathBuf::from(Self::DEFAULT_DEVFS_ROOT)),
            page_size,
        })
    }

    fn device_backend(device: &Device) -> &LinuxDevice {
        unsafe { device.backend_data().as_any().downcast_ref().unwrap() }
    }
}

impl Backend for LinuxBackend {
    /// Enumerates USB devices on Linux, using the sysfs if the `linux-sysfs` feature is enabled
    /// and if the sysfs is available, otherwise using the devfs, which is considerably slower
    /// but less likely to be unmounted or permissions-unavailable.
    fn get_devices(&self) -> UsbResult<Vec<DeviceInformation>> {
        // Device information in Linux is split between the sysfs (/sys/bus/usb/) and the devfs
        // (/dev/bus/usb/).
        // If the sysfs feature is enabled, we'll try to read that, and fallback to the devfs if
        // that fails.

        if cfg!(feature = "linux-sysfs") {
            match enumeration::enumerate_with_sysfs(&self.sysfs_root) {
                Ok(results) => return Ok(results),
                Err(e) => {
                    warn!("Error enumerating via sysfs: {}; trying devfs instead", e);
                    return enumeration::enumerate_with_devfs(&self.devfs_root);
                }
            }
        }

        return enumeration::enumerate_with_devfs(&self.devfs_root);
    }

    fn open(&self, information: &DeviceInformation) -> UsbResult<Box<dyn BackendDevice>> {
        trace!("Attempting to open device for {:?}", information);

        let (busnum, devnum) = unpack_numeric_location(
            information
                .backend_numeric_location
                .expect("Enumeration should have set backend numeric location"),
        );

        let usbdev_path = self
            .devfs_root
            .join(&format!("bus/usb/{:03}/{:03}", busnum, devnum));

        let usbdev_file = File::options().read(true).write(true).open(&usbdev_path)?;

        Ok(Box::new(LinuxDevice { file: usbdev_file }))
    }

    fn release_kernel_driver(&self, device: &mut Device, interface: u8) -> UsbResult<()> {
        todo!()
    }

    fn claim_interface(&self, device: &mut Device, interface: u8) -> UsbResult<()> {
        todo!()
    }

    fn unclaim_interface(&self, device: &mut Device, interface: u8) -> UsbResult<()> {
        todo!()
    }

    fn active_configuration(&self, device: &Device) -> UsbResult<u8> {
        todo!()
    }

    fn set_active_configuration(&self, device: &Device, configuration_index: u8) -> UsbResult<()> {
        todo!()
    }

    fn reset_device(&self, device: &Device) -> UsbResult<()> {
        todo!()
    }

    fn clear_stall(&self, device: &Device, endpoint_address: u8) -> UsbResult<()> {
        todo!()
    }

    fn set_alternate_setting(&self, device: &Device, interface: u8, setting: u8) -> UsbResult<()> {
        todo!()
    }

    fn current_bus_frame(&self, device: &Device) -> UsbResult<(u64, std::time::SystemTime)> {
        todo!()
    }

    fn control_read(
        &self,
        device: &Device,
        request_type: u8,
        request_number: u8,
        value: u16,
        index: u16,
        target: &mut [u8],
        timeout: Option<Duration>,
    ) -> UsbResult<usize> {
        let backend_device: &LinuxDevice = Self::device_backend(&device);

        // The synchronous USBDEVFS_CONTROL IOCTL disallows control transfers where
        // wLength > PAGE_SIZE
        // (https://elixir.bootlin.com/linux/v6.1/source/drivers/usb/core/devio.c#L1173).
        // Some HCD drivers *also* have a similar limitation on urb submission,
        // but the increasingly common XHCI HCD does not.
        // Either way, for control transfers of that size we have to use the asynchronous API to
        // bypass USBDEVFS_CONTROL's limitation. This introduces a separate limitation that
        // requires a secondary allocation, however. See [control_async_blocking] for
        // details.
        unsafe {
            if target.len() < self.page_size {
                backend_device.control_sync(
                    request_type,
                    request_number,
                    value,
                    index,
                    target,
                    timeout,
                )
            } else {
                backend_device.control_async_blocking(
                    request_type,
                    request_number,
                    value,
                    index,
                    target as *mut [u8],
                    timeout,
                    Direction::In,
                )
            }
        }
    }

    fn control_read_nonblocking(
        &self,
        device: &Device,
        request_type: u8,
        request_number: u8,
        value: u16,
        index: u16,
        target: crate::ReadBuffer,
        callback: Box<dyn FnOnce(UsbResult<usize>)>,
        timeout: Option<Duration>,
    ) -> UsbResult<()> {
        todo!()
    }

    fn control_write(
        &self,
        device: &Device,
        request_type: u8,
        request_number: u8,
        value: u16,
        index: u16,
        data: &[u8],
        timeout: Option<Duration>,
    ) -> UsbResult<()> {
        let backend_device: &LinuxDevice = Self::device_backend(&device);

        // The synchronous USBDEVFS_CONTROL IOCTL disallows control transfers where
        // wLength > PAGE_SIZE
        // (https://elixir.bootlin.com/linux/v6.1/source/drivers/usb/core/devio.c#L1173).
        // Some HCD drivers *also* have a similar limitation on urb submission, but the
        // increasingly common XHCI HCD does not. Either way, for control transfers of that size we
        // have to use the asynchronous API to bypass USBDEVFS_CONTROL's limitation. This
        // introduces a separate limitation that requires a secondary allocation, however. See
        // [usbfs_control_write_async_blocking] for details.

        unsafe {
            if data.len() < self.page_size {
                backend_device.control_sync(
                    request_type,
                    request_number,
                    value,
                    index,
                    data as *const [u8] as *mut [u8],
                    timeout,
                )?;
            } else {
                backend_device.control_async_blocking(
                    request_type,
                    request_number,
                    value,
                    index,
                    data as *const [u8] as *mut [u8],
                    timeout,
                    Direction::Out,
                )?;
            }
        }

        Ok(())
    }

    fn control_write_nonblocking(
        &self,
        device: &Device,
        request_type: u8,
        request_number: u8,
        value: u16,
        index: u16,
        data: crate::WriteBuffer,
        callback: Box<dyn FnOnce(UsbResult<usize>)>,
        timeout: Option<Duration>,
    ) -> UsbResult<()> {
        todo!()
    }

    fn read(
        &self,
        device: &Device,
        endpoint: u8,
        buffer: &mut [u8],
        timeout: Option<Duration>,
    ) -> UsbResult<usize> {
        if (endpoint & 0x80) != 0x80 {
            error!("OUT endpoint 0x{:02x} specified for IN transfer", endpoint);
            return Err(Error::InvalidArgument);
        }

        let backend_device: &LinuxDevice = Self::device_backend(&device);

        unsafe { backend_device.bulk(endpoint, buffer as *mut [u8], timeout) }
    }

    fn write(
        &self,
        device: &Device,
        endpoint: u8,
        data: &[u8],
        timeout: Option<Duration>,
    ) -> UsbResult<()> {
        if (endpoint & 0x80) != endpoint {
            error!("IN endpoint specified for OUT transfer");
            return Err(Error::InvalidArgument);
        }
        let backend_device = Self::device_backend(&device);

        unsafe {
            backend_device.bulk(endpoint, data as *const [u8] as *mut [u8], timeout)?;
        }

        Ok(())
    }

    fn read_nonblocking(
        &self,
        device: &Device,
        endpoint: u8,
        buffer: crate::ReadBuffer,
        callback: Box<dyn FnOnce(UsbResult<usize>)>,
        timeout: Option<Duration>,
    ) -> UsbResult<()> {
        todo!()
    }

    fn write_nonblocking(
        &self,
        device: &Device,
        endpoint: u8,
        data: crate::WriteBuffer,
        callback: Box<dyn FnOnce(UsbResult<usize>)>,
        timeout: Option<Duration>,
    ) -> UsbResult<()> {
        todo!()
    }
}

fn read_sysfs_attr<P: AsRef<Path>>(sysfs_attr: P) -> UsbResult<String> {
    let sysfs_attr = sysfs_attr.as_ref();

    // Attempt to read the file from the filesystem...
    let attr_result = fs::read_to_string(&sysfs_attr)?;

    // Trim the trailing newline.
    Ok(attr_result.trim().to_string())
}

pub(crate) fn nix_result_to_code<T>(res: &nix::Result<T>) -> i32 {
    match res {
        Ok(_) => 0,
        Err(e) => *e as i32,
    }
}

/// Packs Linux bus number and device number into a single u64, for storing into a
/// [DeviceInformation] struct.
const fn pack_numeric_location(busnum: u32, devnum: u32) -> u64 {
    ((busnum as u64) << 32) | (devnum as u64)
}

/// Unpacks the `backend_numeric_location` stored in a [DeviceInformation] struct into its
/// Linux bus number and device number.
const fn unpack_numeric_location(numeric_location: u64) -> (u32, u32) {
    let busnum = (numeric_location >> 32) as u32;
    let devnum = (numeric_location & 0x00000000_FFFFFFFF) as u32;

    (busnum, devnum)
}
