//! Core, low-level functionality for Linux.

use std::{
    fs, io::Error as IoError, io::ErrorKind as IoErrorKind, os::unix::prelude::OsStrExt,
    path::Path, time::Duration,
};

use log::{error, warn};
use tap::tap::TapFallible;

use crate::{device::Device, DeviceInformation, Error, UsbResult};

use super::{Backend, BackendDevice};

/// Per-OS data for the Linux backend.
#[derive(Debug)]
pub struct LinuxBackend;

impl LinuxBackend {
    pub fn new() -> UsbResult<Self> {
        Ok(LinuxBackend)
    }
}

impl Backend for LinuxBackend {
    fn get_devices(&self) -> UsbResult<Vec<DeviceInformation>> {
        let sysfs_usb_devices: Vec<_> = fs::read_dir("/sys/bus/usb/devices")
            .expect("Error reading /sys/bus/usb/devices")
            .filter(|entry| {
                // Ignore anything that errors during the read.
                let entry = match entry {
                    Ok(entry) => entry,
                    Err(e) => {
                        error!("Error reading entry in /sys/bus/usb/devices/: {}", e);
                        return false;
                    }
                };

                let metadata = entry
                    .metadata()
                    .expect("Error getting metadata for entry in /sys/bus/usb/devices");

                // Also ignore host controllers, named like "usb1", and aren't real devices that can be talked to.
                let is_device = entry.file_name().as_bytes()[0].is_ascii_digit();
                let is_hub = entry.file_name().as_bytes().contains(&b':');

                // FIXME: check if the symlink is to a directory.
                metadata.is_symlink() && is_device && !is_hub
            })
            .collect();

        let mut results: Vec<DeviceInformation> = Vec::with_capacity(sysfs_usb_devices.len());

        for dev in sysfs_usb_devices {
            let dev = dev.tap_err(|e| error!("Error traversing /sys/bus/usb/devices: {}", e))?;

            let full_path = dev.path();

            let devnum_path = full_path.join("devnum");
            let devnum_str = read_sysfs_attr(&devnum_path).expect(&format!(
                "Error reading USB devnum sysfs attr for device {}",
                full_path.display(),
            ));
            let devnum: u64 = devnum_str.parse().expect(&format!(
                "USB device number for device {} in sysfs is invalid: {}",
                full_path.display(),
                &devnum_str,
            ));

            let vid_path = full_path.join("idVendor");
            let vendor_id = read_sysfs_attr(&vid_path)
                .tap_err(|e| error!("Error reading sysfs attr {}: {}", vid_path.display(), &e,))
                .map(|vid_str| {
                    u16::from_str_radix(&vid_str, 16).expect(&format!(
                        "Device {} idVendor ({}) is not a number",
                        full_path.display(),
                        vid_str,
                    ))
                })?;

            let pid_path = full_path.join("idProduct");
            let product_id = read_sysfs_attr(&pid_path)
                .tap_err(|e| error!("Error reading sysfs attr {}: {}", pid_path.display(), &e,))
                .map(|pid_str| {
                    u16::from_str_radix(&pid_str, 16).expect(&format!(
                        "Device {} idProduct ({}) is not a number",
                        full_path.display(),
                        pid_str,
                    ))
                })?;

            let serial = read_sysfs_attr(&full_path.join("serial")).ok();
            let vendor = read_sysfs_attr(&full_path.join("manufacturer")).ok();
            let product = read_sysfs_attr(&full_path.join("product")).ok();

            let dev_info = DeviceInformation {
                vendor_id,
                product_id,
                serial,
                vendor,
                product,
                backend_numeric_location: Some(devnum),
                backend_string_location: Some(full_path.to_string_lossy().to_string()),
            };

            results.push(dev_info);
        }

        Ok(results)
    }

    fn open(&self, information: &DeviceInformation) -> UsbResult<Box<dyn BackendDevice>> {
        todo!()
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
        todo!()
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
        todo!()
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
        todo!()
    }

    fn write(
        &self,
        device: &Device,
        endpoint: u8,
        data: &[u8],
        timeout: Option<Duration>,
    ) -> UsbResult<()> {
        todo!()
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
