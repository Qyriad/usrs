//! Core, low-level functionality for Linux.

use std::{fs, os::unix::prelude::OsStrExt, time::Duration, path::Path};

use crate::{UsbResult, DeviceInformation, device::Device, Error};

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
                let entry = entry.as_ref().expect("Error reading entry in /sys/bus/usb/devices");
                let metadata = entry.metadata().expect("Error getting metadata for entry in /sys/bus/usb/devices");
                // As opposed to a host controller.
                let is_device = entry
                    .file_name()
                    .as_bytes()[0]
                    .is_ascii_digit();
                let is_hub = entry
                    .file_name()
                    .as_bytes()
                    .contains(&b':');

                //metadata.is_dir() && is_device && !is_hub
                metadata.is_symlink() && is_device && !is_hub
            })
            .collect();

        let mut results: Vec<DeviceInformation> = Vec::with_capacity(sysfs_usb_devices.len());

        for dev in sysfs_usb_devices {
            let dev = dev.expect("Error traversing /sys/bus/usb/devices");

            let full_path = dev.path();

            let devnum: u64 = fs::read_to_string(full_path.join("devnum"))
                .expect("Error reading USB device number from sysfs")
                .parse()
                .expect("USB device number in sysfs is invalid");

            let vendor_id = read_sysfs_attr(&full_path, "idVendor")
                .map(|vid_str| u16::from_str_radix(&vid_str, 16)
                    .expect(&format!("invalid attr idVendor for device {}", full_path.display()))
                )?;

            let product_id = read_sysfs_attr(&full_path, "idProduct")
                .map(|pid_str| u16::from_str_radix(&pid_str, 16)
                    .expect(&format!("invalid attr idProduct for device {}", full_path.display()))
                )?;

            let vendor = read_sysfs_attr(&full_path, "vendor")?;
            let product = read_sysfs_attr(&full_path, "product")?;

            let dev_info = DeviceInformation {
                vendor_id,
                product_id,
                serial: todo!(),
                vendor: todo!(),
                product: todo!(),
                backend_numeric_location: Some(devnum),
                backend_string_location: todo!(),
            };
        }

        todo!();
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

fn read_sysfs_attr<N, A>(sysfs_node: N, attr_name: A) -> UsbResult<String>
where
    N: AsRef<Path>,
    A: AsRef<Path>,
{
    let sysfs_node = sysfs_node.as_ref();
    let attr_name = attr_name.as_ref();

    // Attempt to read the file from the filesystem...
    let attr_result = fs::read_to_string(sysfs_node);
    let attr_value = match attr_result {
        Ok(v) => v,
        Err(io_error) => match io_error.raw_os_error() {
            // If it errored, and we have an error code, propagate that up.
            Some(errno) => return Err(Error::OsError(errno as i64)),

            // Otherwise, consider the error unspecified.
            None => return Err(Error::UnspecifiedOsError),
        },
    };

    Ok(attr_value)
}
