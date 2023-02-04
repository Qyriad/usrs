//! Core, low-level functionality for Linux.

use std::{
    ffi::c_void,
    fs::{self, File},
    io::ErrorKind as IoErrorKind,
    io::{Error as IoError, Read},
    iter,
    os::unix::prelude::{AsRawFd, OsStrExt},
    path::{Path, PathBuf},
    ptr,
    time::Duration,
};

use binrw::{io::Cursor, BinRead};
use log::{error, warn};
use nix::{
    poll::{PollFd, PollFlags},
    sys::select::FdSet,
};
use tap::tap::TapFallible;

use crate::{
    backend::linux::{
        device::LinuxDevice,
        ioctl_c::{__IncompleteArrayField, usbdevfs_urb, USBDEVFS_URB_TYPE_CONTROL},
    },
    descriptor::{self, DeviceDescriptor},
    device::Device,
    DeviceInformation, Error, UsbResult,
};

use super::{Backend, BackendDevice};

mod device;
mod ioctl;
mod ioctl_c;

/// Per-OS data for the Linux backend.
#[derive(Debug)]
pub struct LinuxBackend {
    sysfs_root: PathBuf,
    devfs_root: PathBuf,
}

impl LinuxBackend {
    const DEFAULT_SYSFS_ROOT: &'static str = "/sys";
    const DEFAULT_DEVFS_ROOT: &'static str = "/dev";

    /// Initializes a Linux backend using sensible defaults. Use [with_paths] if you need more
    /// granularity.
    pub fn new() -> UsbResult<Self> {
        Ok(LinuxBackend {
            sysfs_root: PathBuf::from(Self::DEFAULT_SYSFS_ROOT),
            devfs_root: PathBuf::from(Self::DEFAULT_DEVFS_ROOT),
        })
    }

    /// Initialize a Linux backend and override the sysfs and devfs root paths (/sys and /dev
    /// by default).
    // FIXME: This constructor is not accessible currently as the entire module is not.
    pub fn with_paths(sysfs_root: Option<PathBuf>, devfs_root: Option<PathBuf>) -> UsbResult<Self> {
        Ok(LinuxBackend {
            sysfs_root: sysfs_root.unwrap_or_else(|| PathBuf::from(Self::DEFAULT_SYSFS_ROOT)),
            devfs_root: devfs_root.unwrap_or_else(|| PathBuf::from(Self::DEFAULT_DEVFS_ROOT)),
        })
    }

    /// Packs Linux bus number and device number into a single u64, for storing into a
    /// [DeviceInformation] struct.
    fn pack_numeric_location(busnum: u32, devnum: u32) -> u64 {
        ((busnum as u64) << 32) | (devnum as u64)
    }

    /// Unpacks the `backend_numeric_location` stored in a [DeviceInformation] struct into its
    /// Linux bus number and device number.
    fn unpack_numeric_Location(numeric_location: u64) -> (u32, u32) {
        let busnum = (numeric_location >> 32) as u32;
        let devnum = (numeric_location & 0x00000000_FFFFFFFF) as u32;

        (busnum, devnum)
    }
}

impl LinuxBackend {
    /// Enumerates devices via the Linux sysfs, usually at `/sys`.
    ///
    /// This is considerably faster than [enumerate_with_devfs], but is less likely to be available
    /// on some systems, either due to a permissions model or because the sysfs is not mounted at
    /// all.
    fn enumerate_with_sysfs(&self) -> UsbResult<Vec<DeviceInformation>> {
        let usb_devices_dir = self.sysfs_root.join("bus/usb/devices");

        let sysfs_usb_devices: Vec<_> = fs::read_dir(&usb_devices_dir)
            .tap_err(|e| error!("Error traversing {}: {}", usb_devices_dir.display(), &e))?
            .filter(|entry| {
                // Ignore anything that errors during the read.
                let entry = match entry {
                    Ok(entry) => entry,
                    Err(e) => {
                        error!(
                            "Error reading entry in {}: {}; skipping device",
                            usb_devices_dir.display(),
                            e
                        );
                        return false;
                    }
                };

                // Also ignore anything that we can't get the metadata for.
                let metadata = match entry.metadata() {
                    Ok(metadata) => metadata,
                    Err(e) => {
                        error!(
                            "Error getting metadata for {}: {}; skipping device",
                            entry.path().display(),
                            e
                        );
                        return false;
                    }
                };

                //// Ignore host controllers, named like "usb1", which aren't real devices that can be talked to.
                //// Devices are named things like `3-5`, for "bus 3, port 5".
                //// Host controllers are named things like `usb3`, for "bus 3".
                //// Individual endpoints also have nodes in /sys/bus/usb/devices, which are named
                //// like `3-5:1.0`, for "bus 3, port 5, configuration 1, interface 1` (interfaces
                //// are 1-indexed in USB but 0-indexed in Linux).
                //let is_device = entry.file_name().as_bytes()[0].is_ascii_digit();
                // FIXME: determine if root hubs should be enumerated.
                let is_device = true;
                let is_interface = entry.file_name().as_bytes().contains(&b'.');

                // FIXME: check if the symlink is to a directory.
                metadata.is_symlink() && is_device && !is_interface
            })
            .collect();

        let mut results: Vec<DeviceInformation> = Vec::with_capacity(sysfs_usb_devices.len());

        for dev in sysfs_usb_devices {
            let dev =
                dev.tap_err(|e| error!("Error traversing {}: {}", usb_devices_dir.display(), e))?;

            let full_path = dev.path();

            //
            // Read `/sys/bus/usb/devices/<device>/{devnum, busnum}`, which is needed to open the device in
            // `/dev/bus/usb/<bus>/<devnum>`.
            //

            let devnum_path = full_path.join("devnum");
            let devnum: u32 = read_sysfs_attr(&devnum_path)
                .tap_err(|e| error!("Error reading sysfs attr {}: {}", devnum_path.display(), &e))
                .map(|devnum_str| {
                    devnum_str.parse().expect(&format!(
                        "Device {} devnum ({}) is not a number",
                        full_path.display(),
                        devnum_str
                    ))
                })?;

            let busnum_path = full_path.join("busnum");
            let busnum: u32 = read_sysfs_attr(&busnum_path)
                .tap_err(|e| error!("Error reading sysfs attr {}: {}", busnum_path.display(), &e))
                .map(|busnum_str| {
                    busnum_str.parse().expect(&format!(
                        "Device {} busnum({}) is not a number",
                        full_path.display(),
                        busnum_str
                    ))
                })?;

            //
            // Now read other attributes that need to go in our DeviceInformation struct.
            //

            let vid_path = full_path.join("idVendor");
            let vendor_id = read_sysfs_attr(&vid_path)
                .tap_err(|e| error!("Error reading sysfs attr {}: {}", vid_path.display(), &e))
                .map(|vid_str| {
                    u16::from_str_radix(&vid_str, 16).expect(&format!(
                        "Device {} idVendor ({}) is not a number",
                        full_path.display(),
                        vid_str,
                    ))
                })?;

            let pid_path = full_path.join("idProduct");
            let product_id = read_sysfs_attr(&pid_path)
                .tap_err(|e| error!("Error reading sysfs attr {}: {}", pid_path.display(), &e))
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
                backend_numeric_location: Some(Self::pack_numeric_location(busnum, devnum)),
                backend_string_location: Some(full_path.to_string_lossy().to_string()),
            };

            results.push(dev_info);
        }

        Ok(results)
    }

    /// Enumerates devices via the Linux devfs/usbfs, usually at /dev, expecting the usbfs at
    /// /dev/bus/usb.
    ///
    /// This is considerably slower than [enumerate_with_syfs], but is more likely to be available
    /// on some systems, either due to a permissions model or because the sysfs is not mounted at
    /// all.
    /// The devfs may also not be mounted or accessible, on some systems, but if that's the case
    /// then we can't perform any USB IO anyway.
    fn enumerate_with_devfs(&self) -> UsbResult<Vec<DeviceInformation>> {
        let usb_devices_dir = self.devfs_root.join("bus/usb");

        let devfs_usb_busses: Vec<Result<fs::DirEntry, _>> = fs::read_dir(&usb_devices_dir)
            .tap_err(|e| error!("Error traversing /dev/bus/usb/: {}", &e))?
            .collect();

        let mut devfs_usb_nodes: Vec<(u32, u32)> = vec![];

        for bus_entry in devfs_usb_busses {
            // Bus entries should be a directory of the form `/dev/bus/usb/{busnum:03}`.
            // `busnum` is the index (1-indexed) of the bus as a 0-padded 3-digit number.
            let bus_entry =
                bus_entry.tap_err(|e| error!("Error getting USB devfs bus entry: {}", &e))?;

            let devices_in_bus = fs::read_dir(&bus_entry.path())?;

            for dev_entry in devices_in_bus {
                let dev_entry = dev_entry.tap_err(|e| {
                    error!(
                        "Error getting USB devfs dev entry in bus {}: {}",
                        &bus_entry.path().display(),
                        &e,
                    )
                })?;

                // Device entries should be files in the bus directory, of the form
                // `/dev/bus/usb/{busnum:03}/{devnum:03}`. The device number is *not* the port
                // number, which is used for the sysfs path, and iirc(Qyriad) is also not
                // necessarily the same as the device address assigned with SET_ADDRESS.

                // Thus, on a Linux system, all of these should hold true. And if they don't,
                // then we *definitely* can't access USB anyway.
                let busnum: u32 = bus_entry
                    .path()
                    .file_name()
                    .expect(&format!(
                        "devfs bus dir {} should have a name",
                        bus_entry.path().display()
                    ))
                    .to_str()
                    .expect(&format!(
                        "devfs bus dir name {} should be valid UTF-8",
                        bus_entry.path().display()
                    ))
                    .parse()
                    .expect(&format!(
                        "devfs bus dir {} should be a valid number",
                        bus_entry.path().display()
                    ));

                // And again all of these should hold true, and if they don't, we definitely can't
                // access USB.
                let devnum: u32 = dev_entry
                    .path()
                    .file_name()
                    .expect(&format!(
                        "devfs node {} should have a name",
                        dev_entry.path().display()
                    ))
                    .to_str()
                    .expect(&format!(
                        "devfs node name {} should be valid UTF-8",
                        dev_entry.path().display()
                    ))
                    .parse()
                    .expect(&format!(
                        "devfs node {} should be a valid number",
                        dev_entry.path().display()
                    ));

                devfs_usb_nodes.push((busnum, devnum));
            }
        }

        // Now that we know how many devices there are, iterate again with our known length.
        let mut results: Vec<DeviceInformation> = Vec::with_capacity(devfs_usb_nodes.len());

        for (busnum, devnum) in devfs_usb_nodes {
            // Now that we have a path to the USB device node, open the device in real-only mode
            // (no IOCTLs, so no general requests), and read the device file, which contains
            // the device's USB device descriptor binary (as well as the configuration descriptor
            // chain, but we only care about the device descriptor here.
            let usb_file_name = format!("/dev/bus/usb/{:03}/{:03}", busnum, devnum);

            let mut usb_file = File::options()
                .read(true)
                .open(&usb_file_name)
                .tap_err(|e| error!("Error opening USB dev node {}: {}", &usb_file_name, &e))?;

            let mut descriptor_chain: Vec<u8> = vec![];
            usb_file.read_to_end(&mut descriptor_chain).tap_err(|e| {
                error!(
                    "Error reading descriptors from USB device {}: {}",
                    &usb_file_name, &e,
                )
            })?;

            // Now parse the binary data into a [DeviceDescriptor] struct with the help of the
            // [binrw](https://doc.rs/binrw) crate.
            let mut descriptor_reader = Cursor::new(&descriptor_chain);
            let dev_desc = DeviceDescriptor::read(&mut descriptor_reader)
                .tap_err(|e| {
                    error!(
                        "Error reading descriptor for device {}: {}",
                        &usb_file_name, &e,
                    )
                })
                .expect(&format!(
                    "Device {} has an invalid device descriptor!",
                    &usb_file_name
                ));

            // And finally populate the DeviceInformation struct with the information we've
            // learned.
            // Unlike sysfs enumeration, we don't get string descriptors without making a
            // control request.
            let dev_info = DeviceInformation {
                vendor_id: dev_desc.idVendor,
                product_id: dev_desc.idProduct,
                serial: None,
                vendor: None,
                product: None,
                backend_numeric_location: Some(Self::pack_numeric_location(busnum, devnum)),
                backend_string_location: None,
            };

            results.push(dev_info);
        }

        Ok(results)
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
            match self.enumerate_with_sysfs() {
                Ok(results) => return Ok(results),
                Err(e) => {
                    warn!("Error enumerating via sysfs: {}; trying devfs instead", e);
                    return self.enumerate_with_devfs();
                }
            }
        }

        return self.enumerate_with_devfs();
    }

    fn open(&self, information: &DeviceInformation) -> UsbResult<Box<dyn BackendDevice>> {
        let (busnum, devnum) = Self::unpack_numeric_Location(
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
        let backend_device: &LinuxDevice =
            unsafe { device.backend_data().as_any().downcast_ref().unwrap() };

        let dev_fd = backend_device.file.as_raw_fd();

        // Unfortunately, the synchronous USBDEVFS_CONTROL ioctl completely disallows
        // control transfers where wLength > PAGE_SIZE
        // (https://elixir.bootlin.com/linux/v6.1/source/drivers/usb/core/devio.c#L1173).
        // Some HCD drivers *also* have similar limitations on urb submission,
        // but the increasingly common XHCD HCD does not.
        // Either way, we have to implement this synchronous API in terms of Linux's
        // asynchronous API in order to bypass this limitation.

        // Unfortunately, this also introduces an additional limitation.
        // USBDEVFS_SUBMITURB with USBDEVFS_URB_TYPE_CONTROL requires the setup data
        // to be the first 8 bytes of the buffer.
        // This means we can't just use the buffer provided by the user, so unfortunately
        // we'll have to reallocate to account for the extra 8 bytes of setup data.
        let mut buffer_for_urb = vec![0; target.len() + 8];

        let req_len = &(target.len() as u16).to_le_bytes();

        // For a control transfer, we need to fill the buffer with the setup data first.
        buffer_for_urb[0] = request_type; // bmRequestType.
        buffer_for_urb[1] = request_number; // bRequest.
        buffer_for_urb[2..=3].copy_from_slice(&value.to_le_bytes()); // wValue.
        buffer_for_urb[4..=5].copy_from_slice(&index.to_le_bytes()); // wIndex.
        buffer_for_urb[6..=7].copy_from_slice(req_len);

        // Now build the USB request block used for usbfs IOCTLs...
        let mut urb_for_transfer = usbdevfs_urb {
            type_: USBDEVFS_URB_TYPE_CONTROL as u8, //| 0x80,
            endpoint: 0,
            status: 0, // Out.
            flags: 0,
            buffer: buffer_for_urb.as_mut_ptr() as *mut c_void,
            buffer_length: buffer_for_urb.len() as i32, // FIXME: explicitly error if truncated.
            actual_length: 0,                           // OUT
            start_frame: 0,                             // OUT
            // Unnamed struct fields are not yet implemented in Rust:
            // https://github.com/rust-lang/rust/issues/49804.
            __bindgen_anon_1: ioctl_c::usbdevfs_urb__bindgen_ty_1 { stream_id: 0 }, // OUT.
            error_count: 0,                                                         // OUT.
            signr: 0, // No idea what this is.
            usercontext: ptr::null_mut(),
            iso_frame_desc: __IncompleteArrayField::new(),
        };

        // ...submit the URB to the kernel...
        let submit_urb_res = unsafe { ioctl::usbdevfs_submiturb(dev_fd, &mut urb_for_transfer) };
        match submit_urb_res {
            Ok(_) => (),
            Err(nix::Error::ENOENT) => return Err(Error::DeviceNotFound),
            Err(nix::Error::EACCES) => return Err(Error::PermissionDenied),
            Err(code) => return Err(Error::OsError(code as i64)),
        };

        // And wait for completion.
        let mut _out: *mut c_void = ptr::null_mut();
        let reapurb_res = unsafe { ioctl::usbdevfs_reapurb(dev_fd, &mut _out as *mut *mut c_void) };
        match reapurb_res {
            Ok(_) => (),
            Err(nix::Error::ENOENT) => return Err(Error::DeviceNotFound),
            Err(nix::Error::EACCES) => return Err(Error::PermissionDenied),
            Err(code) => return Err(Error::OsError(code as i64)),
        };

        if urb_for_transfer.status < 0 {
            match nix::Error::from_i32(-urb_for_transfer.status) {
                nix::Error::EPIPE => return Err(Error::Stalled),
                _ => return Err(Error::OsError(-urb_for_transfer.status as i64)),
            }
        }

        // Now that the URB is completed and without errors, copy the data into the user's buffer.
        target.copy_from_slice(&buffer_for_urb[8..]);

        Ok(urb_for_transfer.actual_length as usize)
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
