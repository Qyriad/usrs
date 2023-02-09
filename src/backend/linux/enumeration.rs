//! Contains functions for handling enumeration on Linux.

use std::{
    fs::{self, File},
    io::Read,
    os::unix::prelude::OsStrExt,
    path::Path,
};

use binrw::{io::Cursor, BinRead};
use log::{debug, error};
use tap::TapFallible;

use crate::{
    backend::linux::{pack_numeric_location, read_sysfs_attr},
    descriptor::DeviceDescriptor,
    DeviceInformation, UsbResult,
};

/// Enumerates devices via the Linux sysfs, usually at `/sys`.
///
/// This is considerably faster than [enumerate_with_devfs], but is less likely to be available
/// on some systems, either due to a permissions model or because the sysfs is not mounted at
/// all.
pub fn enumerate_with_sysfs<P: AsRef<Path>>(sysfs_root: P) -> UsbResult<Vec<DeviceInformation>> {
    let usb_devices_dir = sysfs_root.as_ref().join("bus/usb/devices");

    debug!(
        "Traversing {} to look for USB devices",
        usb_devices_dir.display()
    );

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

            // FIXME: determine if root hubs should be enumerated.
            let is_device = true;
            let is_interface = entry.file_name().as_bytes().contains(&b':');

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
            backend_numeric_location: Some(pack_numeric_location(busnum, devnum)),
            backend_string_location: Some(full_path.to_string_lossy().to_string()),
        };

        results.push(dev_info);
    }

    debug!("sysfs-based device enumeration successfully completed");

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
pub fn enumerate_with_devfs<P: AsRef<Path>>(devfs_root: P) -> UsbResult<Vec<DeviceInformation>> {
    let usb_devices_dir = devfs_root.as_ref().join("bus/usb");

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
            backend_numeric_location: Some(pack_numeric_location(busnum, devnum)),
            backend_string_location: None,
        };

        results.push(dev_info);
    }

    Ok(results)
}
