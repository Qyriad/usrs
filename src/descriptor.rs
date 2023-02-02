//! Types for working with and parsing USB descriptors.

use binrw::binrw;

// TODO: should this be Copy?
#[derive(Copy, Debug, Clone, PartialEq, Hash)]
#[allow(non_snake_case)]
#[binrw]
// USB is little-endian [USB 2.0§8.1]
#[brw(little)]
pub struct DeviceDescriptor {
    pub bLength: u8,
    pub bDescriptorType: u8,
    pub bcdUSB: u16,
    pub bDeviceClass: u8,
    pub bDeviceSubClass: u8,
    pub bDeviceProtocol: u8,
    pub bMaxPacketSize: u8,
    pub idVendor: u16,
    pub idProduct: u16,
    pub bcdDevice: u16,
    pub iManufacturer: u8,
    pub iProduct: u8,
    pub iSerialNumber: u8,
    pub bNumConfigurations: u8,
}
