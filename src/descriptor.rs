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

// TODO: should this be Copy?
#[derive(Copy, Debug, Clone, PartialEq, Hash)]
#[allow(non_snake_case)]
#[binrw]
#[brw(little)]
pub struct ConfigurationDescriptor {
    pub bLength: u8,
    /// Constant: 2.
    pub bDescriptorType: u8,
    pub wTotalLength: u16,
    pub bNumInterfaces: u8,
    pub bConfigurationValue: u8,
    pub iConfiguration: u8,
    pub bmAttributes: u8,
    pub bMaxPower: u8,
}

// TODO: should this be Copy?
#[derive(Copy, Debug, Clone, PartialEq, Hash)]
#[allow(non_snake_case)]
#[binrw]
#[brw(little)]
pub struct InterfaceDescriptor {
    pub bLength: u8,
    /// Constant: 4
    pub bDescriptorType: u8,
    pub bInterfaceNumber: u8,
    pub bAlternateSetting: u8,
    pub bNumEndpoints: u8,
    pub bInterfaceClass: u8,
    pub bInterfaceSubClass: u8,
    pub bInterfaceProtocol: u8,
    pub iInterface: u8,
}

// TODO: should this be Copy?
#[derive(Copy, Debug, Clone, PartialEq, Hash)]
#[allow(non_snake_case)]
#[binrw]
#[brw(little)]
pub struct EndpointDescriptor {
    pub bLength: u8,
    /// Constant: 5.
    pub bDescriptorType: u8,
    pub bEndpointAddress: u8,
    pub bmAttributes: u8,
    pub wMaxPacketSize: u16,
    pub bInterval: u8,
}
