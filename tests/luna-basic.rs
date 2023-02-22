use std::iter;

use usrs::{request::DescriptorType, DeviceSelector};

const LUNA_COUNTER_DEVICE_DESCRIPTOR: &'static [u8] = &[
    18, // bLength
    1,  // bDescriptorType
    0x00, 0x02, // bcdUSB
    0,    // bDeviceClass
    0,    // bDeviceSubClass
    0,    // bDeviceProtocol
    64,   // bMaxPacketSize0
    0xd0, 0x16, // idVendor
    0x3b, 0x0f, // idProduct
    0x00, 0x00, // bcdDevice
    1,    // iManufacturer
    2,    // iProduct
    3,    // iSerial
    1,    // bNumConfigurations
];

#[test]
fn luna_descriptors() {
    env_logger::init();

    let luna_info = usrs::device(&DeviceSelector {
        vendor_id: Some(0x16d0),
        product_id: Some(0x0f3b),
        ..Default::default()
    })
    .unwrap();

    let mut luna = usrs::open(&luna_info).unwrap();

    let descriptor = luna
        .read_standard_descriptor(DescriptorType::Device, 0)
        .unwrap();

    assert_eq!(descriptor, LUNA_COUNTER_DEVICE_DESCRIPTOR);

    let should_be: Vec<u8> = iter::successors(Some(0u8), |val| Some(val.wrapping_add(1)))
        .take(512)
        .collect();
    let mut buffer = vec![0; 512];
    luna.read(0x81, &mut buffer, None).unwrap();
    assert_eq!(buffer, should_be);
}
