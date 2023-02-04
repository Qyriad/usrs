use std::{any::Any, fs::File};

use super::BackendDevice;

/// Internal type storing the state for a raw USB device on Linux.
#[derive(Debug)]
pub(crate) struct LinuxDevice {
    /// The "everything is a file" handle to a USB device on Linux.
    pub(crate) file: File,
}

impl BackendDevice for LinuxDevice {
    fn as_mut_any(&mut self) -> &mut dyn Any {
        self
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}
