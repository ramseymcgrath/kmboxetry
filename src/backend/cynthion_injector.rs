//! USB injection backend for Cynthion.

use std::cmp::Ordering;
use std::time::Duration;

// Add bitfield crate import
use bitfield::bitfield;
use anyhow::{Context as ErrorContext, Error, bail};
use nusb::{
    self,
    transfer::{
        Control,
        ControlType,
        Recipient,
        EndpointType,
    },
    DeviceInfo,
    Interface
};

// Define the Speed enum since it's referenced but not imported
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Speed {
    Low = 0,
    Full = 1,
    High = 2,
    Auto = 3,
}

impl Speed {
    /// Returns a bitmask representing this speed for device capability queries
    pub fn mask(&self) -> u8 {
        1 << (*self as u8)
    }
}

// Add proper conversion implementations needed for bitfield
impl From<Speed> for u8 {
    fn from(speed: Speed) -> u8 {
        speed as u8
    }
}

impl From<u8> for Speed {
    fn from(value: u8) -> Self {
        match value {
            0 => Speed::Low,
            1 => Speed::Full,
            2 => Speed::High,
            _ => Speed::Auto,
        }
    }
}

// Define CaptureMetadata struct locally since the capture module doesn't exist
#[derive(Clone, Default)]
pub struct CaptureMetadata {
    pub iface_desc: Option<String>,
    pub iface_hardware: Option<String>,
    pub iface_os: Option<String>,
    pub iface_snaplen: Option<u32>,
}


// --- Constants ---
pub const VID_PID: (u16, u16) = (0x1d50, 0x615b);
const CLASS: u8 = 0xff;
const SUBCLASS: u8 = 0x10;
const PROTOCOL: u8 = 0x01;

// NOTE: Endpoint for INJECTION - THIS IS AN ASSUMPTION. Verify with Cynthion docs.
const INJECT_ENDPOINT: u8 = 0x01;
// NOTE: Endpoint for CAPTURE (kept for reference, but unused in injection logic)
const CAPTURE_ENDPOINT: u8 = 0x81;

// Timeout for USB operations
const DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);


// --- Bitfields for Control ---
// State: Used to enable/disable the interface and set speed
bitfield! {
    #[derive(Copy, Clone)]
    struct State(u8);
    bool, enable, set_enable: 0;
    u8, from into Speed, speed, set_speed: 2, 1;
}

impl State {
    fn new(enable: bool, speed: Speed) -> State {
        let mut state = State(0);
        state.set_enable(enable);
        state.set_speed(speed);
        state
    }
}

// TestConfig: Kept as it might be useful for controlling onboard test features
bitfield! {
    #[derive(Copy, Clone)]
    struct TestConfig(u8);
    bool, connect, set_connect: 0;
    u8, from into Speed, speed, set_speed: 2, 1;
}

impl TestConfig {
    fn new(speed: Option<Speed>) -> TestConfig {
        let mut config = TestConfig(0);
        match speed {
            Some(speed) => {
                config.set_connect(true);
                config.set_speed(speed);
            },
            None => {
                config.set_connect(false);
            }
        };
        config
    }
}


// --- Device Discovery and Handle ---

/// A Cynthion device attached to the system, potentially capable of injection.
pub struct CynthionDevice {
    device_info: DeviceInfo,
    interface_number: u8,
    alt_setting_number: u8,
    // Assuming speeds are relevant for injection too
    speeds: Vec<Speed>,
    // Metadata might still be useful
    metadata: CaptureMetadata,
}

/// A handle to an open Cynthion device, configured for injection.
#[derive(Clone)] // Cloning allows sharing the handle if needed
pub struct CynthionInjectorHandle {
    interface: Interface,
    metadata: CaptureMetadata,
    // Store the injection endpoint address
    inject_endpoint_addr: u8,
}


/// Probe for Cynthion devices suitable for injection.
pub fn probe(device_info: DeviceInfo) -> Result<CynthionDevice, Error> {
    // This largely reuses the original probing logic, checking class/subclass/protocol
    CynthionDevice::new(device_info)
}

impl CynthionDevice {
    /// Check whether a Cynthion device has the expected analyzer/injector interface.
    pub fn new(device_info: DeviceInfo) -> Result<CynthionDevice, Error> {

        let device = device_info
            .open()
            .context("Failed to open device")?;

        let config = device
            .active_configuration()
            .context("Failed to retrieve active configuration")?;

        for interface in config.interfaces() {
            let interface_number = interface.interface_number();

            for alt_setting in interface.alt_settings() {
                let alt_setting_number = alt_setting.alternate_setting();

                if alt_setting.class() != CLASS ||
                   alt_setting.subclass() != SUBCLASS
                {
                    continue;
                }

                let protocol = alt_setting.protocol();
                #[allow(clippy::absurd_extreme_comparisons)]
                match PROTOCOL.cmp(&protocol) {
                     Ordering::Less =>
                        bail!("Analyzer/Injector gateware is newer (v{}) than supported by this version (v{}). Please update this tool.", protocol, PROTOCOL),
                    Ordering::Greater =>
                        bail!("Analyzer/Injector gateware is older (v{}) than supported by this version (v{}). Please update gateware.", protocol, PROTOCOL),
                    Ordering::Equal => {}
                }

                // Check if the expected INJECT_ENDPOINT exists in this alt setting
                // This is a basic check, firmware might handle injection without a specific endpoint descriptor
                 let has_inject_endpoint = alt_setting.endpoints()
                    .any(|ep| ep.address() == INJECT_ENDPOINT && ep.transfer_type() == EndpointType::Bulk);

                 if !has_inject_endpoint {
                     // Log or potentially continue if firmware might implicitly handle it
                     // For now, let's be strict and require the endpoint unless we know otherwise.
                     eprintln!("Warning: Alt setting {} on interface {} does not explicitly list Bulk OUT endpoint {:#04x}. Injection might fail if gateware requires it.",
                        alt_setting_number, interface_number, INJECT_ENDPOINT);
                     // continue; // Uncomment this line if you want to strictly require the endpoint descriptor
                 }


                // Try to claim the interface (temporarily for probing speeds)
                let temp_interface = device
                    .claim_interface(interface_number)
                    .context("Failed to claim interface")?;

                if alt_setting_number != 0 {
                    temp_interface
                        .set_alt_setting(alt_setting_number)
                        .context("Failed to select alternate setting")?;
                }

                let metadata = CaptureMetadata {
                    // Adjusted descriptions slightly
                    iface_desc: Some("Cynthion USB Injector".to_string()),
                    iface_hardware: Some({
                        let bcd = device_info.device_version();
                        let major = bcd >> 8;
                        let minor = bcd as u8;
                        format!("Cynthion r{major}.{minor}")
                    }),
                    iface_os: Some(
                        format!("USB Analyzer/Injector v{protocol}")),
                    // snaplen is irrelevant for injection
                    iface_snaplen: None, // Was Some(NonZeroU32::new(0xFFFF).unwrap()),
                    .. Default::default()
                };

                // Fetch the available speeds (using a temporary handle)
                 // Note: Handle creation requires metadata owned, so clone it.
                 let temp_handle = CynthionInjectorHandle {
                     interface: temp_interface, // takes ownership temp_interface
                     metadata: metadata.clone(),
                     inject_endpoint_addr: INJECT_ENDPOINT, // Use the constant
                 };
                let speeds = temp_handle
                    .fetch_supported_speeds()
                    .context("Failed to fetch available speeds")?;

                // Release the interface after probing
                // The drop handler for CynthionInjectorHandle should release it.

                // Found a usable device configuration.
                return Ok(
                    CynthionDevice {
                        device_info,
                        interface_number,
                        alt_setting_number,
                        speeds,
                        metadata, // Use the original metadata
                    }
                )
            }
        }

        bail!("No supported analyzer/injector interface found");
    }

    /// Open this device and return a handle configured for injection.
    pub fn open_injector(&self) -> Result<CynthionInjectorHandle, Error> {
        let device = self.device_info.open()
            .context("Failed to open device for injection")?;
        let interface = device.claim_interface(self.interface_number)
             .context("Failed to claim interface for injection")?;
        if self.alt_setting_number != 0 {
            interface.set_alt_setting(self.alt_setting_number)
                .context("Failed to select alternate setting for injection")?;
        }
        Ok(CynthionInjectorHandle {
            interface,
            metadata: self.metadata.clone(),
            inject_endpoint_addr: INJECT_ENDPOINT, // Use the constant
        })
    }

     /// Get the list of supported speeds found during probing.
    pub fn supported_speeds(&self) -> &[Speed] {
        &self.speeds
    }

    /// Get the device metadata.
    pub fn metadata(&self) -> &CaptureMetadata {
        &self.metadata
    }
}


// --- Injector Handle Implementation ---

impl CynthionInjectorHandle {

    /// Fetch the speeds supported by the Cynthion device.
    /// Renamed from `speeds` to avoid conflict if called directly on handle later.
    fn fetch_supported_speeds(&self) -> Result<Vec<Speed>, Error> {
        use Speed::*;
        let control = Control {
            control_type: ControlType::Vendor,
            recipient: Recipient::Interface,
            request: 2, // Request code for querying speeds (Keep assumption)
            value: 0,
            index: self.interface.interface_number() as u16,
        };
        let mut buf = [0; 64];
        let size = self.interface
            .control_in_blocking(control, &mut buf, DEFAULT_TIMEOUT)
            .context("Failed retrieving supported speeds from device")?;
        if size != 1 {
            bail!("Expected 1-byte response to speed request, got {size}");
        }
        let mut speeds = Vec::new();
        // Now we have the Speed::mask() method implementation
        for speed in [Speed::Auto, Speed::High, Speed::Full, Speed::Low] {
             if buf[0] & speed.mask() != 0 {
                speeds.push(speed);
            }
        }
        Ok(speeds)
    }

    /// Prepares the Cynthion interface for injection at the specified speed.
    /// This likely involves enabling the interface via a control request.
    pub fn prepare_injection (&mut self, speed: Speed) -> Result<(), Error> {
        // Assumes request=1, value=State enables the interface block
        self.write_control_request(1, State::new(true, speed).0)
            .context("Failed to enable Cynthion interface for injection")
    }

    /// Stops injection and disables the Cynthion interface.
    pub fn stop_injection(&mut self) -> Result<(), Error> {
        // Assumes request=1, value=State disables the interface block
        // We use Speed::High as a default placeholder when disabling, matching original code.
        self.write_control_request(1, State::new(false, Speed::High).0)
             .context("Failed to disable Cynthion interface after injection")
    }

     /// Injects a packet/command onto the USB bus via the Cynthion.
    ///
    /// `data`: The raw bytes representing the packet or command expected by the Cynthion firmware.
    ///         The exact format depends on the Cynthion gateware/firmware implementation.
    pub fn inject_packet(&mut self, data: &[u8]) -> Result<(), Error> {
        match self.interface.bulk_out_blocking(
            self.inject_endpoint_addr, // Use configured OUT endpoint
            data,
            DEFAULT_TIMEOUT)
        {
            Ok(sent_len) if sent_len == data.len() => Ok(()),
            Ok(sent_len) => bail!(
                "Failed to send complete packet to injection endpoint: sent {} of {} bytes",
                sent_len, data.len()
            ),
            Err(e) => Err(e).context("USB bulk OUT transfer failed during injection"),
        }
    }

    /// Configures the onboard test device (if applicable).
    /// Kept from original code, functionality depends on firmware.
    pub fn configure_test_device(&mut self, speed: Option<Speed>)
        -> Result<(), Error>
    {
        let test_config = TestConfig::new(speed);
        // Assumes request=3 is for test device config
        self.write_control_request(3, test_config.0)
            .context("Failed to set test device configuration")
    }

    /// Helper function to send a vendor control request (OUT).
    fn write_control_request(&mut self, request: u8, value: u8) -> Result<(), Error> {
        let control = Control {
            control_type: ControlType::Vendor,
            recipient: Recipient::Interface,
            request,
            value: u16::from(value),
            index: self.interface.interface_number() as u16,
        };
        let data = &[]; // No data phase for these requests
        self.interface
            .control_out_blocking(control, data, DEFAULT_TIMEOUT)
            .map(|_| ()) // Discard bytes written count on success
            .context(format!("Vendor control write request failed (req={request}, val={value})"))
    }

     /// Get the device metadata associated with this handle.
    pub fn metadata(&self) -> &CaptureMetadata {
        &self.metadata
    }
}
