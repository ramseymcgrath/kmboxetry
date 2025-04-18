//! USB HID Mouse Injector using Cynthion.

use std::time::Duration;
use anyhow::{Context as ErrorContext, Error, bail};
use nusb::{
    self,
    transfer::{ Control, ControlType, Recipient, EndpointType },
    DeviceInfo,
    Interface,
};

// Assuming Speed enum is still needed for configuration via control requests.
// This might come from another module or be defined here.
// If not needed, remove related code.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum Speed {
    High = 0,
    Full = 1,
    Low = 2,
    Auto = 3,
}

impl From<u8> for Speed {
    fn from(val: u8) -> Speed {
        match val {
            0 => Speed::High,
            1 => Speed::Full,
            2 => Speed::Low,
            _ => Speed::Auto,
        }
    }
}

impl From<Speed> for u8 {
    fn from(speed: Speed) -> u8 {
        match speed {
            Speed::High => 0,
            Speed::Full => 1,
            Speed::Low => 2,
            Speed::Auto => 3,
        }
    }
}

// --- Constants ---
pub const VID_PID: (u16, u16) = (0x1d50, 0x615b); // Cynthion VID/PID
const CLASS: u8 = 0xff;       // Vendor-specific class
const SUBCLASS: u8 = 0x10;    // Defined by Cynthion firmware
const PROTOCOL: u8 = 0x01;    // Protocol version for the firmware feature

// --- ASSUMPTIONS - VERIFY THESE! ---
// Endpoint address for sending HID reports (Interrupt OUT assumed)
const HID_INJECT_ENDPOINT: u8 = 0x02;
// Control request codes (assuming reuse from capture firmware)
const REQ_ENABLE_INTERFACE: u8 = 1;
const REQ_GET_SUPPORTED_SPEEDS: u8 = 2;
const REQ_CONFIGURE_TEST_DEVICE: u8 = 3; // Example if needed
// --- End Assumptions ---

const DEFAULT_TIMEOUT: Duration = Duration::from_secs(1);

// Bitfield for Interface State Control (Enable/Speed) - Assuming reuse
bitfield! {
    #[derive(Copy, Clone)]
    struct InterfaceState(u8);
    bool, enable, set_enable: 0;
    u8, from into Speed, speed, set_speed: 2, 1; // Assumes Speed has from/into
}

impl InterfaceState {
    fn new(enable: bool, speed: Speed) -> InterfaceState {
        let mut state = InterfaceState(0);
        state.set_enable(enable);
        state.set_speed(speed);
        state
    }
}

// Simple Mouse HID Report Structure (Boot Protocol Compatible)
// Byte 0: Buttons (Bit 0: Left, Bit 1: Right, Bit 2: Middle)
// Byte 1: dX (Relative X movement, signed 8-bit)
// Byte 2: dY (Relative Y movement, signed 8-bit)
#[derive(Debug, Copy, Clone)]
#[repr(C)] // Ensure predictable layout
struct MouseReport {
    buttons: u8,
    dx: i8,
    dy: i8,
}

impl MouseReport {
    fn new(buttons: u8, dx: i8, dy: i8) -> Self {
        MouseReport { buttons, dx, dy }
    }

    // Convert to byte array for sending
    fn as_bytes(&self) -> [u8; 3] {
        [self.buttons, self.dx as u8, self.dy as u8]
    }
}

/// Represents a Cynthion device found on the system suitable for injection.
#[derive(Clone)]
pub struct InjectorDevice {
    pub device_info: DeviceInfo,
    interface_number: u8,
    alt_setting_number: u8,
    // We might still want to know supported speeds for configuration
    supported_speeds: Vec<Speed>,
}

/// A handle to an open Cynthion device configured for HID injection.
pub struct HidInjector {
    interface: Interface,
    hid_endpoint_addr: u8,
    // Keep track of the speed it was configured with, if relevant
    current_speed: Option<Speed>,
}

/// Probe for Cynthion devices suitable for HID injection.
pub fn probe_devices() -> Result<Vec<InjectorDevice>, Error> {
    let mut suitable_devices = Vec::new();

    for device_info in nusb::list_devices()? {
        if device_info.vendor_id() == VID_PID.0 && device_info.product_id() == VID_PID.1 {
            match probe_single_device(device_info) {
                Ok(injector_device) => suitable_devices.push(injector_device),
                Err(e) => {
                    // Log probing errors for specific devices but continue searching
                    eprintln!("Note: Found Cynthion VID/PID but failed to probe suitability: {}", e);
                }
            }
        }
    }

    if suitable_devices.is_empty() {
        bail!("No suitable Cynthion injector device found.");
    }
    Ok(suitable_devices)
}

/// Check a single DeviceInfo for the required injection interface.
fn probe_single_device(device_info: DeviceInfo) -> Result<InjectorDevice, Error> {
     let device = device_info.open().context("Failed to open device")?;
    let config = device.active_configuration()
                       .context("Failed to retrieve active configuration")?;

    for interface in config.interfaces() {
        let interface_number = interface.interface_number();
        for alt_setting in interface.alt_settings() {
            let alt_setting_number = alt_setting.alternate_setting();

            // Check if class/subclass/protocol match our expected interface
            if alt_setting.class() != CLASS || alt_setting.subclass() != SUBCLASS {
                continue;
            }

            // Check protocol version compatibility
            let protocol = alt_setting.protocol();
            if protocol != PROTOCOL {
                 bail!(
                    "Unsupported firmware protocol version (found v{}, expected v{}). Please update firmware or injector tool.",
                     protocol, PROTOCOL);
            }

            // Basic check for the assumed HID endpoint (optional, firmware might handle implicitly)
            let has_hid_endpoint = alt_setting.endpoints()
                .any(|ep| ep.address() == HID_INJECT_ENDPOINT && ep.transfer_type() == EndpointType::Interrupt); // Assumes Interrupt OUT

            if !has_hid_endpoint {
                eprintln!(
                    "Warning: Alt setting {} on interface {} does not explicitly list Interrupt OUT endpoint {:#04x}. Injection might fail if firmware requires it.",
                     alt_setting_number, interface_number, HID_INJECT_ENDPOINT);
                // Decide whether to continue or bail based on firmware knowledge
                // continue; // Uncomment to strictly require the endpoint descriptor
            }

            // Temporarily claim to query speeds
            let temp_interface = device.claim_interface(interface_number)
                                    .context("Failed to claim interface")?;
            if alt_setting_number != 0 {
                 temp_interface.set_alt_setting(alt_setting_number)
                            .context("Failed to select alternate setting")?;
            }

             // Use a temporary handle structure for querying speeds
             struct TempHandle { interface: Interface }
             impl TempHandle {
                 fn fetch_supported_speeds(&self) -> Result<Vec<Speed>, Error> {
                    fetch_speeds_from_device(&self.interface)
                }
             }
             let temp_handle = TempHandle { interface: temp_interface };
            let speeds = temp_handle.fetch_supported_speeds()?;
            // temp_handle (and thus temp_interface) is dropped here, releasing the interface


            // Found a potential candidate
            return Ok(InjectorDevice {
                device_info,
                interface_number,
                alt_setting_number,
                supported_speeds: speeds,
            });
        }
    }

    bail!("No suitable injection interface found on device.")
}


impl InjectorDevice {
    /// Open the device and return a handle ready for injection commands.
    pub fn open(&self) -> Result<HidInjector, Error> {
        let device = self.device_info.open()
            .context("Failed to open device for injection")?;
        let interface = device.claim_interface(self.interface_number)
            .context("Failed to claim interface for injection")?;
        if self.alt_setting_number != 0 {
            interface.set_alt_setting(self.alt_setting_number)
                .context("Failed to select alternate setting for injection")?;
        }

        Ok(HidInjector {
            interface,
            hid_endpoint_addr: HID_INJECT_ENDPOINT,
            current_speed: None, // Not configured yet
        })
    }

    /// Get the list of supported speeds detected during probing.
    pub fn supported_speeds(&self) -> &[Speed] {
        &self.supported_speeds
    }
}


impl HidInjector {
    /// Prepare the Cynthion interface for injection at the specified speed.
    /// This sends a control command to enable the hardware.
    pub fn prepare_injection(&mut self, speed: Speed) -> Result<(), Error> {
        // Assume REQ_ENABLE_INTERFACE controls the hardware block
        let state = InterfaceState::new(true, speed);
        self.write_control_request(REQ_ENABLE_INTERFACE, state.0)
            .context(format!("Failed to enable Cynthion interface for injection at speed {:?}", speed))?;
        self.current_speed = Some(speed);
        Ok(())
    }

    /// Stop injection and disable the Cynthion interface.
    pub fn stop_injection(&mut self) -> Result<(), Error> {
        // Send disable command. Use a default speed (e.g., High) as placeholder if needed.
        let speed = self.current_speed.unwrap_or(Speed::High);
        let state = InterfaceState::new(false, speed);
        self.write_control_request(REQ_ENABLE_INTERFACE, state.0)
            .context("Failed to disable Cynthion interface after injection")?;
        self.current_speed = None;
        Ok(())
    }

    /// Send a mouse movement/button report.
    ///
    /// # Arguments
    /// * `buttons` - Button state bitmask (0x01 = Left, 0x02 = Right, 0x04 = Middle).
    /// * `dx` - Relative movement along the X axis (-127 to 127).
    /// * `dy` - Relative movement along the Y axis (-127 to 127).
    pub fn send_mouse_report(&mut self, buttons: u8, dx: i8, dy: i8) -> Result<(), Error> {
        if self.current_speed.is_none() {
            bail!("Injection not prepared. Call prepare_injection() first.");
        }

        let report = MouseReport::new(buttons, dx, dy);
        let report_bytes = report.as_bytes();

        // Use interrupt_out based on the available API in nusb 0.1.13
        // The API has changed from blocking to async
        use futures_lite::future::block_on;
        
        let completion = block_on(self.interface.interrupt_out(
            self.hid_endpoint_addr,
            report_bytes.to_vec()));
            
        match completion.status {
            Ok(()) => Ok(()),
            Err(e) => Err(e).context("USB Interrupt OUT transfer failed sending HID report"),
        }

        // If the HID endpoint was Bulk OUT, you would use this instead:
        /*
        match self.interface.bulk_out_blocking(
            self.hid_endpoint_addr,
            &report_bytes,
            DEFAULT_TIMEOUT)
        {
            Ok(sent_len) if sent_len == report_bytes.len() => Ok(()),
            Ok(sent_len) => bail!(... ),
            Err(e) => Err(e).context("USB Bulk OUT transfer failed sending HID report"),
        }
        */
    }

     /// Helper to send a simple vendor control request (OUT, no data).
    fn write_control_request(&mut self, request: u8, value: u8) -> Result<(), Error> {
        let control = Control {
            control_type: ControlType::Vendor,
            recipient: Recipient::Interface,
            request,
            value: u16::from(value),
            index: self.interface.interface_number() as u16,
        };
        let data = &[]; // No data phase
        self.interface
            .control_out_blocking(control, data, DEFAULT_TIMEOUT)
            .map(|_| ()) // Discard bytes written count on success
            .context(format!("Vendor control OUT request failed (req={request}, val={value:#04x})"))
    }

    // Maybe add a function to query speeds again if needed on an open handle
    // pub fn fetch_supported_speeds(&self) -> Result<Vec<Speed>, Error> {
    //    fetch_speeds_from_device(&self.interface)
    // }
}

/// Helper function to query supported speeds from the device interface.
 fn fetch_speeds_from_device(interface: &Interface) -> Result<Vec<Speed>, Error> {
    use Speed::*;
    let control = Control {
        control_type: ControlType::Vendor,
        recipient: Recipient::Interface,
        request: REQ_GET_SUPPORTED_SPEEDS, // Use assumed request code
        value: 0,
        index: interface.interface_number() as u16,
    };
    let mut buf = [0; 1]; // Expect 1 byte based on previous code
    let timeout = Duration::from_secs(1);
    let size = interface
        .control_in_blocking(control, &mut buf, timeout)
        .context("Failed retrieving supported speeds from device")?;
    if size != 1 {
        bail!("Expected 1-byte response to speed query request, got {size}");
    }
    let mut speeds = Vec::new();
    let speed_mask = buf[0];
    // Check mask against known Speed values/masks
    if speed_mask & Speed::Auto.mask() != 0 { speeds.push(Auto); }
    if speed_mask & Speed::Low.mask()  != 0 { speeds.push(Low); }
    if speed_mask & Speed::Full.mask() != 0 { speeds.push(Full); }
    if speed_mask & Speed::High.mask() != 0 { speeds.push(High); }

    Ok(speeds)
}


// --- Speed Enum Helpers ---
// Provide the methods needed by the InterfaceState bitfield and speed query logic
// Ensure these align with the actual values used by the Cynthion firmware if 'Auto' is used.
impl Speed {
    // Required by fetching/checking supported speeds
    pub fn mask(&self) -> u8 {
        match self {
            Speed::High => 0b1000,
            Speed::Full => 0b0100,
            Speed::Low  => 0b0010,
            Speed::Auto => 0b0001, // Mask seems consistent with original code
        }
    }
}

// --- Drop Implementation ---
// Ensure the interface is disabled when the injector goes out of scope
impl Drop for HidInjector {
    fn drop(&mut self) {
        if self.current_speed.is_some() {
            if let Err(e) = self.stop_injection() {
                eprintln!("Error while auto-stopping injection on drop: {}", e);
            }
        }
        // nusb::Interface handles releasing the USB interface itself on drop
    }
}


// --- Example Usage ---
/*
fn main() -> anyhow::Result<()> {
    println!("Searching for Cynthion injector device...");
    let devices = probe_devices()?;

    // For simplicity, use the first device found
    let device = &devices[0];
    println!("Found device. Speeds: {:?}", device.supported_speeds());

    println!("Opening injector handle...");
    let mut injector = device.open()?;

    // Choose a speed (e.g., Full speed for HID)
    // Check if Full speed is supported before using it
    let target_speed = Speed::Full;
    if !device.supported_speeds().contains(&target_speed) {
        bail!("Device does not support the desired {:?} speed.", target_speed);
    }


    println!("Preparing injection at {:?} speed...", target_speed);
    injector.prepare_injection(target_speed)?;
    println!("Injection ready.");

    // --- Send some mouse movements ---
    println!("Moving mouse right...");
    injector.send_mouse_report(0, 50, 0)?; // Buttons=0, dx=50, dy=0
    std::thread::sleep(Duration::from_millis(50));

    println!("Moving mouse down...");
    injector.send_mouse_report(0, 0, 50)?; // Buttons=0, dx=0, dy=50
    std::thread::sleep(Duration::from_millis(50));

    println!("Moving mouse diagonally (up-left)...");
    injector.send_mouse_report(0, -30, -30)?; // Buttons=0, dx=-30, dy=-30
    std::thread::sleep(Duration::from_millis(50));

     println!("Simulating left click (press)...");
    injector.send_mouse_report(0x01, 0, 0)?; // Buttons=Left, dx=0, dy=0
    std::thread::sleep(Duration::from_millis(100)); // Hold duration

    println!("Simulating left click (release)...");
    injector.send_mouse_report(0x00, 0, 0)?; // Buttons=None, dx=0, dy=0
    std::thread::sleep(Duration::from_millis(50));


    println!("Stopping injection...");
    // stop_injection() called automatically by Drop, or call explicitly:
    // injector.stop_injection()?;

    println!("Done.");
    Ok(())
}
*/
