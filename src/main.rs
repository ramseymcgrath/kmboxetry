// Remove GUI subsystem directive
// #![windows_subsystem = "windows"]

// Import crates
use bitfield;

// Declare necessary modules
mod inject; // Contains HidInjector, probe_devices, Speed, etc.
mod util; // Utility functions and types
mod version; // Keep for version command

use anyhow::{bail, Context, Error};
use clap::Parser;
use std::io::{BufRead, BufReader};
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

// Import necessary items from our injector module
use crate::inject::{probe_devices, HidInjector, InjectorDevice, Speed};
use crate::version::{version, version_info}; // Optional

const UDP_READ_TIMEOUT_MS: u64 = 100; // Prevent blocking forever on UDP socket
const SERIAL_READ_TIMEOUT_MS: u64 = 100; // Prevent blocking forever on Serial read
const MAX_UDP_PACKET_SIZE: usize = 1024; // Max size for incoming UDP commands

/// Command-line arguments structure
#[derive(Parser, Debug)]
#[command(author, version = version(), about = "Cynthion HID Injector Control Server", long_about = None)]
struct Args {
    /// Run in UDP server mode, listening on <IP:PORT> (e.g., 127.0.0.1:9001)
    #[arg(long, value_name = "IP:PORT", conflicts_with = "serial_port")]
    udp: Option<String>,

    /// Run in Serial server mode, using the specified port (e.g., COM3 or /dev/ttyACM0)
    #[arg(long = "serial", value_name = "PORT", conflicts_with = "udp")]
    serial_port: Option<String>,

    /// Baud rate for serial communication
    #[arg(long, value_name = "RATE", default_value = "115200")]
    baud: u32,

    /// USB speed for injection (low, full, high)
    #[arg(long, value_name = "SPEED", default_value = "full")]
    speed: String,

    /// List available Cynthion devices and serial ports and exit
    #[arg(long)]
    list: bool,

    /// Index of the Cynthion device to use (if multiple are found, default 0)
    #[arg(long, value_name = "INDEX", default_value = "0")]
    device_index: usize,

    /// Print dependency versions (use with --version)
    #[arg(long)]
    dependencies: bool,
}

/// Parses the command string "buttons,dx,dy" into (u8, i8, i8)
fn parse_command(cmd_str: &str) -> Result<(u8, i8, i8), Error> {
    let parts: Vec<&str> = cmd_str.trim().split(',').collect();
    if parts.len() != 3 {
        bail!("Invalid command format. Expected 'buttons,dx,dy', got '{}'", cmd_str);
    }

    let buttons = parts[0].parse::<u8>().context("Failed to parse buttons")?;
    let dx = parts[1].parse::<i8>().context("Failed to parse dx")?;
    let dy = parts[2].parse::<i8>().context("Failed to parse dy")?;

    Ok((buttons, dx, dy))
}

/// Finds and prepares the Cynthion device for injection.
fn initialize_injector(args: &Args) -> Result<(InjectorDevice, HidInjector), Error> {
    println!("Searching for Cynthion devices...");
    let devices = probe_devices().context("Failed to probe for Cynthion devices")?;

    if devices.is_empty() {
        bail!("No Cynthion injector devices found.");
    }

    println!("Found {} Cynthion device(s):", devices.len());
    for (i, dev) in devices.iter().enumerate() {
         // Assuming InjectorDevice needs a method to get basic info, or nusb DeviceInfo access
         // Let's assume device_info is public or has a getter for now.
        println!(
            "  [{}] Vendor={:#06x}, Product={:#06x}, Speeds={:?}",
            i,
            dev.device_info.vendor_id(), // Need access to device_info from InjectorDevice
            dev.device_info.product_id(),
            dev.supported_speeds()
        );
    }

    if args.device_index >= devices.len() {
        bail!(
            "Device index {} is out of bounds. Only {} devices found.",
            args.device_index,
            devices.len()
        );
    }

    let selected_device = devices[args.device_index].clone(); // Clone InjectorDevice to take ownership for opening
    println!(
        "Selected device [{}]. Opening injector handle...",
        args.device_index
    );
    let mut injector = selected_device.open()?;

    let target_speed = match args.speed.to_lowercase().as_str() {
        "low" => Speed::Low,
        "full" => Speed::Full,
        "high" => Speed::High,
        _ => bail!("Invalid speed specified: '{}'. Use low, full, or high.", args.speed),
    };

    if !selected_device.supported_speeds().contains(&target_speed) {
        bail!(
            "Selected Cynthion device does not support the desired {:?} speed.",
            target_speed
        );
    }

    println!("Preparing injection at {:?} speed...", target_speed);
    injector.prepare_injection(target_speed)?;
    println!("Injection ready.");

    Ok((selected_device, injector)) // Return both if device info is needed later
}

/// Lists available serial ports.
fn list_serial_ports() {
    println!("Available serial ports:");
    match serialport::available_ports() {
        Ok(ports) => {
            if ports.is_empty() {
                println!("  No serial ports found.");
            } else {
                for port in ports {
                    println!("  {}", port.port_name);
                }
            }
        }
        Err(e) => {
            eprintln!("  Error listing serial ports: {}", e);
        }
    }
}

fn main() -> Result<(), Error> {
    let args = Args::parse();

    if args.list {
        println!("--- Cynthion Devices ---");
        match probe_devices() {
            Ok(devices) if devices.is_empty() => println!("No Cynthion injector devices found."),
            Ok(devices) => {
                for (i, dev) in devices.iter().enumerate() {
                    println!(
                        "  [{}] Vendor={:#06x}, Product={:#06x}, Speeds={:?}",
                        i,
                        dev.device_info.vendor_id(),
                        dev.device_info.product_id(),
                        dev.supported_speeds()
                    );
                }
            }
            Err(e) => eprintln!("Error probing Cynthion devices: {}", e),
        }
        println!("\n--- Serial Ports ---");
        list_serial_ports();
        return Ok(());
    }

    // --- Version Command Handling (Optional) ---
    // Clap handles --version automatically based on Cargo.toml,
    // but we handle the custom --dependencies flag here.
    if args.dependencies {
        println!("Packetry Injector version {}\n\n{}", version(), version_info(true));
        return Ok(());
    }


    // --- Initialize Injector ---
   let (_selected_device, mut injector) = initialize_injector(&args)?;

    // --- Setup Ctrl+C Handler ---
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        println!("\nReceived Ctrl+C, shutting down...");
        r.store(false, Ordering::SeqCst);
    })
    .context("Error setting Ctrl-C handler")?;


    // --- Main Loop (UDP or Serial) ---
    if let Some(udp_addr) = args.udp {
        run_udp_server(&udp_addr, &mut injector, running)?;
    } else if let Some(serial_port_name) = args.serial_port {
        run_serial_server(&serial_port_name, args.baud, &mut injector, running)?;
    } else {
        bail!("No operation mode specified. Use --udp or --serial.");
    }

    // --- Cleanup ---
    println!("Stopping injection...");
    // stop_injection() should be called by HidInjector's Drop impl,
    // but we can call it explicitly for clarity or if Drop fails.
    injector.stop_injection().context("Failed to cleanly stop injection")?;
    println!("Exited.");
    Ok(())
}


// --- UDP Server Logic ---
fn run_udp_server(
    addr: &str,
    injector: &mut HidInjector,
    running: Arc<AtomicBool>,
) -> Result<(), Error> {
    let socket = UdpSocket::bind(addr).context(format!("Failed to bind UDP socket to {}", addr))?;
    socket.set_read_timeout(Some(Duration::from_millis(UDP_READ_TIMEOUT_MS)))?;
    println!("Listening for HID commands on UDP {}...", addr);

    let mut buf = [0u8; MAX_UDP_PACKET_SIZE]; // Reusable buffer

    while running.load(Ordering::SeqCst) {
        match socket.recv_from(&mut buf) {
            Ok((num_bytes, src_addr)) => {
                let cmd_str = match std::str::from_utf8(&buf[..num_bytes]) {
                    Ok(s) => s,
                    Err(_) => {
                        eprintln!("Received non-UTF8 data from {}", src_addr);
                        continue; // Skip non-UTF8 data
                    }
                };
                 println!("UDP <- {}: {}", src_addr, cmd_str.trim()); // Log received command

                match parse_command(cmd_str) {
                    Ok((buttons, dx, dy)) => {
                        if let Err(e) = injector.send_mouse_report(buttons, dx, dy) {
                            eprintln!("Error sending HID report: {}", e);
                            // Decide if we should stop on error, or just log and continue
                        }
                    }
                    Err(e) => {
                        eprintln!("Error parsing command from {}: {}", src_addr, e);
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock || e.kind() == std::io::ErrorKind::TimedOut => {
                // Timeout is expected, allows checking the 'running' flag
                continue;
            }
            Err(e) => {
                // Other UDP read error
                eprintln!("UDP receive error: {}", e);
                // Consider breaking the loop on persistent errors
                std::thread::sleep(Duration::from_millis(100)); // Avoid busy-looping on error
            }
        }
    }
    Ok(())
}


// --- Serial Server Logic ---
fn run_serial_server(
    port_name: &str,
    baud_rate: u32,
    injector: &mut HidInjector,
    running: Arc<AtomicBool>,
) -> Result<(), Error> {
    let port = serialport::new(port_name, baud_rate)
        .timeout(Duration::from_millis(SERIAL_READ_TIMEOUT_MS))
        .open()
        .context(format!("Failed to open serial port '{}'", port_name))?;

    // Use BufReader for potentially easier line-based reading
    let mut reader = BufReader::new(port);
    let mut line_buf = String::new();

    println!(
        "Listening for HID commands on Serial {} ({} baud)...",
        port_name, baud_rate
    );

    while running.load(Ordering::SeqCst) {
        line_buf.clear();
        match reader.read_line(&mut line_buf) {
            Ok(0) => {
                // End of stream (port closed?) - only happens if timeout is None
                 if !running.load(Ordering::SeqCst) { break; } // check if we should exit anyway
                 eprintln!("Serial port connection closed?");
                 // Optionally try to reopen or break
                 std::thread::sleep(Duration::from_secs(1));
                 continue; // Or break;
            }
             Ok(_) => {
                  // Successfully read a line
                  println!("Serial <- {}", line_buf.trim()); // Log received command
                match parse_command(&line_buf) {
                    Ok((buttons, dx, dy)) => {
                        if let Err(e) = injector.send_mouse_report(buttons, dx, dy) {
                             eprintln!("Error sending HID report: {}", e);
                            // Decide if we should stop on error
                        }
                    }
                    Err(e) => {
                        eprintln!("Error parsing command from Serial: {}", e);
                    }
                }
            }
             Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {
                // Timeout is expected, allows checking the 'running' flag
                continue;
            }
             Err(e) => {
                 eprintln!("Serial read error: {}", e);
                // Consider breaking or attempting recovery
                std::thread::sleep(Duration::from_millis(100)); // Avoid busy-looping on error
            }
        }
    }
    Ok(())
}
