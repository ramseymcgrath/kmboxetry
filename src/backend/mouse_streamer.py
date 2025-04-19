#!/usr/bin/env python3

import os
from amaranth import *
from amaranth.hdl.rec import Record
from amaranth.hdl.ast import ClockSignal, ResetSignal # For explicit domain connection

# --- LUNA Imports ---
# Ensure LUNA framework is installed (e.g., pip install git+https://github.com/greatscottgadgets/luna)
# Or adjust path if using a local clone.
try:
    # Core ULPI/PHY components
    from luna.gateware.interface.ulpi import ULPIRegisterWindow, PHYResetController
    from luna.gateware.usb.usb2.phy import ULPIPHYGateware

    # Stream definitions (assuming they are in luna_streams.py or accessible via LUNA path)
    # If you saved the stream code from before into luna_streams.py in the same directory:
    # from luna_streams import USBInStreamInterface, USBOutStreamInterface, USBOutStreamBoundaryDetector
    # Otherwise, use LUNA's built-in if available (may differ slightly if LUNA updated)
    from luna.gateware.stream import StreamInterface
    from luna.gateware.usb.stream import USBInStreamInterface, USBOutStreamInterface, USBOutStreamBoundaryDetector

    # Platform/Builder components
    from luna.gateware.platform import CynthionPlatformRev0D4 # Change Rev if needed
    from luna.builder import configure_default_logging, main

    # ECP5 PLL primitive
    from luna.gateware.architecture.lattice.ecp5 import EHXPLLL

except ImportError as e:
    print(f"Import Error: {e}")
    print("Please ensure the LUNA framework is installed and accessible.")
    print("You might need to run: pip install git+https://github.com/greatscottgadgets/luna")
    exit(1)

# --- ULPI Register Definitions ---
# Common addresses, verify against your specific PHY Datasheet if needed
REG_FUNC_CTRL  = 0x04
REG_OTG_CTRL   = 0x0A
# Check datasheet for Line State Register (e.g., 0x07 for TUSB1210, 0x13 for USB3300?)
# USB33x0 datasheet seems to use IFXC register 0x07 bits [4:3] for Line State directly.
# Let's assume 0x07 for now, but this might need adjustment.
REG_LINE_STATE = 0x07 # Example - VERIFY THIS FOR YOUR PHY (USB3300/USB331x Cynthion uses)

# --- Passthrough Analyzer Module ---
class USBPassthroughAnalyzer(Elaboratable):
    """
    Acts as a transparent USB Full Speed passthrough between two ULPI PHYs.

    Connect 'target_usb' to the Host PC and 'control_usb' to the Device.
    Requires platform definitions for 'target_usb', 'control_usb' (ULPI),
    and 'target_usb_pullup' (output signal).

    Assumes 'usb' (48MHz) and 'sync' (60MHz) clock domains are provided.
    """
    def __init__(self):
        # Configuration Values (FS = Full Speed)
        # Function Control: FS Speed (XcvrSelect=00), FS Term (TermSelect=00), Normal OpMode(00)
        # For USB3300: XcvrSelect[1:0]=01 for FS, TermSel=0, OpMode=00 => 0x04 = 0b00000100
        self.func_ctrl_fs_value = 0b00000100

        # OTG Control: Disable OTG Pullup/Pulldown etc. (Needs datasheet check)
        # USB3300: OTG Ctrl func specific, reg 0x0A not quite the same.
        # Maybe set OTG Func Control (0x05[6]) to 0 instead?
        # Let's try clearing DP/DM pull resistors via OTG_CTRL 0x0A[1:0] = 00 for now
        self.otg_ctrl_clear_pulls_value = 0b00 # Value to write to bits [1:0] via RMW

    def elaborate(self, platform):
        m = Module()

        # --- Get Platform Resources ---
        # These are provided by the CynthionPlatform definition
        target_ulpi      = platform.request('target_usb', 0)      # J2 Connector
        control_ulpi     = platform.request('control_usb', 0)     # J3 Connector
        target_pullup_pin = platform.request('target_usb_pullup', 0).o # Pin A18 on r0.4

        # --- Instantiate PHY Gateware ---
        # Host-facing PHY (connects to PC) - Target Port J2
        m.submodules.host_phy = host_phy = ULPIPHYGateware(bus=target_ulpi, handle_clocking=False, clock_domain="sync") # Handle clocking externally

        # Device-facing PHY (connects to real device) - Control Port J3
        m.submodules.dev_phy = dev_phy = ULPIPHYGateware(bus=control_ulpi, handle_clocking=False, clock_domain="sync") # Handle clocking externally


        # --- Direct Stream Connections (PHY <-> Streams) ---
        # Note: Stream domain defaults to 'usb' (48MHz), PHY domain is 'sync' (60MHz)
        # We need DomainRenamer or FIFOs for proper CDC, but for simple FS passthrough
        # let's try direct connection first, relying on handshaking. May need refinement.

        host_phy_rx_stream = USBOutStreamInterface()
        host_phy_tx_stream = USBInStreamInterface()
        dev_phy_rx_stream = USBOutStreamInterface()
        dev_phy_tx_stream = USBInStreamInterface()

        # Connect PHY signals to streams using the bridge methods
        m.d.comb += host_phy_rx_stream.bridge_to(host_phy)
        m.d.comb += host_phy_tx_stream.bridge_to(host_phy)
        m.d.comb += dev_phy_rx_stream.bridge_to(dev_phy)
        m.d.comb += dev_phy_tx_stream.bridge_to(dev_phy)

        # --- Boundary Detectors ---
        # These run in the 'usb' domain (same as the stream processing)
        m.submodules.host_rx_boundary = host_rx_boundary = USBOutStreamBoundaryDetector(domain="usb")
        m.submodules.dev_rx_boundary = dev_rx_boundary = USBOutStreamBoundaryDetector(domain="usb")

        # Connect raw PHY Rx stream (sync domain) to boundary detector input (usb domain)
        # *** This is a Clock Domain Crossing (CDC)! ***
        # For simplicity here, we connect directly, relying on stream handshaking.
        # A robust solution would use AsyncFIFOs.
        m.d.comb += host_rx_boundary.unprocessed_stream.stream_eq(host_phy_rx_stream)
        m.d.comb += dev_rx_boundary.unprocessed_stream.stream_eq(dev_phy_rx_stream)

        # --- Passthrough Data Path ---
        # Connect boundary detector output (usb domain) to Tx stream (usb domain)
        # Host Rx -> Device Tx
        m.d.comb += dev_phy_tx_stream.stream_eq(host_rx_boundary.processed_stream, omit={'ready'})
        m.d.comb += host_rx_boundary.processed_stream.ready.eq(dev_phy_tx_stream.ready)

        # Device Rx -> Host Tx
        m.d.comb += host_phy_tx_stream.stream_eq(dev_rx_boundary.processed_stream, omit={'ready'})
        m.d.comb += dev_rx_boundary.processed_stream.ready.eq(host_phy_tx_stream.ready)

        # --- PHY Configuration and Control ---
        # These controllers run in the 'usb' domain as they sequence operations
        m.submodules.host_phy_rst = PHYResetController(reset=host_phy.phy_reset, domain='usb')
        m.submodules.dev_phy_rst = PHYResetController(reset=dev_phy.phy_reset, domain='usb')

        # Register access windows run in the PHY ('sync') domain
        m.submodules.host_ulpi_regs = host_ulpi_regs = ULPIRegisterWindow(ulpi_bus=target_ulpi, domain='sync')
        m.submodules.dev_ulpi_regs = dev_ulpi_regs = ULPIRegisterWindow(ulpi_bus=control_ulpi, domain='sync')

        # FSM runs in 'usb' domain to sequence the register operations
        phy_configured = Signal()
        dev_line_state_valid = Signal()
        dev_line_state = Signal(8) # Raw register value read

        # Signals to trigger register access (crossing from 'usb' FSM to 'sync' RegWindow)
        # Use pulses/level signals that are safe for CDC or add synchronizers
        start_host_write_func = Signal()
        start_dev_write_func = Signal()
        start_host_write_otg = Signal()
        start_dev_write_otg = Signal()
        start_dev_read_line = Signal()

        # Responses synchronised back from 'sync' to 'usb'
        host_reg_done_sync = Signal()
        dev_reg_done_sync = Signal()
        dev_read_data_sync = Signal(8)

        # Basic 2-flop synchronizers for CDC
        for sig_name_in, sig_name_out in [
            ("host_ulpi_regs.command_complete", "host_reg_done_sync"),
            ("dev_ulpi_regs.command_complete", "dev_reg_done_sync"),
            ("dev_ulpi_regs.read_data", "dev_read_data_sync")]:

            in_sig = locals()[sig_name_in.split('.')[0]].__dict__[sig_name_in.split('.')[1]] if '.' in sig_name_in else locals()[sig_name_in]
            out_sig = locals()[sig_name_out]

            sync_0 = Signal.like(in_sig, name=f"{sig_name_out}_sync0")
            sync_1 = Signal.like(in_sig, name=f"{sig_name_out}_sync1")
            m.d.usb += sync_0.eq(in_sig)
            m.d.usb += sync_1.eq(sync_0)
            m.d.comb += out_sig.eq(sync_1)


        with m.FSM(domain="usb", name="phy_init_fsm") as init_fsm:

            m.d.comb += [ # Drive register starts from FSM state signals
                host_ulpi_regs.write(REG_FUNC_CTRL, self.func_ctrl_fs_value).start.eq(start_host_write_func),
                dev_ulpi_regs.write(REG_FUNC_CTRL, self.func_ctrl_fs_value).start.eq(start_dev_write_func),
                host_ulpi_regs.write(REG_OTG_CTRL, self.otg_ctrl_clear_pulls_value, mask=0b11).start.eq(start_host_write_otg), # RMW bits 0,1
                dev_ulpi_regs.write(REG_OTG_CTRL, self.otg_ctrl_clear_pulls_value, mask=0b11).start.eq(start_dev_write_otg), # RMW bits 0,1
                dev_ulpi_regs.read(REG_LINE_STATE).start.eq(start_dev_read_line),
            ]

            # STARTUP: Wait for PHY resets to complete
            with m.State("STARTUP"):
                with m.If(host_phy_rst.completed & dev_phy_rst.completed):
                     m.next = "DELAY_AFTER_RESET"

            _delay_cnt = Signal(4, reset=15) # Short delay counter
            with m.State("DELAY_AFTER_RESET"):
                 m.d.usb += _delay_cnt.eq(_delay_cnt-1)
                 with m.If(_delay_cnt == 0):
                      m.d.usb += _delay_cnt.eq(15)
                      m.next = "WRITE_HOST_FUNC_CTRL"


            # WRITE_HOST_FUNC_CTRL: Configure Host PHY for FS
            with m.State("WRITE_HOST_FUNC_CTRL"):
                m.d.usb += start_host_write_func.eq(1) # Assert start for one cycle
                m.next = "WAIT_HOST_FUNC_CTRL"
            with m.State("WAIT_HOST_FUNC_CTRL"):
                m.d.usb += start_host_write_func.eq(0)
                with m.If(host_reg_done_sync): # Wait for synchronized completion
                    m.next = "WRITE_DEV_FUNC_CTRL"

            # WRITE_DEV_FUNC_CTRL: Configure Device PHY for FS
            with m.State("WRITE_DEV_FUNC_CTRL"):
                m.d.usb += start_dev_write_func.eq(1)
                m.next = "WAIT_DEV_FUNC_CTRL"
            with m.State("WAIT_DEV_FUNC_CTRL"):
                m.d.usb += start_dev_write_func.eq(0)
                with m.If(dev_reg_done_sync):
                    m.next = "WRITE_HOST_OTG_CTRL"

            # WRITE_HOST_OTG_CTRL: Clear pulls on host PHY
            with m.State("WRITE_HOST_OTG_CTRL"):
                m.d.usb += start_host_write_otg.eq(1)
                m.next = "WAIT_HOST_OTG_CTRL"
            with m.State("WAIT_HOST_OTG_CTRL"):
                 m.d.usb += start_host_write_otg.eq(0)
                 with m.If(host_reg_done_sync):
                     m.next = "WRITE_DEV_OTG_CTRL"

             # WRITE_DEV_OTG_CTRL: Clear pulls on device PHY
            with m.State("WRITE_DEV_OTG_CTRL"):
                 m.d.usb += start_dev_write_otg.eq(1)
                 m.next = "WAIT_DEV_OTG_CTRL"
            with m.State("WAIT_DEV_OTG_CTRL"):
                 m.d.usb += start_dev_write_otg.eq(0)
                 with m.If(dev_reg_done_sync):
                     m.d.usb += phy_configured.eq(1) # Mark configuration as done
                     m.next = "MONITOR_DEVICE_WAIT"

            _monitor_delay = Signal(16, reset=(1<<16)-1) # Read line state periodically
            # MONITOR_DEVICE_WAIT: Periodically check device PHY Line State
            with m.State("MONITOR_DEVICE_WAIT"):
                 m.d.usb += _monitor_delay.eq(_monitor_delay-1)
                 with m.If(_monitor_delay == 0):
                     m.d.usb += _monitor_delay.eq((1<<16)-1) # Reset timer
                     m.next = "READ_DEV_LINE_STATE"

            # READ_DEV_LINE_STATE: Issue read command
            with m.State("READ_DEV_LINE_STATE"):
                 m.d.usb += start_dev_read_line.eq(1)
                 m.next = "WAIT_DEV_LINE_STATE"
            with m.State("WAIT_DEV_LINE_STATE"):
                 m.d.usb += start_dev_read_line.eq(0)
                 with m.If(dev_reg_done_sync): # Wait for read complete
                     m.d.usb += dev_line_state.eq(dev_read_data_sync) # Store sync'd data
                     m.d.usb += dev_line_state_valid.eq(1) # Mark data as valid for one cycle
                     m.next = "MONITOR_DEVICE_WAIT" # Go back to waiting
                 with m.Else():
                     m.d.usb += dev_line_state_valid.eq(0) # Valid only for one cycle


        # --- Host Pull-up Control Logic ---
        # Runs in 'usb' domain, using the latched line_state value
        is_dev_fs_present = Signal()
        with m.If(dev_line_state_valid): # Check only when new data is valid
             # USB3300 LineState[4:3]: 00=SE0, 01=FS K, 10=FS J, 11=LS states/invalid
             phy_line_state_bits = dev_line_state[4:3]
             m.d.usb += is_dev_fs_present.eq( (phy_line_state_bits == 0b01) | \
                                               (phy_line_state_bits == 0b10) )
        # else: keep previous value of is_dev_fs_present


        # Control the actual pull-up pin (combinatorial based on current state)
        m.d.comb += target_pullup_pin.eq(phy_configured & is_dev_fs_present)

        return m


# --- Top-Level Cynthion Module ---
class CynthionUSBPassthroughTop(Elaboratable):
    """ Top-level module for Cynthion board, sets up clocks and instantiates the passthrough analyzer. """
    def elaborate(self, platform):
        m = Module()

        # --- Clock Generation ---
        # Request the main 100MHz clock from the platform
        clk100 = platform.request(platform.default_clk)

        # Define the clock domains we need
        m.domains.sync = ClockDomain()    # 60 MHz for ULPI PHYs
        m.domains.usb  = ClockDomain()    # 48 MHz for USB logic/streams

        # Instantiate the ECP5 PLL
        m.submodules.pll = pll = EHXPLLL()
        pll.register_clkin(clk100, 100e6) # Provide input clock spec

        # Configure PLL for 480MHz VCO (100MHz / 5 = 20MHz PFD * 24 = 480MHz)
        pll.create_clkout(m.domains.sync, 60e6, margin=0) # 480 / 8 = 60MHz
        pll.create_clkout(m.domains.usb, 48e6, margin=0)  # 480 / 10 = 48MHz

        # --- Passthrough Instantiation ---
        m.submodules.passthrough = USBPassthroughAnalyzer()

        # Optional: Add LED indicators?
        # led = platform.request("led", 0).o # Get first LED
        # m.d.comb += led.eq(passthrough_module_signal) # Connect to some internal signal

        return m


# --- Main Build Execution ---
if __name__ == "__main__":
    # Configure logging for LUNA build process
    configure_default_logging()

    # Select the Cynthion board revision you are using
    # Check your Cynthion board silkscreen (e.g., r0.2, r0.4)
    # Options: CynthionPlatformRev0D2, CynthionPlatformRev0D3, CynthionPlatformRev0D4
    platform = CynthionPlatformRev0D4

    # Define build arguments
    build_dir = "build"
    toolchain = "trellis" # For ECP5 using Yosys/nextpnr

    # Ensure build directory exists
    os.makedirs(build_dir, exist_ok=True)

    print(f"Building for platform: {platform.__name__}")
    print(f"Toolchain: {toolchain}")
    print(f"Build output directory: {build_dir}")

    # Build the design
    builder_args = {
        "output_dir": build_dir,
        "toolchain": toolchain,
    }
    main(CynthionUSBPassthroughTop(), platform=platform(), **builder_args)

    print(f"\nBuild complete. Bitstream generated in '{build_dir}/gateware/'")
    print("See flashing guide below.")

