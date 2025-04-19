#!/usr/bin/env python3

import os
from amaranth import *
from amaranth.hdl.rec import Record
from amaranth.hdl.ast import Rose, Fell, ClockSignal, ResetSignal
from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFOBuffered
from amaranth.lib.scheduler import RoundRobin
# Import UART library
from amaranth.lib.uart import AsyncUART, Parity, StopBits

# --- LUNA Imports ---
# Ensure LUNA framework is installed (e.g., pip install git+https://github.com/greatscottgadgets/luna)
try:
    # Core ULPI/PHY components
    from luna.gateware.interface.ulpi import ULPIRegisterWindow, PHYResetController
    from luna.gateware.usb.usb2.phy import ULPIPHYGateware

    # Stream definitions
    from luna.gateware.stream import StreamInterface
    from luna.gateware.usb.stream import USBInStreamInterface, USBOutStreamInterface, USBOutStreamBoundaryDetector

    # USB Packet definitions (for PIDs)
    from luna.gateware.usb.usb2.packet import PID

    # CDC Utilities
    from luna.gateware.utils.cdc import synchronize

    # Platform/Builder components
    from luna.gateware.platform import CynthionPlatformRev0D4 # Change Rev if needed
    from luna.builder import configure_default_logging, main

    # ECP5 PLL primitive
    from luna.gateware.architecture.lattice.ecp5 import EHXPLLL

except ImportError as e:
    print(f"Import Error: {e}")
    print("Please ensure the LUNA framework is installed and accessible.")
    print("You might need to run: pip install amaranth amaranth-boards git+https://github.com/greatscottgadgets/luna.git")
    exit(1)
except Exception as e:
    print(f"An unexpected error occurred during LUNA import: {e}")
    exit(1)


# --- ULPI Register Definitions ---
# Common addresses, verify against your specific PHY Datasheet if needed
REG_FUNC_CTRL  = 0x04
REG_OTG_CTRL   = 0x0A
REG_LINE_STATE = 0x07 # Example - VERIFY for USB3300/USB331x (used by Cynthion)

# Check datasheet for Line State Register bits - Assuming USB33x0 [4:3]
LINE_STATE_SE0 = 0b00
LINE_STATE_FS_K = 0b01
LINE_STATE_FS_J = 0b10


# --- Simple Mouse Packet Injector ---
class SimpleMouseInjector(Elaboratable):
    """
    Generates a simple 3-byte USB mouse report packet (DATA1 PID) when triggered.

    Operates in the 'usb' domain.

    Attributes
    ----------
    source: USBInStreamInterface(), output stream for the packet
    trigger: Signal(), input, assert high for one cycle to start injection
    buttons: Signal(8), input, button state for the report
    dx: Signal(8), input, delta X for the report
    dy: Signal(8), input, delta Y for the report
    """
    def __init__(self):
        # Interface
        self.source = USBInStreamInterface(payload_width=8) # Ensure correct payload width
        # Control Signals (Inputs)
        self.trigger = Signal()
        self.buttons = Signal(8)
        self.dx = Signal(8)
        self.dy = Signal(8)

        # Internal state
        self._data_pid = Signal(4, reset=PID.DATA1) # Simplification: Always uses DATA1

    def elaborate(self, platform):
        m = Module()

        source      = self.source
        buttons     = self.buttons
        dx          = self.dx
        dy          = self.dy

        # Latched inputs on trigger
        latched_buttons = Signal.like(buttons)
        latched_dx = Signal.like(dx)
        latched_dy = Signal.like(dy)

        # Default outputs for the stream
        m.d.comb += [
            source.valid.eq(0),
            source.first.eq(0),
            source.last.eq(0),
            source.payload.eq(0)
        ]

        with m.FSM(domain="usb", name="injector_fsm"):

            with m.State("IDLE"):
                # Wait for a trigger signal
                with m.If(self.trigger):
                    m.d.usb += [ # Latch the inputs on trigger rising edge
                        latched_buttons.eq(buttons),
                        latched_dx.eq(dx),
                        latched_dy.eq(dy),
                    ]
                    m.next = "SEND_PID"

            with m.State("SEND_PID"):
                # Send the DATA1 PID
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(self._data_pid),
                    source.first.eq(1), # First byte of packet
                    source.last.eq(0),
                ]
                # Wait until the PID is accepted before sending data.
                with m.If(source.ready):
                    m.next = "SEND_BYTE_0"

            with m.State("SEND_BYTE_0"):
                # Send the first data byte (Button state)
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_buttons),
                    source.first.eq(0),
                    source.last.eq(0),
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_1"

            with m.State("SEND_BYTE_1"):
                # Send the second data byte (dX)
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_dx),
                    source.first.eq(0),
                    source.last.eq(0),
                ]
                with m.If(source.ready):
                    m.next = "SEND_BYTE_2"

            with m.State("SEND_BYTE_2"):
                 # Send the third data byte (dY)
                m.d.comb += [
                    source.valid.eq(1),
                    source.payload.eq(latched_dy),
                    source.first.eq(0),
                    source.last.eq(1), # Last byte of packet
                ]
                with m.If(source.ready):
                    m.next = "IDLE" # Packet sent, return to idle

        return m

# --- Stream Arbiter (Packet Aware) ---
class PacketArbiter(Elaboratable):
    """
    Arbitrates between two streams, prioritizing inject_stream when valid.
    Only switches streams *between* packets (respects first/last signals).

    Operates in the 'usb' domain.

    Inputs: passthrough_in, inject_in (both USBInStreamInterface layout)
    Output: merged_out (USBInStreamInterface layout)
    """
    def __init__(self):
        self.passthrough_in = USBInStreamInterface(payload_width=8)
        self.inject_in      = USBInStreamInterface(payload_width=8)
        self.merged_out     = USBInStreamInterface(payload_width=8)

    def elaborate(self, platform):
        m = Module()

        passthrough = self.passthrough_in
        inject      = self.inject_in
        merged      = self.merged_out

        # Default: merged_out is not valid, inputs are not ready
        m.d.comb += [
            merged.valid.eq(0),
            passthrough.ready.eq(0),
            inject.ready.eq(0),
        ]

        with m.FSM(domain="usb", name="arbiter_fsm"):

            with m.State("IDLE"):
                # Priority to injection stream if valid (and starting a new packet)
                with m.If(inject.valid):
                    m.next = "FORWARD_INJECT"
                # Otherwise, check passthrough stream if valid (and starting a new packet)
                with m.Elif(passthrough.valid):
                    m.next = "FORWARD_PASSTHROUGH"

            # Forwarding passthrough packet data
            with m.State("FORWARD_PASSTHROUGH"):
                # Connect merged output signals to passthrough input signals
                m.d.comb += merged.stream_eq(passthrough) # Copies valid, payload, first, last
                # Propagate ready back: Passthrough is ready if merged output is ready
                m.d.comb += passthrough.ready.eq(merged.ready)

                # If this is the last byte and it's accepted, return to IDLE to re-arbitrate
                with m.If(passthrough.valid & passthrough.last & merged.ready):
                    m.next = "IDLE"
                # If input becomes invalid unexpectedly during a packet (error?), return to IDLE
                with m.Elif(~passthrough.valid):
                     m.next = "IDLE"
                # Otherwise (still valid, not last or not ready), stay in this state
                with m.Else():
                    m.next = "FORWARD_PASSTHROUGH"


            # Forwarding injection packet data
            with m.State("FORWARD_INJECT"):
                # Connect merged output signals to inject input signals
                m.d.comb += merged.stream_eq(inject)
                 # Propagate ready back: Inject is ready if merged output is ready
                m.d.comb += inject.ready.eq(merged.ready)

                # If this is the last byte and it's accepted, return to IDLE to re-arbitrate
                with m.If(inject.valid & inject.last & merged.ready):
                    m.next = "IDLE"
                # If input becomes invalid unexpectedly during a packet, return to IDLE
                with m.Elif(~inject.valid):
                    m.next = "IDLE"
                # Otherwise (still valid, not last or not ready), stay in this state
                with m.Else():
                    m.next = "FORWARD_INJECT"

        return m

# --- UART Command Handler ---
class UARTCommandHandler(Elaboratable):
    """
    Receives 3 bytes via UART (buttons, dx, dy) and signals when ready.

    Operates in the 'sync' domain.

    Parameters:
        uart_pins : Record containing .rx and .tx signals for UART.
        baud_rate : Desired baud rate (e.g., 115200).
        clk_freq  : Clock frequency of the 'sync' domain (e.g., 60_000_000).

    Attributes (Outputs):
        o_buttons : Signal(8), latched button value from UART.
        o_dx      : Signal(8), latched dx value from UART.
        o_dy      : Signal(8), latched dy value from UART.
        o_cmd_ready : Signal(), pulsed high for one 'sync' cycle when 3 bytes received.
    """
    def __init__(self, *, uart_pins, baud_rate=115200, clk_freq=60_000_000):
        self._pins = uart_pins
        self._baud = baud_rate
        self._clk_freq = clk_freq

        # Output Signals
        self.o_buttons = Signal(8)
        self.o_dx = Signal(8)
        self.o_dy = Signal(8)
        self.o_cmd_ready = Signal() # Pulse

    def elaborate(self, platform):
        m = Module()

        # --- UART Peripheral ---
        # Ensure integer division for divisor
        uart_divisor = int(self._clk_freq // self._baud)
        m.submodules.uart = uart = AsyncUART(
            divisor=uart_divisor,
            pins=self._pins,
            parity=Parity.NONE,
            data_bits=8,
            stop_bits=StopBits.ONE
        )

        # --- Receiver State Machine ('sync' domain) ---
        temp_buttons = Signal(8)
        temp_dx = Signal(8)
        # temp_dy is received directly before latching

        # Default: command not ready, don't ack UART RX
        m.d.comb += [
            self.o_cmd_ready.eq(0),
            uart.rx.ack.eq(0)
        ]
        # Default: don't drive UART TX
        m.d.comb += [
             uart.tx.data.eq(0),
             uart.tx.ack.eq(0) # Changed from default 'req' to 'ack' in amaranth-soc
        ]

        with m.FSM(domain="sync", name="uart_rx_fsm"):

            with m.State("IDLE"):
                # Wait for the first byte (buttons)
                with m.If(uart.rx.rdy):
                    m.d.sync += temp_buttons.eq(uart.rx.data)
                    m.d.comb += uart.rx.ack.eq(1) # Acknowledge receiving the byte
                    m.next = "WAIT_DX"

            with m.State("WAIT_DX"):
                 # Wait for the second byte (dx)
                with m.If(uart.rx.rdy):
                    m.d.sync += temp_dx.eq(uart.rx.data)
                    m.d.comb += uart.rx.ack.eq(1)
                    m.next = "WAIT_DY"

            with m.State("WAIT_DY"):
                 # Wait for the third byte (dy)
                with m.If(uart.rx.rdy):
                    # Latch all values to outputs and signal ready (on the cycle AFTER ack)
                    m.d.sync += [
                        self.o_buttons.eq(temp_buttons),
                        self.o_dx.eq(temp_dx),
                        self.o_dy.eq(uart.rx.data), # Latch dy directly
                    ]
                    # Pulse command ready for one cycle HIGH on the same cycle we ack RX
                    m.d.comb += [
                        self.o_cmd_ready.eq(1),
                        uart.rx.ack.eq(1)
                    ]
                    m.next = "IDLE" # Ready to receive next command sequence

            # TODO: Add handling for UART errors (uart.rx.err.parity, framing, overrun) if needed.
            # Simple error handling: If an error occurs, reset FSM
            # with m.If(uart.rx.err.any()):
            #    m.next = "IDLE"

        # Discard any pending UART errors after handling (or after ignoring)
        # m.d.comb += uart.rx.err.ack.eq(uart.rx.err.any()) # Not directly available in AsyncUART

        return m

# --- USB Passthrough Analyzer with Injection ---
class USBPassthroughAnalyzer(Elaboratable):
    """
    Passthrough core with added packet injection capability via input signals.
    Assumes 'usb' (48MHz) and 'sync' (60MHz) clock domains are provided.

    Inputs (from Top Level):
        i_inject_trigger : Signal() - Single cycle pulse to trigger injection
        i_buttons : Signal(8) - Data for injected packet
        i_dx : Signal(8) - Data for injected packet
        i_dy : Signal(8) - Data for injected packet
    """
    def __init__(self):
        self.func_ctrl_fs_value = 0b00000100 # FS mode for USB3300
        self.otg_ctrl_clear_pulls_value = 0b00 # Clear D+/D- pull R's

        # Input signals for injection control
        self.i_inject_trigger = Signal()
        self.i_buttons = Signal(8)
        self.i_dx = Signal(8)
        self.i_dy = Signal(8)

    def elaborate(self, platform):
        m = Module()

        # --- PHY and Resource Setup ---
        target_ulpi = platform.request('target_usb', 0)      # J2 -> Host PC
        control_ulpi = platform.request('control_usb', 0)    # J3 -> Target Device
        target_pullup_pin = platform.request('target_usb_pullup', 0).o # To Host PC

        m.submodules.host_phy = host_phy = ULPIPHYGateware(bus=target_ulpi, handle_clocking=False, clock_domain="sync")
        m.submodules.dev_phy = dev_phy = ULPIPHYGateware(bus=control_ulpi, handle_clocking=False, clock_domain="sync")

        # Interface streams (raw connections to PHYs - These cross domains via bridge_to)
        host_phy_tx_stream = USBInStreamInterface()  # To Host PHY Tx ('sync' domain expected by bridge_to)
        host_phy_rx_stream = USBOutStreamInterface() # From Host PHY Rx ('sync' domain provided by bridge_to)
        dev_phy_tx_stream = USBInStreamInterface()   # To Device PHY Tx ('sync' domain expected by bridge_to)
        dev_phy_rx_stream = USBOutStreamInterface()  # From Device PHY Rx ('sync' domain provided by bridge_to)

        m.d.comb += host_phy_tx_stream.bridge_to(host_phy)
        m.d.comb += host_phy_rx_stream.bridge_to(host_phy)
        m.d.comb += dev_phy_tx_stream.bridge_to(dev_phy)
        m.d.comb += dev_phy_rx_stream.bridge_to(dev_phy)

        # --- Boundary Detectors (operate in 'usb' domain) ---
        # These handle packet boundaries (first/last) based on NXT/DIR changes.
        m.submodules.host_rx_boundary = host_rx_boundary = USBOutStreamBoundaryDetector(domain="usb")
        m.submodules.dev_rx_boundary = dev_rx_boundary = USBOutStreamBoundaryDetector(domain="usb")

        # Connect PHY Rx stream ('sync') to Boundary Detector ('usb')
        # Note: This direct connection relies on stream handshaking for CDC. Robust designs use AsyncFIFOs.
        m.d.comb += host_rx_boundary.unprocessed_stream.stream_eq(host_phy_rx_stream, omit={'ready'})
        m.d.comb += host_phy_rx_stream.ready.eq(host_rx_boundary.unprocessed_stream.ready)

        m.d.comb += dev_rx_boundary.unprocessed_stream.stream_eq(dev_phy_rx_stream, omit={'ready'})
        m.d.comb += dev_phy_rx_stream.ready.eq(dev_rx_boundary.unprocessed_stream.ready)

        # --- Host to Device Path (Host Rx -> Device Tx, unmodified passthrough) ---
        # Connect Boundary Detector output ('usb') to Device PHY Tx ('sync')
        # Note: This direct connection relies on stream handshaking for CDC. Robust designs use AsyncFIFOs.
        m.d.comb += dev_phy_tx_stream.stream_eq(host_rx_boundary.processed_stream, omit={'ready'})
        m.d.comb += host_rx_boundary.processed_stream.ready.eq(dev_phy_tx_stream.ready)


        # --- Device to Host Path (Device Rx -> Injector/Arbiter -> Host Tx) ---
        # 1. Instantiate Injector and Arbiter (operate in 'usb' domain)
        m.submodules.injector = injector = SimpleMouseInjector()
        m.submodules.arbiter = arbiter = PacketArbiter()

        # 2. Connect Injection Controls (inputs provided to this module)
        m.d.comb += [
            injector.trigger.eq(self.i_inject_trigger),
            injector.buttons.eq(self.i_buttons),
            injector.dx.eq(self.i_dx),
            injector.dy.eq(self.i_dy),
        ]

        # 3. Connect streams to Arbiter (all in 'usb' domain)
        # Passthrough path: Device Boundary Detector output -> Arbiter passthrough input
        m.d.comb += arbiter.passthrough_in.stream_eq(dev_rx_boundary.processed_stream, omit={'ready'})
        m.d.comb += dev_rx_boundary.processed_stream.ready.eq(arbiter.passthrough_in.ready)

        # Injection path: Injector output -> Arbiter inject input
        m.d.comb += arbiter.inject_in.stream_eq(injector.source, omit={'ready'})
        m.d.comb += injector.source.ready.eq(arbiter.inject_in.ready)

        # 4. Connect Arbiter Output ('usb') to Host Tx Path ('sync')
        # Note: This direct connection relies on stream handshaking for CDC. Robust designs use AsyncFIFOs.
        m.d.comb += host_phy_tx_stream.stream_eq(arbiter.merged_out, omit={'ready'})
        m.d.comb += arbiter.merged_out.ready.eq(host_phy_tx_stream.ready)

        # --- PHY Config FSM & Pull-up Control ('usb' domain for FSM, 'sync' for reg access) ---
        m.submodules.host_phy_rst = PHYResetController(reset=host_phy.phy_reset, domain='usb')
        m.submodules.dev_phy_rst = PHYResetController(reset=dev_phy.phy_reset, domain='usb')

        m.submodules.host_ulpi_regs = host_ulpi_regs = ULPIRegisterWindow(ulpi_bus=target_ulpi, domain='sync')
        m.submodules.dev_ulpi_regs = dev_ulpi_regs = ULPIRegisterWindow(ulpi_bus=control_ulpi, domain='sync')

        phy_configured = Signal()
        dev_line_state_valid = Signal() # Flag indicating dev_line_state holds fresh data
        dev_line_state = Signal(8)      # Raw line state register value

        # Signals to start register operations (driven by FSM in 'usb' domain)
        start_host_write_func = Signal()
        start_dev_write_func = Signal()
        start_host_write_otg = Signal()
        start_dev_write_otg = Signal()
        start_dev_read_line = Signal()

        # Synchronized signals from Register Window ('sync') back to FSM ('usb')
        host_reg_done_sync = Signal()
        dev_reg_done_sync = Signal()
        dev_read_data_sync = Signal(8)

        # Implement basic synchronizers for CDC (sync -> usb)
        for sig_name_in_str, sig_name_out_str in [
            ("host_ulpi_regs.command_complete", "host_reg_done_sync"),
            ("dev_ulpi_regs.command_complete", "dev_reg_done_sync"),
            ("dev_ulpi_regs.read_data", "dev_read_data_sync")]:
            try:
                # Get the actual signal objects
                sig_parts = sig_name_in_str.split('.')
                sig_instance = locals()[sig_parts[0]]
                in_sig = getattr(sig_instance, sig_parts[1])
                out_sig = synchronize(m, in_sig, o_domain="usb", name=f"{sig_name_out_str}_cdc")
                # Connect the output of the synchronizer module to our local signal name
                m.d.comb += locals()[sig_name_out_str].eq(out_sig)
            except (KeyError, AttributeError, Exception) as e:
                 # Added generic Exception catch
                 print(f"Warning: Could not find or synchronize signal: {sig_name_in_str} - {e}")
                 # Assign a dummy signal to allow elaboration to continue if needed
                 dummy_like = Signal(8) if 'data' in sig_name_in_str else Signal()
                 m.d.comb += locals()[sig_name_out_str].eq(dummy_like)


        # PHY Initialization and Monitoring FSM ('usb' domain)
        with m.FSM(domain="usb", name="phy_init_fsm") as init_fsm:
            # Connect FSM states to register start signals
            m.d.comb += [
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
                      m.d.usb += _delay_cnt.eq(15) # Reset for next time if needed
                      m.next = "WRITE_HOST_FUNC_CTRL"

            # WRITE_HOST_FUNC_CTRL: Configure Host PHY for FS
            with m.State("WRITE_HOST_FUNC_CTRL"):
                m.d.usb += start_host_write_func.eq(1) # Assert start for one cycle
                m.next = "WAIT_HOST_FUNC_CTRL"
            with m.State("WAIT_HOST_FUNC_CTRL"):
                m.d.usb += start_host_write_func.eq(0) # Deassert start
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
                 m.d.usb += dev_line_state_valid.eq(0) # Ensure valid flag is low unless set below
                 m.d.usb += _monitor_delay.eq(_monitor_delay - 1)
                 with m.If(_monitor_delay == 0):
                     m.d.usb += _monitor_delay.eq((1<<16)-1) # Reset timer
                     m.next = "READ_DEV_LINE_STATE"

            # READ_DEV_LINE_STATE: Issue read command
            with m.State("READ_DEV_LINE_STATE"):
                 m.d.usb += start_dev_read_line.eq(1)
                 m.next = "WAIT_DEV_LINE_STATE"
            with m.State("WAIT_DEV_LINE_STATE"):
                 m.d.usb += start_dev_read_line.eq(0)
                 with m.If(dev_reg_done_sync): # Wait for sync'd read complete
                     m.d.usb += dev_line_state.eq(dev_read_data_sync) # Store sync'd data
                     m.d.usb += dev_line_state_valid.eq(1) # Mark data as valid for one cycle
                     m.next = "MONITOR_DEVICE_WAIT" # Go back to waiting
                 # Remain in this state if dev_reg_done_sync is not high


        # --- Host Pull-up Control Logic ('usb' domain) ---
        # Controls the pull-up resistor seen by the Host PC based on detected device state.
        is_dev_fs_present = Signal()
        with m.If(dev_line_state_valid): # Only update based on fresh line state data
             # Check LineState[4:3] bits from the register value
             # Assumes USB3300: 00=SE0, 01=FS K, 10=FS J, 11=LS states/invalid
             phy_line_state_bits = dev_line_state[4:3]
             m.d.usb += is_dev_fs_present.eq( (phy_line_state_bits == LINE_STATE_FS_K) | \
                                               (phy_line_state_bits == LINE_STATE_FS_J) )
        # else: is_dev_fs_present retains its previous value

        # Enable the host pull-up only after PHYs are configured and a FS device is detected
        m.d.comb += target_pullup_pin.eq(phy_configured & is_dev_fs_present)

        return m

# --- Top-Level Cynthion Module with UART Control ---
class CynthionUartInjectionTop(Elaboratable):
    """ Top-level module integrating Passthrough/Injection core with UART command handler. """
    BAUD_RATE = 115200
    SYNC_CLK_FREQ = 60_000_000 # 60 MHz

    def elaborate(self, platform):
        m = Module()

        # --- Clock Generation ---
        clk100 = platform.request(platform.default_clk) # Base clock from Cynthion
        m.domains.sync = ClockDomain()    # 60 MHz for ULPI PHYs & UART
        m.domains.usb  = ClockDomain()    # 48 MHz for USB logic/streams

        # Instantiate the ECP5 PLL
        m.submodules.pll = pll = EHXPLLL()
        pll.register_clkin(clk100, 100e6) # Input is 100 MHz

        # Create the required clock domains
        pll.create_clkout(m.domains.sync, self.SYNC_CLK_FREQ, margin=0)
        pll.create_clkout(m.domains.usb, 48e6, margin=0) # FS/LS require 48MHz Ref

        # --- UART Handler ---
        # Request PMOD pins (PMOD A: pin 0=RX, pin 1=TX - Check CynthionPlatform file!)
        uart_resource_name = "pmod"
        uart_resource_index = 0
        uart_rx_pin_index = 0 # Typically Pin 1 on PMOD spec (mapped to index 0)
        uart_tx_pin_index = 1 # Typically Pin 2 on PMOD spec (mapped to index 1)
        try:
            pmod_pins = platform.request(uart_resource_name, uart_resource_index)
            # Create a record for uart pins for AsyncUART
            uart_pins = Record([('rx', 1), ('tx', 1)])
            m.d.comb += [
                uart_pins.rx.eq(pmod_pins[uart_rx_pin_index]), # Connect FPGA RX to PMOD pin
                pmod_pins[uart_tx_pin_index].eq(uart_pins.tx), # Connect FPGA TX to PMOD pin
            ]

            m.submodules.uart_handler = uart_handler = UARTCommandHandler(
                uart_pins=uart_pins,
                baud_rate=self.BAUD_RATE,
                clk_freq=self.SYNC_CLK_FREQ
            )

            # --- CDC for UART handler outputs ---
            # Synchronize data signals from UART domain ('sync') to processing domain ('usb')
            sync_buttons = synchronize(m, uart_handler.o_buttons, o_domain="usb", stages=3) # Use 3 stages for better metastability resilience
            sync_dx = synchronize(m, uart_handler.o_dx, o_domain="usb", stages=3)
            sync_dy = synchronize(m, uart_handler.o_dy, o_domain="usb", stages=3)

            # Synchronize the command ready pulse ('sync') and detect rising edge ('usb')
            cmd_ready_sync = synchronize(m, uart_handler.o_cmd_ready, o_domain="usb", stages=3)
            inject_trigger = Signal()
            # Generate a single 'usb' cycle pulse on the rising edge of the synchronized command ready
            m.d.comb += inject_trigger.eq(Rose(cmd_ready_sync, domain="usb"))

            # --- Passthrough/Injector Core ---
            m.submodules.analyzer = analyzer = USBPassthroughAnalyzer()

            # Connect synchronized signals to the analyzer's injection inputs
            m.d.comb += [
                analyzer.i_buttons.eq(sync_buttons),
                analyzer.i_dx.eq(sync_dx),
                analyzer.i_dy.eq(sync_dy),
                analyzer.i_inject_trigger.eq(inject_trigger),
            ]

            # Optional: Blink an LED on command ready for visual feedback
            try:
               led = platform.request("led", 0).o
               # Blink briefly on command ready pulse
               m.d.comb += led.eq(inject_trigger)
            except: # Catch ResourceError or others if LED not present
               pass


        except Exception as e:
            # Handle case where PMOD / UART pins aren't defined correctly
            print(f"\n\n*** Resource Error: Failed to request UART pins ('{uart_resource_name}', {uart_resource_index}). Check platform file. ***\n\n")
            print(f"Error details: {e}")
            # Fallback: Instantiate analyzer without UART connection to allow partial build checks?
            # m.submodules.analyzer = USBPassthroughAnalyzer() # Or just let it fail


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

    # Build the design using the top-level module with UART integration
    builder_args = {
        "output_dir": build_dir,
        "toolchain": toolchain,
        # Add other LUNA build options if needed, e.g.,
        # "verbose": True,
    }
    main(CynthionUartInjectionTop(), platform=platform(), **builder_args)

    print(f"\nBuild complete. Bitstream generated in '{build_dir}/gateware/'")
    print("--- Usage ---")
    print("1. Flash the generated 'top.bit' using 'dfu-util'.")
    print("2. Connect Host PC <-> Cynthion J2 (TARGET).")
    print("3. Connect Target USB Device (Mouse) <-> Cynthion J3 (CONTROL).")
    print("4. Connect CP2102/UART adapter (TX->PMOD A Pin 1, RX->PMOD A Pin 2, GND->GND).")
    print(f"5. Send exactly 3 raw bytes (buttons, dx, dy) via serial at {CynthionUartInjectionTop.BAUD_RATE} baud, 8N1.")
    print("   Example (Python): ser.write(bytes([0, 0, 10])) # Move down 10")
