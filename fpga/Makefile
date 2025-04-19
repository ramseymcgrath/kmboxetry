# Makefile for Cynthion FPGA build

# Tool paths - adjust these to match your system
YOSYS      = yosys
NEXTPNR    = nextpnr-ecp5
ECPPACK    = ecppack
OPENLOADER = openFPGALoader

# Project settings
PROJECT    = cynthion_injector
TOP        = top
DEVICE     = 85k
PACKAGE    = CABGA381
CONSTRAINTS = constraints.lpf
BITSTREAM  = bitstream.bit

# Source files
VERILOG_FILES = $(TOP).v

# Build targets
all: $(BITSTREAM)

# Synthesis with Yosys
$(PROJECT).json: $(VERILOG_FILES)
	$(YOSYS) -p "read_verilog $(VERILOG_FILES); synth_ecp5 -json $@ -top $(TOP)"

# Place and route with nextpnr
$(PROJECT).config: $(PROJECT).json $(CONSTRAINTS)
	$(NEXTPNR) --$(DEVICE) --package $(PACKAGE) --json $(PROJECT).json --lpf $(CONSTRAINTS) --textcfg $@

# Generate bitstream with ecppack
$(BITSTREAM): $(PROJECT).config
	$(ECPPACK) --compress $< $@

# Flash the FPGA
flash: $(BITSTREAM)
	$(OPENLOADER) -b cynthion $(BITSTREAM)

# Clean build artifacts
clean:
	rm -f *.json *.config *.bit

.PHONY: all flash clean