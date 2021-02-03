#PIN_DEF = olimexice40HX8K.pcf
PIN_DEF = blackice-mx.pcf
DEVICE = hx8k
PROJ=top

all: $(PROJ).rpt $(PROJ).bin

%.blif: %.v
	yosys -DSYNTHESES -p 'synth_ice40 -top $(PROJ) -json $(PROJ).json -blif $@' $<

%.asc: $(PIN_DEF) %.blif
	# olimex nextpnr-ice40 -r --$(DEVICE) --package ct256 --json $(PROJ).json --asc $(PROJ).asc --opt-timing --pcf $(PIN_DEF)
	nextpnr-ice40 -r --$(DEVICE) --package tq144:4k  --json $(PROJ).json --asc $(PROJ).asc --opt-timing --pcf $(PIN_DEF)

%.bin: %.asc
	icepack $< $@

%.rpt: %.asc
	icetime -d $(DEVICE) -mtr $@ $<

%_tb: %_tb.v %.v
	iverilog -o $@ $^

%_tb.vcd: %_tb
	vvp -N $< +vcd=$@

%_syn.v: %.blif
	yosys -p '-D SYNTHESIS read_blif -wideports $^; write_verilog $@'

%_syntb: %_tb.v %_syn.v
	iverilog -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`

%_syntb.vcd: %_syntb
	vvp -N $< +vcd=$@

sim: $(PROJ)_tb.vcd

postsim: $(PROJ)_syntb.vcd

prog: $(PROJ).bin
	stty -F /dev/ttyACM0 raw
	cat $< > /dev/ttyACM0

clean:
	rm -f $(PROJ).blif $(PROJ).asc $(PROJ).rpt $(PROJ).bin $(PROJ).json

.SECONDARY:
.PHONY: all prog clean
