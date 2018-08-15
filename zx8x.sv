//============================================================================
// 
//  ZX80-ZX81 replica for MiST
//  Copyright (C) 2018 György Szombathelyi
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

`default_nettype none

module zx8x
(
   input         CLOCK_27,   // Input clock 27 MHz

   output  [5:0] VGA_R,
   output  [5:0] VGA_G,
   output  [5:0] VGA_B,
   output        VGA_HS,
   output        VGA_VS,

   output        LED,

   output        AUDIO_L,
   output        AUDIO_R,

	input         UART_RX,

   input         SPI_SCK,
   output        SPI_DO,
   input         SPI_DI,
   input         SPI_SS2,
   input         SPI_SS3,
   input         CONF_DATA0,

   output [12:0] SDRAM_A,
   inout  [15:0] SDRAM_DQ,
   output        SDRAM_DQML,
   output        SDRAM_DQMH,
   output        SDRAM_nWE,
   output        SDRAM_nCAS,
   output        SDRAM_nRAS,
   output        SDRAM_nCS,
   output  [1:0] SDRAM_BA,
   output        SDRAM_CLK,
   output        SDRAM_CKE
);

assign LED = ~ioctl_download & ~tape_ready;

`include "build_id.v"
localparam CONF_STR = 
{
	"ZX8X;;",
	"F,O  P  ,Load tape;",
	"O4,Model,ZX80,ZX81;",
	"O5,RAM size,1k,16k;",
	"O6,Video frequency,50Hz,60Hz;",
	"O7,Inverse video,Off,On;",
	"O9,Scanlines,Off,On;",
	"T0,Reset;",
	"V,v1.0.",`BUILD_DATE
};


////////////////////   CLOCKS   ///////////////////
wire clk_sys;
wire locked;

pll pll
(
	.inclk0(CLOCK_27),
	.c0(clk_sys), //26 MHz
	.locked(locked)
);

reg  ce_cpu_p;
reg  ce_cpu_n;
reg  ce_13,ce_65;

always @(negedge clk_sys) begin
	reg [2:0] counter = 0;

	counter  <=  counter + 1'd1;
	ce_cpu_p <= !counter[2] & !counter[1:0];
	ce_cpu_n <=  counter[2] & !counter[1:0];
	ce_65    <= !counter[1:0];
	ce_13    <= !counter[0];
end

//////////////////   MIST ARM I/O   ///////////////////
wire [10:0] ps2_key;
wire [24:0] ps2_mouse;

wire  [7:0] joystick_0;
wire  [7:0] joystick_1;
wire  [1:0] buttons;
wire  [1:0] switches;
wire        scandoubler_disable;
wire [31:0] status;

wire        ioctl_wr;
wire [13:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;
mist_io #(.STRLEN($size(CONF_STR)>>3)) user_io
(
	.clk_sys(clk_sys),
	.CONF_DATA0(CONF_DATA0),
	.SPI_SCK(SPI_SCK),
	.SPI_DI(SPI_DI),
	.SPI_DO(SPI_DO),
	.SPI_SS2(SPI_SS2),
	
	.conf_str(CONF_STR),

	.status(status),
	.scandoubler_disable(scandoubler_disable),
	.buttons(buttons),
	.switches(switches),
	.joystick_0(joystick_0),
	.joystick_1(joystick_1),
	.ps2_key(ps2_key),

	.sd_conf(0),
	.sd_sdhc(1),
	.ioctl_ce(1),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_download(ioctl_download),
	.ioctl_index(ioctl_index),

	// unused
	.ps2_kbd_clk(),
	.ps2_kbd_data(),
	.ps2_mouse_clk(),
	.ps2_mouse_data(),
	.joystick_analog_0(),
	.joystick_analog_1(),
	.sd_ack_conf()
);


///////////////////   CPU   ///////////////////
wire [15:0] addr;
wire  [7:0] cpu_din;
wire  [7:0] cpu_dout;
wire        nM1;
wire        nMREQ;
wire        nIORQ;
wire        nRD;
wire        nWR;
wire        nRFSH;
wire        nHALT;
wire        nINT = addr[6];
wire        nNMI;
wire        nWAIT;
reg       	reset;

T80pa cpu
(
	.RESET_n(~reset),
	.CLK(clk_sys),
	.CEN_p(ce_cpu_p),
	.CEN_n(ce_cpu_n),
	.WAIT_n(nWAIT),
	.INT_n(nINT),
	.NMI_n(nNMI),
	.BUSRQ_n(1),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.HALT_n(nHALT),
	.A(addr),
	.DO(cpu_dout),
	.DI(cpu_din)
);

wire [7:0] io_dout = kbd_n ? 8'hFF : { tape_in, hz50, 1'b0, key_data };

always_comb begin
	case({nMREQ, ~nM1 | nIORQ | nRD})
	    'b01: cpu_din = (~nM1 & nopgen) ? 8'h0 : mem_out;
	    'b10: cpu_din = io_dout;
	 default: cpu_din = 8'hFF;
	endcase
end

wire tape_in = ~UART_RX;
reg init_reset = 1;
reg zx81;
reg mem_16k;
wire hz50 = ~status[6];

always @(posedge clk_sys) begin
	reg old_download;
	old_download <= ioctl_download;
	if(~ioctl_download & old_download & !ioctl_index) init_reset <= 0;
	if(~ioctl_download & old_download & ioctl_index) tape_ready <= 1;
	
	reset <= buttons[1] | status[0] | (mod[1] & Fn[11]) | init_reset;
	if (reset) begin
		zx81 <= status[4];
		mem_16k <= status[5];
		tape_ready <= 0;
	end
end

//////////////////   MEMORY   //////////////////
reg   [7:0] rom[12288];
reg   [7:0] ram[16384];
wire [12:0] rom_a  = nRFSH ? addr[12:0] : { addr[12:9], ram_data_latch[5:0], row_counter };
wire [13:0] tape_load_addr = (ioctl_index[7:6] == 1) ? tape_addr + 4'd8 : tape_addr - 1'd1;
wire [13:0] ram_a  = tapewrite ? tape_load_addr : mem_16k ? addr[13:0] : { 4'b0000, addr[9:0] };
wire			rom_e  = ~addr[14] & (~addr[12] | zx81);
wire        ram_e  = addr[14];
wire        ram_we = tapewrite | (~(nWR | nMREQ) & addr[14]);
wire  [7:0] ram_in = tapewrite ? tape_in_byte : cpu_dout;
wire  [7:0] rom_out;
wire  [7:0] ram_out;
wire  [7:0] mem_out;

always_comb begin
	casex({ tapeloader, rom_e, ram_e })
		'b110: mem_out = tape_loader_patch[addr - (zx81 ? 13'h0347 : 13'h0207)];
		'b010: mem_out = rom_out;
		'b001: mem_out = ram_out;
		default: mem_out = 8'd0;
	endcase
end
always @(posedge SPI_SCK) begin
	if (ioctl_wr & !ioctl_index) begin
		rom[ioctl_addr] <= ioctl_dout;
	end
end

always @(posedge clk_sys) begin
	rom_out <= rom[{ (zx81 ? rom_a[12] : 2'h2), rom_a[11:0] }];
end

always @(posedge clk_sys) begin
	if (ram_we) begin
		ram[ram_a] <= ram_in;
		ram_out <= ram_in;
	end else
		ram_out <= ram[ram_a];
end

////////////////////  TAPE  //////////////////////
reg   [7:0] tape_ram[16384];
reg         tapeloader, tapewrite;
reg  [13:0] tape_addr;
reg   [7:0] tape_in_byte;
reg         tape_ready;  // there is data in the tape memory
// patch the load ROM routines to loop until the memory is filled from $4000(.o file ) $4009 (.p file)
// xor a; loop: nop or scf, jr nc loop, jp h0207 (jp h0203 - ZX80)
reg   [7:0] tape_loader_patch[7] = '{8'haf, 8'h00, 8'h30, 8'hfd, 8'hc3, 8'h07, 8'h02};

always @(posedge SPI_SCK) begin
	if (ioctl_wr & ioctl_index) begin
		tape_ram[ioctl_addr] <= ioctl_dout;
	end
end

always @(posedge clk_sys) begin
	tape_in_byte <= tape_ram[tape_addr];
end

always @(posedge clk_sys) begin
	reg old_nM1;
	
	old_nM1 <= nM1;
	tapewrite <= tapeloader; //delay writing by one cycle (until tape_in_byte arrives)
	
	if (~nM1 & old_nM1 & tape_ready) begin
		if (zx81) begin
			if (addr == 16'h0347) begin
				tape_loader_patch[1] <= 8'h00; //nop
				tape_loader_patch[5] <= 8'h07; //0207h
				tape_addr <= 13'h0;
				tapeloader <= 1;
			end
			if (addr >= 16'h03c3 || addr < 16'h0347) begin
				tapeloader <= 0;
			end
		end else begin
			if (addr == 16'h0207) begin
				tape_loader_patch[1] <= 8'h00; //nop
				tape_loader_patch[5] <= 8'h03; //0203h
				tape_addr <= 13'h0;
				tapeloader <= 1;
			end
			if (addr >= 16'h024d || addr < 16'h0207) begin
				tapeloader <= 0;
			end
		end
	end
	if (tapeloader) begin
		if (tape_addr != ioctl_addr) begin
			tape_addr <= tape_addr + 1'h1;
		end else begin
			tape_loader_patch[1] <= 8'h37; //scf
		end
	end
end

////////////////////  VIDEO //////////////////////
// Based on the schematic:
// http://searle.hostei.com/grant/zx80/zx80.html

// character generation
wire      nopgen = addr[15] & ~mem_out[6] & nHALT;
wire      data_latch_enable = nRFSH & ce_cpu_n & ~nMREQ;
reg [7:0] ram_data_latch;
reg       nopgen_store;
reg [2:0] row_counter;
wire      shifter_start = nMREQ & nopgen_store & ce_cpu_p & (~zx81 | ~NMIlatch);
reg [7:0] shifter_reg;
reg       video_out;
reg       inverse;

reg[6:0]  back_porch_counter = 1;

always @(posedge clk_sys) begin
	reg old_csync;
	reg old_shifter_start;
	
	old_csync <= csync;
	old_shifter_start <= shifter_start;

	if (data_latch_enable) begin
		ram_data_latch <= mem_out;
		nopgen_store <= nopgen;
	end

	if (nMREQ & ce_cpu_p) inverse <= 0;

	if (~old_shifter_start & shifter_start) begin
		shifter_reg <= (~nM1 & nopgen) ? 8'h0 : mem_out;
		inverse <= ram_data_latch[7];
	end else if (ce_65) begin
		shifter_reg <= { shifter_reg[6:0], 1'b0 };
	end
	
	video_out <= (~status[7] ^ shifter_reg[7] ^ inverse) & !back_porch_counter & csync; 

	if (old_csync & ~csync)	row_counter <= row_counter + 1'd1;
	if (~vsync) row_counter <= 0;

	if (~old_csync & csync) back_porch_counter <= 1;
   if (back_porch_counter) back_porch_counter <= back_porch_counter + 1'd1;

	end

// ZX80 sync generator
reg ic11,ic18,ic19_1,ic19_2;
//wire csync = ic19_2; //ZX80 original
wire csync = vsync & ~hsync;
wire vsync = ic11;

always @(posedge clk_sys) begin

	reg old_nM1;
	old_nM1 <= nM1;
	
	if (~(nIORQ | nWR) & (~zx81 | ~NMIlatch)) ic11 <= 1;
	if (~kbd_n & (~zx81 | ~NMIlatch)) ic11 <= 0;
	
	if (~nIORQ) ic18 <= 1;
	if (~ic19_2) ic18 <= 0;
	
	if (old_nM1 & ~nM1) begin
		ic19_1 <= ~ic18;
		ic19_2 <= ic19_1;
	end
	if (~ic11) ic19_2 <= 0;
end

// ZX81 upgrade
// http://searle.hostei.com/grant/zx80/zx80nmi.html

wire hsync;
reg  NMIlatch;

assign nWAIT = ~(nHALT & ~nNMI) | ~zx81;
assign nNMI = ~(NMIlatch & hsync) | ~zx81;

always @(posedge clk_sys) begin
	reg [7:0] counter = 0;
	
	if (ce_cpu_n) counter <= counter + 1'd1;
	if (counter == 8'd207 | (~nM1 & ~nIORQ)) counter <= 0;
	hsync = (counter >= 16 && counter <= 31);
	
	if (zx81) begin
		if (~nIORQ & ~nWR & (addr[0] ^ addr[1])) NMIlatch <= addr[1];
	end
end

wire       v_sd_out, HS_sd_out, VS_sd_out;

scandoubler scandoubler
(
	.clk(clk_sys),
	.ce_2pix(ce_13),

	.scanlines(status[9]),

	.csync(csync),
	.v_in(video_out),

	.hs_out(HS_sd_out),
	.vs_out(VS_sd_out),
	.v_out(v_sd_out)
);

wire [5:0] R_out,G_out,B_out;
osd osd
(
	.*,
	.R_in((scandoubler_disable ? video_out : v_sd_out) ? 6'b111111 : 6'd0),
	.G_in((scandoubler_disable ? video_out : v_sd_out) ? 6'b111111 : 6'd0),
	.B_in((scandoubler_disable ? video_out : v_sd_out) ? 6'b111111 : 6'd0),
	
	.R_out(R_out),
	.G_out(G_out),
	.B_out(B_out),
	.HSync(scandoubler_disable ? hsync : HS_sd_out),
	.VSync(scandoubler_disable ? vsync : VS_sd_out)
);


assign VGA_HS = scandoubler_disable ? csync : HS_sd_out;
assign VGA_VS = scandoubler_disable ? 1'd1 : VS_sd_out;
assign VGA_R = R_out;
assign VGA_G = G_out;
assign VGA_B = B_out;

////////////////////   HID   /////////////////////

wire kbd_n = nIORQ | nRD | addr[0];

wire [11:1] Fn;
wire  [2:0] mod;
wire  [4:0] key_data;

keyboard kbd( .* );

endmodule
