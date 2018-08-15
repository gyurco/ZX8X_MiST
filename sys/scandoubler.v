//
// Scandoubler from zx01_mist.v
// 
// Copyright (c) 2014 Till Harbaum <till@harbaum.org> 
// 
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>. 

module scandoubler (
  // system interface
  input 	          clk,
  input            ce_2pix,

  input            scanlines,
		    
  // video input
  input            csync,
  input            v_in,

  // output interface
  output reg          hs_out,
  output reg          vs_out,
  output reg          v_out
);

// column counter running at 13MHz, twice the zx81 pixel clock
reg [8:0] sd_col;

// column counter running at 13MHz, but counting through a whole zx81 line
reg [9:0] zx_col;

// counter to determine sync lengths in the composity sync signal
// used to differentiate between hsync and vsync
reg [7:0] sync_len;
//reg vs, csD /* synthesis noprune */;
reg vs, csD;

// horizontal display goes from 40 to 168. We add 16 border pixels left and right
wire h_de = (sd_col >= (2*32)) && (sd_col < 2*182);   // 176

// vertical display goes from line 32 to 224.We add 16 border pixels top and bottom
wire v_de = (line_cnt >= 16) && (line_cnt < 296);    // 240  

wire hs = sd_col >= (2*192);

// line counter also for debug purposes
reg [9:0] line_cnt /* synthesis noprune */;

// enough space for two complete lines (incl. border and sync),
// each being 414 physical pixels wide
reg       line_buffer[1023:0];

// toggle bit to switch between both line buffers
reg sd_toggle;

// video output of scan doubler
reg sd_video;

// scan doublers hsync/vsync generator runs on 6.5MHz
always @(posedge clk) begin

   reg scanline;
	
	if (ce_2pix) begin

		csD <= csync;

		if(csync) begin
			sync_len <= 8'd0;
			vs_out <= 1'b0;
		end else begin
			// count sync pulse length. Stop counting at 255
			if(sync_len < 255)
				sync_len <= sync_len + 8'd1;

			// if counter passes 80 then we are seeing a vsync
			if(sync_len == 80) begin
				vs_out <= 1'b1;
				line_cnt <= 10'd0;
				scanline <= 0;
			end
		end
		hs_out <= hs;

		// reset scan doubler column counter on rising edge of csync (end of sync) or
		// every 414 pixels
		if((sd_col == 413) ||(csync && !csD && sync_len < 80)) begin
			sd_col <= 9'd0;
			scanline <= !scanline;
		end else
			sd_col <= sd_col + 9'd1;
		
		// change toggle bit at the end of each zx line
		if(csync && !csD) begin
			sd_toggle <= !sd_toggle;
			line_cnt <= line_cnt + 10'd1;
		end
			
		// zx81 column counter
		if((csync && !csD && sync_len < 80)) begin
			zx_col <= 10'd0;
		end else 
			zx_col <= zx_col + 10'd1;

		// fetch one line at half the scan doubler frequency
		if(zx_col[0])
			line_buffer[{sd_toggle, zx_col[9:1]}] <= v_in;
		
		// output other line at full scan doubler frequency
		if (scanlines & scanline)
			v_out <= 0;
		else
			v_out <= line_buffer[{!sd_toggle, sd_col}] && v_de && h_de;
	end
end
endmodule
