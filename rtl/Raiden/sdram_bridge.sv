// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
/*  This file is part of Raiden_MiSTer.

    Raiden_MiSTer is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Raiden_MiSTer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Raiden_MiSTer.  If not, see <http://www.gnu.org/licenses/>.

    Author: Umberto Parisi (rmonic79)
    Version: 1.0
    Date: 2026

*/

// sdram_bridge — Bridge e arbitro SDRAM per Darius.
// Multiplexa 3 client sul controller Genesis 4-port di Sorgelig:
//   Port 0: Tile/Sprite/Text ROM reads + ROM download writes
//   Port 1: Main CPU ROM
//   Port 2: Sub CPU ROM
//   Port 3: unused (audio Z80 ROM lives in BRAM, not SDRAM)
//
// Clients use ACTIVE-LOW level protocol:
//   - CPU raises req_n LOW (active) when it needs data
//   - Bridge translates to Genesis toggle protocol (req XOR → new request)
//   - Bridge holds dtack_n HIGH until SDRAM responds
//   - Bridge pulses dtack_n LOW for one cycle when data arrives
//
// The Genesis controller (sdram.sv by Sorgelig) uses toggle protocol:
//   - Toggle req to start a request
//   - ack matches req when data is ready
//   - dout is shared across all 3 ports (active port has valid data)
//
// SDRAM address map (word addresses, [24:1]):
//   0x000000-0x02FFFF: Tile ROM     (384KB = 192K words)
//   0x030000-0x05FFFF: Main CPU ROM (384KB = 192K words)
//   0x060000-0x07FFFF: Sub CPU ROM  (256KB = 128K words)
//
// During ioctl_download, port 0 is used for ROM writes.
// After download completes, port 0 serves tile reads.

module sdram_bridge (
	input         clk,       // system clock (48+ MHz)
	input         reset,
	input         sdram_ready, // high when SDRAM controller init complete

	// ==================== Download from HPS ====================
	input         ioctl_download,
	input         ioctl_wr,
	input  [26:0] ioctl_addr,
	input  [15:0] ioctl_dout,    // 16-bit: WIDE=1
	input  [15:0] ioctl_index,
	output        ioctl_wait,

	// ==================== Video / Tile ROM (port 0) ====================
	// 32-bit tile data assembled from 2× 16-bit reads
	input  [23:0] tile_byte_addr,  // byte address relativo alla region selezionata
	input         tile_req,        // level: HIGH while data needed
	input   [2:0] gfx_kind,        // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
	output [31:0] tile_data,       // 32-bit assembled {hi_word, lo_word}
	output reg    tile_valid,      // pulse HIGH for 1 cycle when ready

	// ==================== Main CPU ROM (port 1) ====================
	input  [23:0] main_byte_addr,  // byte address from maincpu_map
	input         main_req,        // level: HIGH while data needed
	output [15:0] main_data,       // 16-bit read data
	output reg    main_ready,      // pulse HIGH for 1 cycle when ready

	// ==================== Sub CPU ROM (port 2) ====================
	// Raiden ha 2× V30 (Main + Sub). Questa porta serve il Sub V30:
	//   sub_byte_addr (input) = offset relativo all'inizio ROM Sub
	//   in SDRAM la ROM Sub è a SUB_BASE (word) = $030000 = $060000 byte.
	input  [23:0] sub_byte_addr,
	input         sub_req,
	output [15:0] sub_data,
	output reg    sub_ready,

	// ==================== OKI ADPCM ROM (port 3) ====================
	// jt6295 byte-level read polling: oki_byte_addr 18-bit (256KB),
	// oki_data 8-bit, oki_ok=1 quando data corrisponde all'ultimo addr richiesto.
	input  [17:0] oki_byte_addr,
	output reg [7:0] oki_data,
	output reg    oki_ok,

	// ==================== Genesis SDRAM controller ports ====================
	// Port 0 (tile/download)
	output reg [24:1] sdram_addr0,
	output reg [15:0] sdram_din0,
	output reg        sdram_wrl0,
	output reg        sdram_wrh0,
	output reg        sdram_req0,
	input             sdram_ack0,
	input      [15:0] sdram_dout0,

	// Port 1 (main CPU)
	output reg [24:1] sdram_addr1,
	output     [15:0] sdram_din1,
	output             sdram_wrl1,
	output             sdram_wrh1,
	output reg        sdram_req1,
	input             sdram_ack1,
	input      [15:0] sdram_dout1,

	// Port 2 (sub CPU)
	output reg [24:1] sdram_addr2,
	output     [15:0] sdram_din2,
	output             sdram_wrl2,
	output             sdram_wrh2,
	output reg        sdram_req2,
	input             sdram_ack2,
	input      [15:0] sdram_dout2,

	// Port 3: OKI ADPCM ROM (jt6295)
	output reg  [24:1] sdram_addr3,
	output wire [15:0] sdram_din3,
	output wire        sdram_wrl3,
	output wire        sdram_wrh3,
	output reg         sdram_req3,
	input              sdram_ack3,
	input       [15:0] sdram_dout3
);

// CPU/Audio ROM ports are read-only
assign sdram_din1 = 16'd0;
assign sdram_wrl1 = 1'b0;
assign sdram_wrh1 = 1'b0;
assign sdram_din2 = 16'd0;
assign sdram_wrl2 = 1'b0;
assign sdram_wrh2 = 1'b0;
assign sdram_din3 = 16'd0;
assign sdram_wrl3 = 1'b0;
assign sdram_wrh3 = 1'b0;

// SDRAM base offsets (WORD addresses) — Raiden layout (mra/Raiden (Japan).mra v2):
//   gfx_kind 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
// MRA layout RAW byte-pack (no map=21 dup):
//   byte_base / word_base / size:
//   $000000 / $000000  Main 1            (128KB byte = 64K word)
//   $020000 / $010000  Main 2            (256KB byte = 128K word)
//   $060000 / $030000  Sub V30           (256KB byte = 128K word)
//   $0A0000 / $050000  Audio Z80         ( 64KB byte raw = 32K word)
//   $0B0000 / $058000  Text              ( 64KB byte interleave = 32K word)
//   $0C0000 / $060000  bgtiles           (512KB byte raw = 256K word)
//   $140000 / $0A0000  fgtiles           (512KB byte raw = 256K word)
//   $1C0000 / $0E0000  sprites           (512KB byte raw = 256K word)
//   $240000 / $120000  oki               ( 64KB byte raw = 32K word)
localparam [23:0] MAIN_BASE   = 24'h000000;
localparam [23:0] SUB_BASE    = 24'h030000;
localparam [23:0] TXT_BASE    = 24'h058000;
localparam [23:0] BG_BASE     = 24'h060000;
localparam [23:0] FG_BASE     = 24'h0A0000;
localparam [23:0] MG_BASE     = 24'h060000;  // Raiden: no MG → riuso BG_BASE
localparam [23:0] SPR_BASE    = 24'h0E0000;

// ================================================================
// PORT 0: Download writes + Tile 32-bit reads
// ================================================================

// --- Download byte-pair assembler ---
// MRA sends word-pair interleaved: HI byte (even addr), LO byte (odd addr)
// We assemble into 16-bit words and write to SDRAM
reg        dl_phase;       // 0=first byte, 1=second byte
reg  [7:0] dl_hi_byte;    // registered first byte
reg  [7:0] dl_lo_byte;    // registered second byte
reg [26:0] dl_addr_save;  // address of first byte (word addr = [26:1])
reg        dl_word_valid;  // assembled word ready to write
reg        dl_toggle;      // toggle for SDRAM request
reg        dl_wait_r;      // waiting for SDRAM to complete write
reg  [1:0] dl_bank_idx;   // current bank being written (0, 1, 2, 3) — quadruplication

wire dl_idle = (sdram_ack0 == dl_toggle);  // no pending request

// Registered download mode — prevents phantom toggle on ioctl_download falling edge.
// FIX (pattern Darius2 sdram_bridge.sv:155-163): scendere SOLO quando dl_wait_r=0
// (sequenza bank 0..3 completata). Altrimenti l'ultima word perde i bank 1/2/3.
reg download_active;
always @(posedge clk) begin
	if (reset)
		download_active <= 0;
	else if (ioctl_download)
		download_active <= 1;
	else if (dl_idle && !dl_wait_r)
		download_active <= 0;
end

always @(posedge clk) begin
	if (reset) begin
		dl_toggle     <= 0;
		dl_wait_r     <= 0;
		dl_bank_idx   <= 2'd0;
	end else begin
		// FIX (pattern Darius2 sdram_bridge.sv:171-174): NON resettare dl_wait_r
		// al cadere di ioctl_download — la sequenza multi-bank deve completare.
		// (rimosso "if (~ioctl_download) dl_wait_r <= 0;")

		// WIDE=1: Genesis-style — latch word, then write to banks 0, 1, 2
		if (ioctl_download && ioctl_wr && ioctl_index == 16'd0) begin
			dl_hi_byte   <= ioctl_dout[15:8];
			dl_lo_byte   <= ioctl_dout[7:0];
			dl_addr_save <= ioctl_addr;
			dl_wait_r    <= 1;
			dl_bank_idx  <= 2'd0;   // start with bank 0
			dl_toggle    <= ~dl_toggle;
		end
		else if (dl_wait_r && dl_idle) begin
			if (dl_bank_idx < 2'd3) begin
				// Write same word to next bank
				dl_bank_idx <= dl_bank_idx + 2'd1;
				dl_toggle   <= ~dl_toggle;
			end else begin
				// All 4 banks written — done
				dl_wait_r <= 0;
			end
		end
	end
end

// Stretch ioctl_wait: dopo completamento sequenza 4-bank, tieni wait alto
// per N cicli extra (ripristino commit 213d572).
reg [7:0] dl_stretch_cnt;
reg dl_wait_r_d;
always @(posedge clk) begin
	if (reset) begin
		dl_stretch_cnt <= 0;
		dl_wait_r_d    <= 0;
	end else begin
		dl_wait_r_d <= dl_wait_r;
		// Falling edge di dl_wait_r → carico stretch
		if (dl_wait_r_d & ~dl_wait_r)
			dl_stretch_cnt <= 8'd64;   // ~64 cicli @96 MHz = ~670 ns extra
		else if (dl_stretch_cnt != 0)
			dl_stretch_cnt <= dl_stretch_cnt - 8'd1;
	end
end

assign ioctl_wait = dl_wait_r | (dl_stretch_cnt != 0) | (ioctl_download & ~sdram_ready);

// --- Tile 32-bit prefetch FSM ---
// Two consecutive 16-bit reads assembled into 32-bit
reg [2:0]  tile_state;
reg [15:0] tile_hi_word;
reg        tile_req_prev;
reg        tile_toggle;

localparam [2:0]
	TS_IDLE    = 3'd0,
	TS_REQ_HI  = 3'd1,
	TS_WAIT_HI = 3'd2,
	TS_REQ_LO  = 3'd3,
	TS_WAIT_LO = 3'd4;

wire tile_idle = (sdram_ack0 == tile_toggle);
wire [23:1] tile_word_addr = tile_byte_addr[23:1];

always @(posedge clk) begin
	if (reset || download_active) begin
		tile_state    <= TS_IDLE;
		tile_valid    <= 0;
		// Inherit dl_toggle so no phantom toggle on transition
		tile_toggle   <= dl_toggle;
		tile_req_prev <= 0;
	end else begin
		tile_valid    <= 0;
		tile_req_prev <= tile_req;

		case (tile_state)
			TS_IDLE: begin
				// Rising edge of tile_req → start prefetch
				if (tile_req && !tile_req_prev) begin
					tile_state <= TS_REQ_HI;
				end
			end

			TS_REQ_HI: begin
				// Request high word (addr + 0) when port is idle
				if (tile_idle) begin
					tile_toggle <= ~tile_toggle;
					tile_state  <= TS_WAIT_HI;
				end
			end

			TS_WAIT_HI: begin
				if (tile_idle) begin
					// High word arrived
					tile_hi_word <= sdram_dout0;
					tile_state   <= TS_REQ_LO;
				end
			end

			TS_REQ_LO: begin
				// Request low word (addr + 1) when port is idle
				if (tile_idle) begin
					tile_toggle <= ~tile_toggle;
					tile_state  <= TS_WAIT_LO;
				end
			end

			TS_WAIT_LO: begin
				if (tile_idle) begin
					// Low word arrived → 32-bit data ready
					tile_valid <= 1;
					tile_state <= TS_IDLE;
				end
			end

			default: tile_state <= TS_IDLE;
		endcase
	end
end

// 32-bit output: Big Endian {hi_word, lo_word}
assign tile_data = {tile_hi_word, sdram_dout0};

// --- Port 0 mux: download OR tile ---
// Uses registered download_active to avoid phantom toggle on ioctl_download edge.
always @(*) begin
	if (download_active) begin
		sdram_addr0 = {dl_bank_idx, dl_addr_save[22:1]};  // bank in [24:23]
		sdram_din0  = {dl_hi_byte, dl_lo_byte};
		sdram_wrl0  = 1'b1;
		sdram_wrh0  = 1'b1;
		sdram_req0  = dl_toggle;
	end else begin
		// GFX read: word address con region base secondo gfx_kind, forced bank 0
		begin
			reg [23:0] gfx_base;
			reg [23:0] gfx_addr;
			case (gfx_kind)
				3'd0: gfx_base = BG_BASE;
				3'd1: gfx_base = MG_BASE;
				3'd2: gfx_base = FG_BASE;
				3'd3: gfx_base = SPR_BASE;
				3'd4: gfx_base = TXT_BASE;
				default: gfx_base = BG_BASE;
			endcase
			case (tile_state)
				TS_WAIT_HI,
				TS_REQ_HI:  gfx_addr = {1'b0, tile_word_addr} + gfx_base;
				TS_WAIT_LO,
				TS_REQ_LO:  gfx_addr = {1'b0, tile_word_addr} + gfx_base + 24'd1;
				default:     gfx_addr = {1'b0, tile_word_addr} + gfx_base;
			endcase
			sdram_addr0 = {2'b00, gfx_addr[21:0]};  // force bank 0
		end
		sdram_din0  = 16'd0;
		sdram_wrl0  = 1'b0;
		sdram_wrh0  = 1'b0;
		sdram_req0  = tile_toggle;
	end
end

// ================================================================
// PORT 1: Main CPU ROM — level req → toggle bridge
// ================================================================
// Edge-detect on main_req rising, send one toggle, wait for ack,
// then pulse main_ready for 1 cycle. Hold DTACK by not pulsing
// ready until SDRAM responds.

reg        main_req_prev;
reg        main_pending;
reg [15:0] main_data_reg;

always @(posedge clk) begin
	if (reset) begin
		sdram_req1    <= 0;
		main_pending  <= 0;
		main_ready    <= 0;
		main_req_prev <= 0;
		main_data_reg <= 16'd0;
	end else begin
		main_ready    <= 0;
		main_req_prev <= main_req;

		// Rising edge of main_req AND not already pending → new request
		if (main_req && !main_req_prev && !main_pending) begin
			sdram_req1   <= ~sdram_req1;
			main_pending <= 1;
		end

		// SDRAM responded → latch data + pulse ready
		if (main_pending && (sdram_ack1 == sdram_req1)) begin
			main_data_reg <= sdram_dout1;
			main_ready    <= 1;
			main_pending  <= 0;
		end
	end
end

assign main_data = main_data_reg;

// Word address = byte_addr >> 1, plus MAIN_BASE offset, forced to bank 1
always @(*) begin
	reg [23:0] main_word;
	main_word = {1'b0, main_byte_addr[23:1]} + MAIN_BASE;
	sdram_addr1 = {2'b01, main_word[21:0]};  // force bank 1
end

// ================================================================
// PORT 2: Sub CPU ROM — same pattern as Main
// ================================================================

reg        sub_req_prev;
reg        sub_pending;
reg [15:0] sub_data_reg;

always @(posedge clk) begin
	if (reset) begin
		sdram_req2    <= 0;
		sub_pending   <= 0;
		sub_ready     <= 0;
		sub_req_prev  <= 0;
		sub_data_reg  <= 16'd0;
	end else begin
		sub_ready    <= 0;
		sub_req_prev <= sub_req;

		if (sub_req && !sub_req_prev && !sub_pending) begin
			sdram_req2  <= ~sdram_req2;
			sub_pending <= 1;
		end

		// Latch data at ack moment
		if (sub_pending && (sdram_ack2 == sdram_req2)) begin
			sub_data_reg <= sdram_dout2;
			sub_ready    <= 1;
			sub_pending  <= 0;
		end
	end
end

assign sub_data = sub_data_reg;

always @(*) begin
	reg [23:0] sub_word;
	// Sub V30 ROM in SDRAM @ word offset SUB_BASE (= 0x030000).
	// sub_byte_addr in arrivo dal Sub_top è offset relativo all'inizio ROM
	// (cpu_addr 0xC0000-0xFFFFF → byte_off 0..0x3FFFF).
	sub_word = SUB_BASE + {1'b0, sub_byte_addr[23:1]};
	sdram_addr2 = {2'b10, sub_word[21:0]};  // force bank 2
end

// ================================================================
// PORT 3: OKI ADPCM ROM (256KB byte-stream)
// ================================================================
// MRA: <interleave output=16><part map="21"/></interleave> mette ogni byte
// del file in 1 byte SDRAM (NO duplicazione). 2 byte file consecutivi
// formano 1 word SDRAM (low=byte_pari, high=byte_dispari? — verifico
// guardando il pattern dei tile che funziona).
//
// Tile arbiter usa word_addr = byte_addr >> 1 senza preoccuparsi del byte
// LSB perché legge word interi (32-bit assembly). OKI ha bisogno del byte
// puntuale: word_offset = byte_addr[17:1], byte_lsb = byte_addr[0]
// seleziona LO o HI byte della word SDRAM.
//
// SDRAM byte $240000 / 2 = word $120000.
localparam [23:0] OKI_BASE = 24'h120000;

reg [17:0] oki_addr_prev;
reg        oki_pending;
reg        oki_byte_lsb_d;   // bit0 dell'addr corrente (latched al request)

wire oki_addr_changed = (oki_byte_addr != oki_addr_prev);

always @(posedge clk) begin
	if (reset) begin
		sdram_req3     <= 1'b0;
		oki_addr_prev  <= 18'h3FFFF;
		oki_pending    <= 1'b0;
		oki_byte_lsb_d <= 1'b0;
		oki_data       <= 8'd0;
		oki_ok         <= 1'b0;
	end else begin
		if (oki_addr_changed && !oki_pending) begin
			sdram_req3     <= ~sdram_req3;
			oki_addr_prev  <= oki_byte_addr;
			oki_byte_lsb_d <= oki_byte_addr[0];
			oki_pending    <= 1'b1;
			oki_ok         <= 1'b0;
		end
		if (oki_pending && (sdram_ack3 == sdram_req3)) begin
			oki_data       <= oki_byte_lsb_d ? sdram_dout3[15:8] : sdram_dout3[7:0];
			oki_ok         <= 1'b1;
			oki_pending    <= 1'b0;
		end
	end
end

// Word offset = byte_addr[17:1] + OKI_BASE
always @(*) begin
	reg [23:0] oki_word;
	oki_word = OKI_BASE + {7'd0, oki_byte_addr[17:1]};
	sdram_addr3 = {2'b11, oki_word[21:0]};
end

endmodule
