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

// rom_cache — Instruction cache direct-mapped per CPU ROM.
// 512 entry × 16-bit: hit in 1 ciclo vs 7+ cicli da SDRAM.
// Sta tra il CPU ROM port e l'sdram_bridge. Su miss fa fetch da SDRAM,
// riempie la cache e restituisce il dato. Riduce la latenza del fetch
// del Main 68K.
//
// 512 entries × (15-bit tag + 16-bit data + 1 valid) = ~16 Kbit = 2 M10K
// Index: byte_addr[10:2] (word addr [9:1])
// Tag:   byte_addr[23:11] (word addr [23:10])

module rom_cache #(
	parameter CACHE_BITS = 9  // 2^9 = 512 entries
)
(
	input  wire        clk,
	input  wire        reset,

	// CPU side (from maincpu_map)
	input  wire [23:0] cpu_addr,     // byte address
	input  wire        cpu_req,      // rising edge = new request
	output reg  [15:0] cpu_data,
	output reg         cpu_ready,    // pulse when data valid

	// SDRAM side (to sdram_bridge)
	output reg  [23:0] sdram_addr,
	output reg         sdram_req,    // rising edge = new request
	input  wire [15:0] sdram_data,
	input  wire        sdram_ready   // pulse when data valid
);

localparam ENTRIES = 1 << CACHE_BITS;
localparam TAG_BITS = 23 - CACHE_BITS;  // word addr is [23:1], index is [CACHE_BITS:1]

// Cache storage. Valid dentro la M10K del tag ({valid,tag}): con 8K entry un
// vettore FF sarebbe 8192 registri. Invalidazione = sweep in M10K al reset.
(* ramstyle = "M10K,no_rw_check" *) reg [15:0]        cache_data [0:ENTRIES-1];
(* ramstyle = "M10K,no_rw_check" *) reg [TAG_BITS:0]  cache_tag  [0:ENTRIES-1];

// Address decomposition (word address)
wire [22:0]          word_addr = cpu_addr[23:1];
wire [CACHE_BITS-1:0] idx     = word_addr[CACHE_BITS-1:0];
wire [TAG_BITS-1:0]   tag     = word_addr[22:CACHE_BITS];

// Registered cache read
reg [15:0]         rd_data;
reg [TAG_BITS-1:0] rd_tag;
reg                rd_valid;

always @(posedge clk) begin
	rd_data              <= cache_data[idx];
	{rd_valid, rd_tag}   <= cache_tag[idx];
end

// FSM
localparam S_INIT    = 3'd4;   // sweep invalidazione post-reset
localparam S_IDLE    = 3'd0;
localparam S_CHECK   = 3'd1;
localparam S_MISS    = 3'd2;
localparam S_FILL    = 3'd3;

reg  [2:0]  state;
reg [CACHE_BITS-1:0] init_cnt;
reg         req_prev;
reg [22:0]  pending_word_addr;
reg [TAG_BITS-1:0] pending_tag;
reg [CACHE_BITS-1:0] pending_idx;

// Level-triggered: ogni volta che siamo in IDLE e cpu_req=1 parte un nuovo lookup.
// Trigger fetch successive senza richiedere toggle di cpu_req.
// L'indirizzo viene latchato all'inizio del lookup; cpu_ready resta alto 1 ciclo
// alla fine (per non far ripartire il fetch finché V30 non aggiorna l'addr).
always @(posedge clk) begin
	cpu_ready  <= 1'b0;
	sdram_req  <= 1'b0;

	if (reset) begin
		state             <= S_INIT;
		req_prev          <= 0;
		init_cnt          <= 0;
		pending_word_addr <= 23'h7FFFFF;   // valore impossibile come "last served"
	end else begin
		req_prev <= cpu_req;

		case (state)
			S_INIT: begin
				// sweep: invalida tutte le entry (valid=0 nel tag M10K)
				cache_tag[init_cnt] <= {(TAG_BITS+1){1'b0}};
				init_cnt <= init_cnt + 1'b1;
				if (init_cnt == {CACHE_BITS{1'b1}}) state <= S_IDLE;
			end

			S_IDLE: begin
				// Start new lookup whenever cpu_req is high AND the address
				// is different from the last one served (avoid re-serving in loop).
				if (cpu_req && (word_addr != pending_word_addr || !req_prev)) begin
					pending_word_addr <= word_addr;
					pending_tag       <= tag;
					pending_idx       <= idx;
					state             <= S_CHECK;
				end
			end

			S_CHECK: begin
				// Cache read results available (1 cycle BRAM latency)
				if (rd_valid && rd_tag == pending_tag) begin
					// HIT — return cached data immediately
					cpu_data  <= rd_data;
					cpu_ready <= 1'b1;
					state     <= S_IDLE;
				end else begin
					// MISS — fetch from SDRAM
					sdram_addr <= {pending_word_addr, 1'b0};  // back to byte address
					sdram_req  <= 1'b1;
					state      <= S_MISS;
				end
			end

			S_MISS: begin
				// Wait for SDRAM response
				if (sdram_ready) begin
					// Fill cache and return data
					cache_data[pending_idx]  <= sdram_data;
					cache_tag[pending_idx]   <= {1'b1, pending_tag};
					cpu_data  <= sdram_data;
					cpu_ready <= 1'b1;
					state     <= S_IDLE;
				end
			end

			default: state <= S_IDLE;
		endcase
	end
end

endmodule
