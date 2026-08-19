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

// tile_rom_arbiter — Round-robin arbiter per ROM grafica Raiden.
// Multiplexa i client su un singolo Port0 del sdram_bridge:
//   Client 0: BG  (kind=0)
//   Client 1: MG  (kind=1)
//   Client 2: FG  (kind=2)
//   Client 3: SPR (kind=3)
//   Client 4: TXT (kind=4)  — solo se Text non è in cache BRAM
// Tutti usano letture 32-bit dalla SDRAM.

module tile_rom_arbiter (
	input  wire        clk,
	input  wire        reset,
	input  wire        hblank,   // FG gets exclusive access during hblank

	// Renderer 0 (LEFT)
	input  wire        r0_req,
	input  wire [23:0] r0_addr,
	output reg  [31:0] r0_data,
	output reg         r0_valid,

	// Renderer 1 (CENTER)
	input  wire        r1_req,
	input  wire [23:0] r1_addr,
	output reg  [31:0] r1_data,
	output reg         r1_valid,

	// Renderer 2 (RIGHT)
	input  wire        r2_req,
	input  wire [23:0] r2_addr,
	output reg  [31:0] r2_data,
	output reg         r2_valid,

	// Sprite renderer
	input  wire        r3_req,
	input  wire [23:0] r3_addr,
	output reg  [31:0] r3_data,
	output reg         r3_valid,

	// FG text renderer
	input  wire        r4_req,
	input  wire [23:0] r4_addr,
	output reg  [31:0] r4_data,
	output reg         r4_valid,

	// To sdram_bridge Port0
	output reg         tile_req,
	output reg  [23:0] tile_addr,
	output reg   [2:0] tile_kind,       // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
	input  wire [31:0] tile_data,
	input  wire        tile_valid
);

// =====================================================================
// Tile ROM cache — direct-mapped per-layer
// =====================================================================
// BG/MG 256 entry (addr[9:2] idx, addr[18:10] tag, 9-bit)
// FG   512 entry (addr[10:2] idx, addr[18:11] tag, 8-bit) — raddoppiata per
// scene piene (FG ha most updates in Raiden, glitch HW = miss burst durante
// passaggio nuvole/oggetti densi).
// Indici diversi per FG: cache_idx_fg = tile_addr[10:2], cache_tag_in_fg = tile_addr[18:11].
// Cache data + tag in M10K BRAM (1 ciclo latency read).
// Cache valid: inglobato nel tag come MSB (valid=1 + tag). Reset = scrivo tag=invalid
// implicito (al boot M10K X, cache_hit cmp fallirà sempre fino al primo fill).
// Eliminato reg [N-1:0] cache_valid_* che esplode in fase di map (MUX N-way).
(* ramstyle = "M10K,no_rw_check" *) reg [31:0] cache_data_bg [0:511];
(* ramstyle = "M10K,no_rw_check" *) reg [31:0] cache_data_mg [0:255];
(* ramstyle = "M10K,no_rw_check" *) reg [31:0] cache_data_fg [0:511];
// Tag include valid bit come MSB: {valid, tag[7:0]} BG/FG (512), {valid, tag[8:0]} MG (256)
(* ramstyle = "M10K,no_rw_check" *) reg [8:0]  cache_tag_bg  [0:511];
(* ramstyle = "M10K,no_rw_check" *) reg [9:0]  cache_tag_mg  [0:255];
(* ramstyle = "M10K,no_rw_check" *) reg [8:0]  cache_tag_fg  [0:511];

wire [7:0] cache_idx       = tile_addr[9:2];     // MG (256 entry)
wire [8:0] cache_tag_in    = tile_addr[18:10];   // MG tag (9-bit)
wire [8:0] cache_idx_bg    = tile_addr[10:2];    // BG (512 entry)
wire [7:0] cache_tag_in_bg = tile_addr[18:11];   // BG tag (8-bit)
wire [8:0] cache_idx_fg    = tile_addr[10:2];    // FG (512 entry)
wire [7:0] cache_tag_in_fg = tile_addr[18:11];   // FG tag (8-bit)

// Registered read M10K — 1 ciclo latency. ARB_CHECK aspetta 1 ciclo extra.
reg [31:0] cache_rd_data_bg, cache_rd_data_mg, cache_rd_data_fg;
reg [9:0]  cache_rd_tag_mg;                      // MG: {valid, tag[8:0]}
reg [8:0]  cache_rd_tag_bg, cache_rd_tag_fg;     // BG/FG: {valid, tag[7:0]}
always @(posedge clk) begin
	cache_rd_data_bg  <= cache_data_bg[cache_idx_bg];
	cache_rd_data_mg  <= cache_data_mg[cache_idx];
	cache_rd_data_fg  <= cache_data_fg[cache_idx_fg];
	cache_rd_tag_bg   <= cache_tag_bg[cache_idx_bg];
	cache_rd_tag_mg   <= cache_tag_mg[cache_idx];
	cache_rd_tag_fg   <= cache_tag_fg[cache_idx_fg];
end

// Cache hit: MSB(tag)=valid bit, low bits=tag.
// Confronta {1'b1, tag} con il valore registrato (matcha solo se MSB=1=valid).
wire cache_hit_bg = (cache_rd_tag_bg == {1'b1, cache_tag_in_bg});
wire cache_hit_mg = (cache_rd_tag_mg == {1'b1, cache_tag_in});
wire cache_hit_fg = (cache_rd_tag_fg == {1'b1, cache_tag_in_fg});

wire cache_hit = (active_client == 3'd0) ? cache_hit_bg :
                 (active_client == 3'd1) ? cache_hit_mg :
                 (active_client == 3'd2) ? cache_hit_fg :
                                            1'b0;

wire [31:0] cache_hit_data = (active_client == 3'd0) ? cache_rd_data_bg :
                             (active_client == 3'd1) ? cache_rd_data_mg :
                             (active_client == 3'd2) ? cache_rd_data_fg :
                                                        32'd0;

// Rising edge detection on all 5 clients
reg r0_req_prev, r1_req_prev, r2_req_prev, r3_req_prev, r4_req_prev;

// Pending request bits — set on rising edge, cleared on grant
reg [4:0] pending;

// Latched addresses
reg [23:0] r0_addr_lat, r1_addr_lat, r2_addr_lat, r3_addr_lat, r4_addr_lat;

// Round-robin priority: which client to check first (0-4)
reg [2:0] next_prio;

// Active client being served
reg [2:0] active_client;

// FSM
localparam ARB_IDLE       = 2'd0;
localparam ARB_CHECK_WAIT = 2'd1;  // wait 1 cycle for cache BRAM read latency
localparam ARB_CHECK      = 2'd2;  // cache lookup result ready (registered)
localparam ARB_WAIT       = 2'd3;  // waiting SDRAM
reg [1:0] arb_state;

// Detect rising edges
wire r0_rising = r0_req && !r0_req_prev;
wire r1_rising = r1_req && !r1_req_prev;
wire r2_rising = r2_req && !r2_req_prev;
wire r3_rising = r3_req && !r3_req_prev;
wire r4_rising = r4_req && !r4_req_prev;

// Combinational grant selection (round-robin across 5 clients)
reg [2:0] grant_id;
reg       grant_found;
reg [23:0] grant_addr;

always @(*) begin
	grant_found = 0;
	grant_id    = 3'd0;
	grant_addr  = r0_addr_lat;

	if (hblank && pending[4]) begin
		// HBlank: FG text gets exclusive access for bandwidth
		grant_found = 1;
		grant_id    = 3'd4;
		grant_addr  = r4_addr_lat;
	end else begin
		// Normal: round-robin across all 5 clients
		begin : grant_search
			integer i;
			reg [2:0] check;
			reg [3:0] check_wide;
			for (i = 0; i < 5; i = i + 1) begin
				check_wide = {1'b0, next_prio} + i[3:0];
				check = (check_wide >= 4'd5) ? check_wide[2:0] - 3'd5 : check_wide[2:0];
				if (!grant_found && pending[check]) begin
					grant_id = check;
					case (check)
						3'd0: grant_addr = r0_addr_lat;
						3'd1: grant_addr = r1_addr_lat;
						3'd2: grant_addr = r2_addr_lat;
						3'd3: grant_addr = r3_addr_lat;
						3'd4: grant_addr = r4_addr_lat;
						default: grant_addr = r0_addr_lat;
					endcase
					grant_found = 1;
				end
			end
		end
	end
end

always @(posedge clk) begin
	if (reset) begin
		// cache_tag_* M10K post-power-on contiene X; MSB=valid sarà 0 dopo
		// primo confronto (no FF reset → ok, cache miss garantito al boot).
		r0_req_prev  <= 0;
		r1_req_prev  <= 0;
		r2_req_prev  <= 0;
		r3_req_prev  <= 0;
		r4_req_prev  <= 0;
		pending      <= 5'b00000;
		r0_addr_lat  <= 0;
		r1_addr_lat  <= 0;
		r2_addr_lat  <= 0;
		r3_addr_lat  <= 0;
		r4_addr_lat  <= 0;
		next_prio    <= 0;
		active_client <= 0;
		arb_state    <= ARB_IDLE;
		tile_req     <= 0;
		tile_addr    <= 0;
		tile_kind      <= 3'd0;
		r0_data      <= 0;
		r1_data      <= 0;
		r2_data      <= 0;
		r3_data      <= 0;
		r4_data      <= 0;
		r0_valid     <= 0;
		r1_valid     <= 0;
		r2_valid     <= 0;
		r3_valid     <= 0;
		r4_valid     <= 0;
	end else begin
		// Edge detection
		r0_req_prev <= r0_req;
		r1_req_prev <= r1_req;
		r2_req_prev <= r2_req;
		r3_req_prev <= r3_req;
		r4_req_prev <= r4_req;

		// Latch addresses and set pending on rising edge
		if (r0_rising) begin pending[0] <= 1'b1; r0_addr_lat <= r0_addr; end
		if (r1_rising) begin pending[1] <= 1'b1; r1_addr_lat <= r1_addr; end
		if (r2_rising) begin pending[2] <= 1'b1; r2_addr_lat <= r2_addr; end
		if (r3_rising) begin pending[3] <= 1'b1; r3_addr_lat <= r3_addr; end
		if (r4_rising) begin pending[4] <= 1'b1; r4_addr_lat <= r4_addr; end

		// Clear valid pulses (1-cycle)
		r0_valid <= 0;
		r1_valid <= 0;
		r2_valid <= 0;
		r3_valid <= 0;
		r4_valid <= 0;

		// Clear tile_req after 1 cycle (bridge detects rising edge)
		tile_req <= 0;

		case (arb_state)
			ARB_IDLE: begin
				if (grant_found) begin
					active_client  <= grant_id;
					pending[grant_id] <= 1'b0;
					// Latch grant address for cache/SDRAM use next cycle
					tile_addr      <= grant_addr;
					tile_kind      <= grant_id;       // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
					arb_state      <= ARB_CHECK_WAIT;
				end
			end

			ARB_CHECK_WAIT: begin
				// Aspetto 1 ciclo: tile_addr appena latched in IDLE → cache_idx
				// combinatorio è già nuovo, ma cache BRAM read latency=1 → al ciclo
				// successivo cache_rd_* sono pronti.
				arb_state <= ARB_CHECK;
			end

			ARB_CHECK: begin
				// Cache lookup result is now stable (registered addr from IDLE)
				if (cache_hit && active_client != 3'd3 && active_client != 3'd4) begin
					// Cache hit — deliver data dalla cache per-layer
					case (active_client)
						3'd0: begin r0_data <= cache_hit_data; r0_valid <= 1'b1; end
						3'd1: begin r1_data <= cache_hit_data; r1_valid <= 1'b1; end
						3'd2: begin r2_data <= cache_hit_data; r2_valid <= 1'b1; end
						default: ;
					endcase
					next_prio <= (active_client == 3'd4) ? 3'd0 : active_client + 3'd1;
					arb_state <= ARB_IDLE;
				end else begin
					// Cache miss o sprite/text — fetch da SDRAM
					tile_req  <= 1'b1;
					arb_state <= ARB_WAIT;
				end
			end

			ARB_WAIT: begin
				if (tile_valid) begin
					// Route data to requesting client
					case (active_client)
						3'd0: begin r0_data <= tile_data; r0_valid <= 1'b1; end
						3'd1: begin r1_data <= tile_data; r1_valid <= 1'b1; end
						3'd2: begin r2_data <= tile_data; r2_valid <= 1'b1; end
						3'd3: begin r3_data <= tile_data; r3_valid <= 1'b1; end
						3'd4: begin r4_data <= tile_data; r4_valid <= 1'b1; end
						default: ;
					endcase
					// Fill cache per-layer (BG/MG/FG)
					case (active_client)
						3'd0: begin
							cache_data_bg[cache_idx_bg] <= tile_data;
							cache_tag_bg[cache_idx_bg]  <= {1'b1, cache_tag_in_bg};
						end
						3'd1: begin
							cache_data_mg[cache_idx] <= tile_data;
							cache_tag_mg[cache_idx]  <= {1'b1, cache_tag_in};
						end
						3'd2: begin
							cache_data_fg[cache_idx_fg] <= tile_data;
							cache_tag_fg[cache_idx_fg]  <= {1'b1, cache_tag_in_fg};
						end
						default: ;
					endcase
					// Rotate priority: next after active
					next_prio <= (active_client == 3'd4) ? 3'd0 : active_client + 3'd1;
					arb_state <= ARB_IDLE;
				end
			end

			default: arb_state <= ARB_IDLE;
		endcase
	end
end

endmodule
