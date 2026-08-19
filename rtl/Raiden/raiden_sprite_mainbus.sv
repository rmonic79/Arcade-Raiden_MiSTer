// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden sprite RAM bus (pattern Irem M72 board_b_d-style + BUFFERED_SPRITERAM16)
// Gestisce spriteram (Main side, $07000-$07FFF, 4KB = 2K word) con:
// - Porta A: Main V30 R/W
// - Buffer: copia spriteram → buf su rising vblank (MAME BUFFERED_SPRITERAM16)
// - Porta B (buffer): renderer R-only
//
// MAME memory map Main:
//   $07000-$07FFF  spriteram (4KB) — share "spriteram" (BUFFERED_SPRITERAM16)
//
// Pattern M72: DOUT_VALID = MRD & sprite_memrq.

module raiden_sprite_mainbus #(parameter SS_IDX = -1)
(
	input  wire        clk,
	input  wire        reset,

	// CPU Main interface
	input  wire [19:0] cpu_addr,
	input  wire        cpu_rd,
	input  wire        cpu_wr,
	input  wire  [1:0] cpu_be,
	input  wire [15:0] cpu_dout,

	// memrq da raiden_addr_main
	input  wire        sprite_memrq,

	// CPU read mux output
	output wire [15:0] DOUT,
	output wire        DOUT_VALID,

	// Vblank rising → copia spriteram → buffer
	input  wire        vblank_rising,

	// Renderer porta B (read buffer)
	input  wire [10:0] spr_vram_addr,
	output wire [15:0] spr_vram_data,

	// Savestate slave (salvo il CPU bank spr_lo/hi; i buffer si ricostruiscono a vblank)
	ssbus_if.slave     ss_spr
);

// Word index (4KB / 2 byte = 2K word)
wire [10:0] spr_word_addr = cpu_addr[11:1];

// M72-native byte lanes: cpu_be[0]=lane bassa, cpu_be[1]=lane alta; cpu_dout già allineato.
wire cpu_we_lo = cpu_wr && cpu_be[0];
wire cpu_we_hi = cpu_wr && cpu_be[1];

// ─── Sprite RAM CPU bank ────────────────────────────────────────────────
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_hi [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin spr_lo[i]=0; spr_hi[i]=0; end end

reg [15:0] spr_cpu_rdata;
wire        spr_we_lo_cpu = sprite_memrq && cpu_we_lo;
wire        spr_we_hi_cpu = sprite_memrq && cpu_we_hi;
wire [15:0] spr_wdata_cpu = cpu_dout;
wire [10:0] spr_idx;
wire        spr_we_lo, spr_we_hi;
wire [15:0] spr_wdata_eff;
ss_ram16_adaptor #(.WIDTHAD(11), .SS_IDX(SS_IDX)) u_ss_spr (
	.clk(clk), .we_lo_in(spr_we_lo_cpu), .we_hi_in(spr_we_hi_cpu),
	.addr_in(spr_word_addr), .wdata_in(spr_wdata_cpu),
	.we_lo_out(spr_we_lo), .we_hi_out(spr_we_hi), .addr_out(spr_idx), .wdata_out(spr_wdata_eff),
	.q_in(spr_cpu_rdata), .ssbus(ss_spr)
);
always @(posedge clk) begin
	if (spr_we_lo) spr_lo[spr_idx] <= spr_wdata_eff[7:0];
	if (spr_we_hi) spr_hi[spr_idx] <= spr_wdata_eff[15:8];
	spr_cpu_rdata <= {spr_hi[spr_idx], spr_lo[spr_idx]};
end

// v110: DOPPIO buffer cascade — sprite a renderer è ritardato di N-2 invece
// di N-1. Test: sprite "anticipa" il BG visivamente, quindi rallentandolo
// di 1 frame extra dovrebbe allinearsi.
// Buffer1 ← spriteram a vblank_rising. Buffer2 ← Buffer1 al successivo
// vblank_rising. Renderer legge Buffer2.

// ─── Sprite RAM Buffer 1 (intermedio, copy on vblank) ───────────────────
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_lo_buf [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_hi_buf [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin spr_lo_buf[i]=0; spr_hi_buf[i]=0; end end

// ─── Sprite RAM Buffer 2 (renderer reads, copy from Buffer1 on vblank) ──
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_lo_buf2 [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] spr_hi_buf2 [0:2047];
initial begin integer i; for (i=0; i<2048; i=i+1) begin spr_lo_buf2[i]=0; spr_hi_buf2[i]=0; end end

// Copy FSM: su rising vblank, copy parallel: spriteram→buf E buf→buf2
reg copying;
reg [11:0] copy_idx;
always @(posedge clk) begin
	if (reset) begin
		copying  <= 1'b0;
		copy_idx <= 12'd0;
	end else if (vblank_rising) begin
		copying  <= 1'b1;
		copy_idx <= 12'd0;
	end else if (copying) begin
		spr_lo_buf[copy_idx[10:0]]  <= spr_lo[copy_idx[10:0]];
		spr_hi_buf[copy_idx[10:0]]  <= spr_hi[copy_idx[10:0]];
		spr_lo_buf2[copy_idx[10:0]] <= spr_lo_buf[copy_idx[10:0]];
		spr_hi_buf2[copy_idx[10:0]] <= spr_hi_buf[copy_idx[10:0]];
		if (copy_idx == 12'd2047) copying <= 1'b0;
		copy_idx <= copy_idx + 12'd1;
	end
end

// Renderer porta B → legge buf2 (= N-2 lag)
reg [15:0] spr_vram_rdata;
always @(posedge clk) spr_vram_rdata <= {spr_hi_buf2[spr_vram_addr], spr_lo_buf2[spr_vram_addr]};
assign spr_vram_data = spr_vram_rdata;

// ─── DOUT_VALID + DOUT (pattern M72) ────────────────────────────────────
reg sprite_rd_lat;

always @(posedge clk) begin
	if (reset) begin
		sprite_rd_lat   <= 1'b0;
	end else begin
		sprite_rd_lat   <= cpu_rd & sprite_memrq;
	end
end

// Core M72 lane-aware: word naturale, il core seleziona il byte (niente swap).
assign DOUT_VALID = sprite_rd_lat;
assign DOUT       = spr_cpu_rdata;

endmodule
