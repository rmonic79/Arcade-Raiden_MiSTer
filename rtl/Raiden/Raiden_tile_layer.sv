// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
/*  Raiden_MiSTer — Tile-layer parametrico Seibu D-Con (16x16, 4bpp, 32x32).
    Author: Umberto Parisi (rmonic79)

    Modulo unico per BG / MG / FG. Differenze fra layer (da MAME dcon.cpp):

      LAYER  COLOR_BASE  TRANSP  GFX_BANK  TILE_KIND  SCROLL+128
      BG     0x400       no      no        0          sì
      MG     0x500       sì(15)  sì        1          sì
      FG     0x600       sì(15)  no        2          sì

    Parametri:
      COLOR_BASE   : 11-bit base palette
      HAS_TRANSP   : 1=pen 15 trasparente, 0=sempre opaco
      HAS_GFX_BANK : 1=tile_idx |= gfx_bank_select (per MG)
      TILE_KIND    : 3-bit kind passato a SDRAM arbiter

    Architettura: line buffer ping-pong + prefetcher SDRAM (vedi BG renderer
    cancellato — qui generalizzato).

    Decode tile (dcon_tilelayout):
      base_left  = idx*128 + row*4
      base_right = idx*128 + 64 + row*4
      gruppo (col_local 0..3 di sx o dx) — fetch 32-bit a base+offset:
        byte0 = base+0 (plane 2,3)
        byte1 = base+1 (plane 0,1)
      sub      = 3 - col[1:0]
      pen      = {byte_lo[sub+4], byte_lo[sub], byte_hi[sub+4], byte_hi[sub]}
*/

module Raiden_tile_layer #(
	parameter [10:0] COLOR_BASE   = 11'h400,
	parameter        HAS_TRANSP   = 0,
	parameter        HAS_GFX_BANK = 0,
	parameter  [2:0] TILE_KIND    = 3'd0,
	parameter        MIRROR_H     = 0
) (
	input  wire        clk,
	input  wire        reset,
	input  wire        ce_pix,

	input  wire  [4:0] decode_mode,    // OSD-driven pen decoder mode 0..31

	input  wire  [9:0] hpos,
	input  wire  [8:0] vpos,
	input  wire        de,
	input  wire        layer_en,
	input  wire        new_line,

	input  wire [15:0] scroll_x,
	input  wire [15:0] scroll_y,

	// OSD offset di rendering (debug pixel-hunting, indipendente dallo scroll content)
	input  wire signed [9:0] xoff,
	input  wire signed [9:0] yoff,

	// Solo per MG: high bits aggiunti al tile_idx
	input  wire [15:0] gfx_bank,

	// VRAM read port (CPU dual-port lato B)
	output reg  [10:0] vram_addr,
	input  wire [15:0] vram_data,

	// SDRAM tile fetch via arbiter
	output reg         rom_req,
	output reg  [23:0] rom_addr,
	input  wire [31:0] rom_data,
	input  wire        rom_valid,

	// Output pixel (combinatoriale: ultimo pixel a destra mostrato senza latency)
	output wire        opaque,
	output wire [10:0] pen_index
);

	// ─── Line buffers (ping-pong) ────────────────────────────────────────────
	// 12 bit/pixel: {color[3:0], 4'd0, pen[3:0]} → ricostruito al read in pen_index
	// Bit10 (extra) per "pen=15 transparent" = bit 7 del campo a 12 bit (impostato da decode)
	// Layout: [11:8]=color, [7]=transp_flag, [6:4]=000, [3:0]=pen
	(* ramstyle = "M10K,no_rw_check" *) reg [11:0] linebuf0 [0:511];
	(* ramstyle = "M10K,no_rw_check" *) reg [11:0] linebuf1 [0:511];
	reg        active_buf;

	// Prefetcher state
	reg  [4:0] tile_col_pf;
	reg  [3:0] pf_state;
	reg [11:0] pf_tile_idx;
	reg  [3:0] pf_tile_clr;
	reg        pf_side;
	reg [31:0] pf_rom_data;
	reg  [3:0] decode_step;

	localparam PF_IDLE    = 4'd0;
	localparam PF_VRAM_R  = 4'd1;
	localparam PF_VRAM_W  = 4'd2;
	localparam PF_VRAM_W2 = 4'd3;   // wait extra: BRAM read latency
	localparam PF_ROM_REQ = 4'd4;
	localparam PF_ROM_W   = 4'd5;
	localparam PF_DECODE  = 4'd6;
	localparam PF_NEXT    = 4'd7;
	localparam PF_DONE    = 4'd8;
	localparam PF_CLEAR   = 4'd9;   // pre-clear linebuf scanline-start (fix linea persistente)

	// Sentinel "pixel vuoto": transp=1 → opaque=0 al read → mostra layer sotto.
	localparam [11:0] LB_EMPTY = 12'h080;
	reg [8:0] clear_idx;

	// Coordinate prefetch — target = riga da mostrare al new_line successivo.
	//
	// Logica corretta (non più gated VBLANK):
	//   Durante display vpos=N (N<V_VISIBLE-1): prefetch riga N+1 nel buffer
	//     non-attivo. Al new_line N→N+1: swap → display mostra riga N+1.
	//   Durante display vpos=V_VISIBLE-1: prefetch riga 0 (next frame).
	//   Durante VBLANK (vpos >= V_VISIBLE): prefetch IGNORATO (= idle), così
	//     il buffer caricato durante vpos=V_VISIBLE-1 non viene sovrascritto.
	//
	// Toggle buffer al new_line — solo quando entriamo in una riga visibile
	// (vpos=0..V_VISIBLE-1 dopo wrap).
	//
	// PROBLEMA tempistico: la prefetch parte al new_line che inizia riga N+1.
	// Durante riga N+1 il prefetch sta caricando il buffer per riga N+2.
	// Al new_line N+1→N+2 swap → buffer ora-pieno diventa attivo.
	// → FUNZIONA solo se il prefetch finisce ENTRO una riga (= 384 pixel × 16 cicli
	//    = 6144 cicli). Per 21 tile × 2 fetch × ~30 cicli = ~1260 cicli. OK ✓
	//
	// PRIMA RIGA del frame: serve buffer pre-caricato durante VBLANK precedente.
	//   Soluzione: durante riga V_VISIBLE-1 prefetch fa target_y=0 (next frame),
	//   poi VBLANK no toggle. Al wrap V_TOTAL→0 SWAP → buffer ha riga 0 pronta.
	localparam [8:0] V_VISIBLE = 9'd224;

	// Scroll latch (v111 fix): snapshot a vpos==V_VISIBLE (= rising VBlank,
	// stesso istante della copy spriteram in raiden_sprite_mainbus). Cattura
	// scroll PRE-IRQ vblank handler = scroll stato CPU per frame display CORRENTE
	// (non N+1 come prima). Sprite buffer copy stesso istante = sprite stato
	// CPU stesso. ENTRAMBI = stato CPU pre-IRQ frame N → ALLINEATI.
	// Prefetch riga 0 spostato a vpos==V_VISIBLE (= dentro VBlank, DOPO latch
	// update). Margine: 39 linee VBlank (~200k cicli) >> 1260 prefetch ✓.
	reg [15:0] scroll_x_lat, scroll_y_lat;
	always @(posedge clk) begin
		if (reset) begin
			scroll_x_lat <= 16'd0;
			scroll_y_lat <= 16'd0;
		end else if (new_line && (vpos == V_VISIBLE)) begin
			scroll_x_lat <= scroll_x;
			scroll_y_lat <= scroll_y;
		end
	end

	wire [15:0] target_y = (vpos == V_VISIBLE) ? 16'd0
	                                           : ({7'd0, vpos} + 16'd1);
	wire [15:0] eff_y_pf = target_y + scroll_y_lat + {{6{yoff[9]}}, yoff};
	wire  [4:0] tile_y_pf = eff_y_pf[8:4];
	wire  [3:0] row_pf    = eff_y_pf[3:0];

	// gated_new_line: trigger prefetch per vpos visibili + vpos=V_VISIBLE
	// (= rising VBlank, per prefetch riga 0 next frame con scroll fresh).
	wire vpos_visible    = (vpos < V_VISIBLE);
	wire vpos_first_vbl  = (vpos == V_VISIBLE);
	wire gated_new_line  = new_line & (vpos_visible | vpos_first_vbl);

	wire [4:0] first_tile_x   = scroll_x_lat[8:4];
	wire [3:0] first_pixel_off = scroll_x_lat[3:0];

	wire [4:0] cur_tile_x = first_tile_x + tile_col_pf;
	wire signed [10:0] dst_x_signed = ({1'b0, tile_col_pf, 4'd0}) - {7'd0, first_pixel_off} + {{1{xoff[9]}}, xoff};

	// Tile index con eventuale gfx_bank (MG ha bit 12 = 13-bit totali, BG/FG = 12 bit)
	wire [12:0] effective_tile_idx = HAS_GFX_BANK
	                                  ? ({1'b0, pf_tile_idx} | gfx_bank[12:0])
	                                  : {1'b0, pf_tile_idx};

	// ─── Prefetcher main FSM ─────────────────────────────────────────────────
	always @(posedge clk) begin
		if (reset) begin
			pf_state    <= PF_IDLE;
			tile_col_pf <= 5'd0;
			rom_req     <= 1'b0;
			vram_addr   <= 11'd0;
			active_buf  <= 1'b0;
			decode_step <= 4'd0;
			pf_side     <= 1'b0;
			clear_idx   <= 9'd0;
		end else begin
			case (pf_state)
				PF_IDLE: begin
					if (gated_new_line) begin
						active_buf  <= ~active_buf;
						tile_col_pf <= 5'd0;
						pf_side     <= 1'b0;
						clear_idx   <= 9'd0;
						pf_state    <= PF_CLEAR;
					end
				end

				// Pre-clear linebuf non-attivo a LB_EMPTY (transp=1).
				// Evita linea persistente da scanline precedente su bordi non coperti.
				PF_CLEAR: begin
					if (active_buf == 1'b0)
						linebuf1[clear_idx] <= LB_EMPTY;
					else
						linebuf0[clear_idx] <= LB_EMPTY;
					if (clear_idx == 9'd319) begin
						clear_idx <= 9'd0;
						pf_state  <= PF_VRAM_R;
					end else begin
						clear_idx <= clear_idx + 9'd1;
					end
				end

				PF_VRAM_R: begin
					// Emetto vram_addr (registered alla fine di questo ciclo).
					// MAME raiden.cpp:443 BG e FG usano TILEMAP_SCAN_COLS:
					//   tile_index = col * height + row = col * 32 + row
					// Quindi vram_addr = {col[4:0], row[4:0]} (col high, row low).
					vram_addr <= {1'b0, cur_tile_x, tile_y_pf};
					pf_state  <= PF_VRAM_W;
				end

				PF_VRAM_W: begin
					// vram_addr è effettivo all'inizio di questo ciclo.
					// La BRAM dual_port produce vram_data alla FINE di questo ciclo
					// (registered output). Quindi vram_data sarà valido in PF_VRAM_W2.
					pf_state <= PF_VRAM_W2;
				end

				PF_VRAM_W2: begin
					// vram_data ora valido (= dato di addr emesso in PF_VRAM_R).
					pf_tile_idx <= vram_data[11:0];
					pf_tile_clr <= vram_data[15:12];
					pf_state    <= PF_ROM_REQ;
				end

				PF_ROM_REQ: begin
					// rom_addr = tile_idx*128 + side_off + row*4
					rom_addr <= ({4'd0, effective_tile_idx, 7'd0})
					           + (pf_side ? 24'd64 : 24'd0)
					           + ({18'd0, row_pf, 2'd0});
					rom_req  <= 1'b1;
					pf_state <= PF_ROM_W;
				end

				PF_ROM_W: begin
					if (rom_valid) begin
						pf_rom_data <= rom_data;
						rom_req     <= 1'b0;
						decode_step <= 4'd0;
						pf_state    <= PF_DECODE;
					end
				end

				PF_DECODE: begin
					// MAME raiden_tilelayout (raiden.cpp:685-694):
					//   planes = STEP4(12,-4) = {12, 8, 4, 0}
					//   xoff   = STEP4(0,1) | STEP4(16,1) | STEP4(512,1) | STEP4(528,1)
					//   yoff   = STEP16(0, 32)  (row*32 bit)
					//   total  = 16*16*4 = 1024 bit = 128 byte per tile
					//
					// Per metà tile (8 colonne × 4 plane = 32 bit = 4 byte file).
					// Bridge legge 2 SDRAM word ($g+0, $g+1) e assembla 32-bit:
					//   sdram[+0] = {file[2g+1], file[2g+0]}   ← word little-endian
					//   sdram[+1] = {file[2g+3], file[2g+2]}
					//   tile_data = {sdram[+0], sdram[+1]} (bridge usa big-endian assembly)
					// Quindi mapping byte file → tile_data bit ranges:
					//   file[2g+0] = pf_rom_data[23:16]   (= byte 0 della row)
					//   file[2g+1] = pf_rom_data[31:24]   (= byte 1 della row)
					//   file[2g+2] = pf_rom_data[ 7: 0]   (= byte 2 della row)
					//   file[2g+3] = pf_rom_data[15: 8]   (= byte 3 della row)
					//
					// MAME readbit MSB-first → bit_in_byte = 7 - (bitnum mod 8).
					// Per row=R (yoff già nell'addr), col=C, plane=P:
					//   bitnum = xoff[C] + planes[P]
					// Caso col 0..3 (xoff = 0..3):
					//   plane 0 (offs 12): bit 12+col → byte 1, bit (7-(12+col-8)) = 3-col
					//   plane 1 (offs  8): bit  8+col → byte 1, bit (7-col)
					//   plane 2 (offs  4): bit  4+col → byte 0, bit (3-col)
					//   plane 3 (offs  0): bit  0+col → byte 0, bit (7-col)
					// Caso col 4..7 (xoff = 16..19):
					//   plane 0: bit 16+12+sub → byte 3, bit (3-sub)
					//   plane 1: bit 16+ 8+sub → byte 3, bit (7-sub)
					//   plane 2: bit 16+ 4+sub → byte 2, bit (3-sub)
					//   plane 3: bit 16+ 0+sub → byte 2, bit (7-sub)
					//   (sub = col-4 = decode_step[1:0])
					// La metà destra (col 8-15) è in un fetch separato a offset
					// tile_idx*128 + 64 (pf_side=1), stesso schema applicato.
					begin : decode_blk
						reg [31:0] pf_data_eff;
						reg [7:0] byte_a, byte_b;
						reg [1:0] sub;
						reg [2:0] bit_lo, bit_hi;
						reg [3:0] pen;
						reg signed [10:0] dx;
						reg        transp;
						// decode_mode bit-by-bit:
						//   bit 0: byte mapping a↔b
						//   bit 1: sub direction (sub vs 3-sub)
						//   bit 2: bit position (3-sub/7-sub vs sub/4+sub)
						//   bit 3: pf_rom_data byte-reverse 32-bit
						//   bit 4: nibble-swap per byte (HI↔LO 4-bit)
						pf_data_eff = decode_mode[3] ?
						              {pf_rom_data[7:0], pf_rom_data[15:8], pf_rom_data[23:16], pf_rom_data[31:24]} :
						              pf_rom_data;
						if (decode_mode[4]) begin
							pf_data_eff = {pf_data_eff[27:24], pf_data_eff[31:28],
							               pf_data_eff[19:16], pf_data_eff[23:20],
							               pf_data_eff[11:8],  pf_data_eff[15:12],
							               pf_data_eff[3:0],   pf_data_eff[7:4]};
						end
						if (decode_step[2] == 1'b0) begin
							if (decode_mode[0] == 1'b0) begin
								byte_a = pf_data_eff[31:24]; byte_b = pf_data_eff[23:16];
							end else begin
								byte_a = pf_data_eff[23:16]; byte_b = pf_data_eff[31:24];
							end
						end else begin
							if (decode_mode[0] == 1'b0) begin
								byte_a = pf_data_eff[15:8]; byte_b = pf_data_eff[7:0];
							end else begin
								byte_a = pf_data_eff[7:0];  byte_b = pf_data_eff[15:8];
							end
						end
						sub = decode_mode[1] ? (2'd3 - decode_step[1:0]) : decode_step[1:0];
						if (decode_mode[2] == 1'b0) begin
							bit_lo = 3 - {1'b0, sub};
							bit_hi = 7 - {1'b0, sub};
						end else begin
							bit_lo = {1'b0, sub};
							bit_hi = 4 + {1'b0, sub};
						end
						pen[0] = byte_a[bit_lo];
						pen[1] = byte_a[bit_hi];
						pen[2] = byte_b[bit_lo];
						pen[3] = byte_b[bit_hi];
						transp = (HAS_TRANSP != 0) && (pen == 4'd15);
						dx = dst_x_signed + (pf_side ? 11'sd8 : 11'sd0) + {8'd0, decode_step[2:0]};
						if (dx >= 0 && dx < 256) begin
							if (active_buf == 1'b0)
								linebuf1[dx[8:0]] <= {pf_tile_clr, transp, 3'd0, pen};
							else
								linebuf0[dx[8:0]] <= {pf_tile_clr, transp, 3'd0, pen};
						end
					end
					if (decode_step == 4'd7) begin
						pf_state <= PF_NEXT;
					end else begin
						decode_step <= decode_step + 4'd1;
					end
				end

				PF_NEXT: begin
					if (pf_side == 1'b0) begin
						pf_side  <= 1'b1;
						pf_state <= PF_ROM_REQ;
					end else begin
						pf_side <= 1'b0;
						if (tile_col_pf == 5'd20) begin
							pf_state <= PF_DONE;
						end else begin
							tile_col_pf <= tile_col_pf + 5'd1;
							pf_state    <= PF_VRAM_R;
						end
					end
				end

				PF_DONE: begin
					if (gated_new_line) begin
						active_buf  <= ~active_buf;
						tile_col_pf <= 5'd0;
						pf_side     <= 1'b0;
						clear_idx   <= 9'd0;
						pf_state    <= PF_CLEAR;
					end
				end

				default: pf_state <= PF_IDLE;
			endcase
		end
	end

	// ─── Read side (registrata M10K + lookahead 1 = latenza netta 0) ────────
	// Legge hpos+1: il dato registrato per hpos e' pronto al ciclo di display giusto.
	// Bordo pixel-0 primato dai 48 cicli di hblank (a timing_hpos=47 rd_addr=0 via wrap);
	// active_buf stabile nel visibile (swap a timing_hpos=0). Behavior-preserving.
	wire [8:0] rd_addr = hpos[8:0] + 9'd1;
	reg [11:0] lb0_q, lb1_q;
	always @(posedge clk) if (ce_pix) begin
		lb0_q <= linebuf0[rd_addr];
		lb1_q <= linebuf1[rd_addr];
	end
	wire [11:0] read_data  = active_buf ? lb1_q : lb0_q;
	wire  [3:0] read_color = read_data[11:8];
	wire        read_transp = read_data[7];
	wire  [3:0] read_pen   = read_data[3:0];

	// Output combinatoriale (no latency): ultimo pixel a destra mostrato.
	wire pixel_active = de & layer_en & (hpos < 10'd256) & ~read_transp;
	assign opaque    = pixel_active;
	assign pen_index = pixel_active ? (COLOR_BASE + {3'd0, read_color, read_pen}) : 11'd0;

`ifdef V30_SIM_PROBES
// Probe FAME FETCH: righe in cui il prefetcher NON ha finito la linea prima
// del new_line successivo -> tile mancanti (BG: sentinella = colore solido).
integer pf_starve = 0;
integer pf_lines  = 0;
integer pf_dump   = 0;
always @(posedge clk) begin
	if (gated_new_line) begin
		pf_lines <= pf_lines + 1;
		if (pf_state != PF_IDLE && pf_state != PF_DONE) begin
			pf_starve <= pf_starve + 1;
			if (pf_starve < 25)
				$display("[pfstarve %m] linea#%0d INCOMPLETA: state=%0d col=%0d", pf_lines, pf_state, tile_col_pf);
		end
	end
	// Dump indici tile della riga scelta: mostra se il blocco solido viene da
	// un run di codici uguali/zero in VRAM (contenuto) o da altro.
	if (pf_state == PF_VRAM_W2 && vpos == 9'd80 && pf_dump < 60) begin
		pf_dump <= pf_dump + 1;
		$display("[tiledump %m] col=%0d idx=%03h clr=%0d vram=%04h", tile_col_pf, vram_data[11:0], vram_data[15:12], vram_data);
	end
end
final $display("[pfstarve %m] TOTALE righe incomplete: %0d su %0d", pf_starve, pf_lines);
`endif

endmodule
