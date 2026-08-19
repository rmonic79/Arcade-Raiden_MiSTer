// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
/*  Raiden_MiSTer — Sprite renderer Seibu SEI0211.
    Author: Umberto Parisi (rmonic79)

    Spriteram: 256 entry × 8 byte (4 word) @ 0x08F800-0x08FFFF.

    Format word (formato "comune", non alt_format) da MAME
    src/mame/seibu/sei021x_sei0220_spr.cpp:
      w0 bit 15      = enable (0=skip, 1=draw)
      w0 bit 14      = flip_x
      w0 bit 13      = flip_y
      w0 bit 12..10  = sizex (3-bit, +1) → 1..8 tile larghezza
      w0 bit 9..7    = sizey (3-bit, +1) → 1..8 tile altezza
      w0 bit 6       = ext (extra bit per priority callback)
      w0 bit 5..0    = color (6-bit, color_base = color << 4)
      w1 bit 15..14  = priority code (2-bit, → pri_cb)
      w1 bit 13..0   = tile_code (14-bit)
      w2 bit 8..0    = X (signed 9-bit)
      w3 bit 8..0    = Y (signed 9-bit)

    Tile size 16×16 4bpp = 128 byte/tile = 32 word 16-bit.
    Multi-tile: tile_code += 1 per sub-tile (TODO: ordine x-major o y-major
    da verificare con dump MAME — assunto x-major per ora).

    Priority callback dcon.cpp::pri_cb:
      pri=0 → above FG
      pri=1 → above MG
      pri=2 → above BG
      pri=3 → above Text (= sotto Text)

    Pen 15 = trasparente.

    Architettura:
      - Sprite scan FSM durante linea N: scorre 256 entry, per quelle che
        intersecano linea N+1 (target_y), fetch SDRAM dei tile coperti e
        scrive in line buffer non-attivo.
      - Read side: a hpos legge line buffer attivo, restituisce pen+pri_code.
      - Ping-pong al new_line.

    Worst case: 256 entry × max 8 sizex × 1 fetch/tile = 2048 fetch/linea.
    Realistico: 30 sprite × 4 sizex avg = 120 fetch. Banda OK.
*/

module Raiden_sprite_renderer (
	input  wire        clk,
	input  wire        reset,
	input  wire        ce_pix,

	input  wire  [9:0] hpos,        // 0..319 logico
	input  wire  [8:0] vpos,        // 0..223 logico
	input  wire        de,
	input  wire        layer_en,
	input  wire        new_line,
	input  wire        flip_screen, // DIP flip: specchia sprite (240-x/240-y, !flip)

	// OSD offset (debug pixel-hunting + hardcoded shift)
	input  wire signed [9:0] xoff,
	input  wire signed [9:0] yoff,

	// Decoder mode OSD selector (0..31)
	input  wire  [4:0] decode_mode,

	// Sprite RAM read port (dual-port lato B)
	output reg  [10:0] spr_addr,    // 2048 word total (512 entry × 4 word)
	input  wire [15:0] spr_data,

	// SDRAM tile fetch via arbiter (client r3, kind=3, no cache)
	output reg         rom_req,
	output reg  [23:0] rom_addr,
	input  wire [31:0] rom_data,
	input  wire        rom_valid,

	// Output pixel (combinatoriale: ultimo pixel a destra mostrato senza latency)
	output wire        opaque,
	output wire [10:0] pen_index,
	output wire  [1:0] pri_code
);

	// ─── Line buffer ping-pong (320 × 10-bit: color[3:0] + pri[1:0] + pen[3:0]) ──
	// Layout 10-bit: [9:6]=color (4-bit MAME Raiden), [5:4]=pri_code, [3:0]=pen
	// Valid pixel = pen != 0xF (15 = trasparente in MAME)
	localparam [9:0] LB_EMPTY = 10'h00F;     // color=0, pri=0, pen=15 (trasparente)
	(* ramstyle = "M10K,no_rw_check" *) reg [9:0] linebuf0 [0:511];
	(* ramstyle = "M10K,no_rw_check" *) reg [9:0] linebuf1 [0:511];
	reg        active_buf;

	// ─── Sprite scan FSM ─────────────────────────────────────────────────────
	// Stati: IDLE → CLEAR (azzera buffer non-attivo a pen=15) → RW0..3 (legge
	// 4 word di entry) → CHECK → ROM_REQ/W → DECODE → NEXT_TX/E → DONE.
	localparam SC_IDLE     = 4'd0;
	localparam SC_CLEAR    = 4'd1;
	localparam SC_RW0      = 4'd2;
	localparam SC_RW1      = 4'd3;
	localparam SC_RW2      = 4'd4;
	localparam SC_RW3      = 4'd5;
	localparam SC_CHECK    = 4'd6;
	localparam SC_CHECK2   = 4'd7;
	localparam SC_ROM_REQ  = 4'd8;
	localparam SC_ROM_W    = 4'd9;
	localparam SC_DECODE   = 4'd10;
	localparam SC_NEXT_TX  = 4'd11;
	localparam SC_NEXT_E   = 4'd12;
	localparam SC_DONE     = 4'd13;
	localparam SC_NEXT_E_FAST = 4'd14;   // OPT-B: 1-ciclo wait latency dopo pre-emit addr w0

	reg [3:0] sc_state;
	reg [8:0] entry_idx;       // 0..511
	reg [8:0] clear_idx;       // 0..319 per clear buffer
	reg [15:0] sp_w0, sp_w1, sp_w2, sp_w3;
	reg        pf_side;        // 0=metà sx tile (col 0..7), 1=metà dx (col 8..15)

	// Decoded fields - MAME RAIDEN layout (raiden.cpp:298-316):
	//   w0: [15]=active, [14]=flipy, [13]=flipx, [11:8]=color(4-bit), [7:0]=Y(8-bit unsigned)
	//   w1: [11:0]=code(12-bit)
	//   w2: [15:14]=priority(2-bit, 0=SKIP), [8:0]=X(signed 9-bit)
	//   w3: unused
	// Sprite size: SEMPRE 16×16 (1 tile, no multi-tile)
	wire        sp_enable = sp_w0[15];
	// flip_screen (DIP) inverte flipx/flipy per-sprite (raiden.cpp:353-354).
	wire        sp_flipy  = sp_w0[14] ^ flip_screen;  // INVERTITO vs sei021x generic
	wire        sp_flipx  = sp_w0[13] ^ flip_screen;
	wire  [3:0] sp_color  = sp_w0[11:8];      // 4-bit color (NON 6)
	wire  [7:0] sp_yraw   = sp_w0[7:0];       // Y in word 0, unsigned 8-bit
	wire [11:0] sp_code   = sp_w1[11:0];      // 12-bit code (NON 14)
	wire  [1:0] sp_pri    = sp_w2[15:14];     // priority in word 2 (NON word 1)
	wire  [8:0] sp_xraw   = sp_w2[8:0];       // X signed 9-bit in word 2

	// X signed 9-bit + xoff. Display 256 wide.
	// flip_screen (DIP): x = 240 - x (raiden.cpp:351). xoff additivo DOPO il flip.
	wire signed [10:0] sp_x_raw = sp_xraw[8] ? ({2'b11, sp_xraw}) : {2'b00, sp_xraw};
	wire signed [10:0] sp_x_f   = flip_screen ? (11'sd240 - sp_x_raw) : sp_x_raw;
	wire signed [10:0] sp_x = sp_x_f + {{1{xoff[9]}}, xoff};
	// Y unsigned 8-bit + yoff (hardcoded -16 nel top per match HW Raiden).
	// flip_screen: y = 240 - y (raiden.cpp:352). yoff additivo DOPO il flip.
	wire signed [10:0] sp_y_raw = {3'b000, sp_yraw};
	wire signed [10:0] sp_y_f   = flip_screen ? (11'sd240 - sp_y_raw) : sp_y_raw;
	wire signed [10:0] sp_y = sp_y_f + {{1{yoff[9]}}, yoff};

	// Sprite SEMPRE 16×16
	wire [3:0] sp_w  = 4'd1;
	wire [3:0] sp_h  = 4'd1;

	// Target Y per la linea che stiamo prefetchando.
	// v112 fix sprite scanline 0:
	// Prefetch riga 0 spostato da vpos=223 (PRE-copy spriteram) a vpos=225
	// (POST-copy spriteram a vpos=254 timing ≈ vpos_logic 224 + 0.2 linee).
	// A vpos=225 la copy spriteram (vblank_rising) ha già aggiornato buf2 al
	// frame N-1. Riga 0 prefetched legge buf2=N-1, allineato con righe 1..223.
	// Margine prefetch fino a display vpos=30 next frame = 35 linee >> ok.
	wire [8:0] target_y = (vpos == 9'd225) ? 9'd0 : (vpos + 9'd1);

	// Sprite intersect check: target_y in [sp_y, sp_y + sp_h*16)
	wire signed [10:0] dy_top = {2'b00, target_y} - sp_y;
	wire        in_y     = (dy_top >= 0) && (dy_top < {3'd0, sp_h, 4'd0});  // sp_h*16
	wire  [3:0] tile_y_in = dy_top[7:4];   // tile row 0..7 dentro sprite
	wire  [3:0] row_in    = dy_top[3:0];   // row dentro tile 0..15
	// flip Y
	wire  [3:0] eff_tile_y = sp_flipy ? (sp_h - 4'd1 - tile_y_in) : tile_y_in;
	wire  [3:0] eff_row    = sp_flipy ? (4'd15 - row_in)          : row_in;

	// Iteratore tile_x
	reg  [3:0] tile_x_pf;
	reg [31:0] pf_rom_data;
	reg  [3:0] decode_step;
	wire  [3:0] eff_tile_x = sp_flipx ? (sp_w - 4'd1 - tile_x_pf) : tile_x_pf;

	// Sprite size 16×16 fisso → cur_tile = sp_code (no multi-tile arithmetic)
	wire [11:0] cur_tile = sp_code;

	// new_line gating
	// v112: accetta anche vpos=225 per prefetch riga 0 dentro VBlank (post-copy).
	wire vpos_visible = (vpos < 9'd224);
	wire vpos_row0_vbl = (vpos == 9'd225);
	wire gated_new_line = new_line & (vpos_visible | vpos_row0_vbl);

	always @(posedge clk) begin
		if (reset) begin
			sc_state    <= SC_IDLE;
			entry_idx   <= 9'd0;
			tile_x_pf   <= 4'd0;
			rom_req     <= 1'b0;
			spr_addr    <= 11'd0;
			active_buf  <= 1'b0;
			decode_step <= 4'd0;
			clear_idx   <= 9'd0;
		end else begin
			case (sc_state)
				SC_IDLE: begin
					if (gated_new_line) begin
						active_buf <= ~active_buf;
						// Scan 511 → 0 (4KB spriteram, 512 entry × 4 word).
						// FIRST-WIN: entry 0 disegnato per ultimo = sopra le altre.
						// Inverte la priorità tra sprite rispetto al default MAME.
						entry_idx  <= 9'd511;
						tile_x_pf  <= 4'd0;
						clear_idx  <= 9'd0;
						sc_state   <= SC_CLEAR;
					end
				end

				// CLEAR: azzera 320 entry del buffer non-attivo a LB_EMPTY (pen=15
				// trasparente) per evitare ghost da scanline precedente.
				// Ottimizzazione (OPT-D): a clear_idx=318 pre-emit spr_addr w0 della
				// prima entry (= entry 511 scan decrescente). Quando arriviamo a
				// clear_idx=319, addr w0 è già in volo BRAM (1 ciclo latency) →
				// transizione diretta a SC_RW1 (sp_w0 valido), senza SC_RW0 wait.
				SC_CLEAR: begin
					if (active_buf == 1'b0)
						linebuf1[clear_idx] <= LB_EMPTY;
					else
						linebuf0[clear_idx] <= LB_EMPTY;
					if (clear_idx == 9'd318) begin
						spr_addr  <= {entry_idx, 2'd0};   // pre-emit w0 1 ciclo prima
						clear_idx <= clear_idx + 9'd1;
					end else if (clear_idx == 9'd319) begin
						clear_idx <= 9'd0;
						spr_addr  <= {entry_idx, 2'd1};   // emit w1 → dato w1 in RW2
						sc_state  <= SC_RW1;              // skip SC_RW0
					end else begin
						clear_idx <= clear_idx + 9'd1;
					end
				end

				// SC_RW0 ELIMINATA (OPT-D): integrata in pre-fetch da SC_CLEAR/NEXT_E.

				SC_RW1: begin
					sp_w0    <= spr_data;            // w0 valido (addr emesso 1 ciclo fa)
					spr_addr <= {entry_idx, 2'd2};
					sc_state <= SC_RW2;
				end
				SC_RW2: begin
					sp_w1    <= spr_data;
					spr_addr <= {entry_idx, 2'd3};
					sc_state <= SC_RW3;
				end
				SC_RW3: begin
					sp_w2    <= spr_data;
					sc_state <= SC_CHECK;
				end

				// SC_CHECK (OPT-A): decisione in QUESTO ciclo. sp_w2 latched (NON
				// blocking) a fine RW3 → in SC_CHECK è NEW. sp_w3 NON USATO →
				// commento obsoleto eliminato. SC_CHECK2 RIMOSSO.
				// (OPT-B): se scartata, transition diretta SC_RW1 con pre-emit addr
				// nuova entry → evita SC_NEXT_E (-1 ciclo) e SC_RW0 (-1 ciclo).
				SC_CHECK: begin
					if (sp_enable && in_y && layer_en && (sp_pri != 2'd0)) begin
						tile_x_pf <= 4'd0;
						pf_side   <= 1'b0;
						sc_state  <= SC_ROM_REQ;
					end else begin
						// Scartata: decrementa entry e pre-emit addr w0 della prossima
						if (entry_idx == 9'd0) begin
							sc_state <= SC_DONE;
						end else begin
							entry_idx <= entry_idx - 9'd1;
							spr_addr  <= {entry_idx - 9'd1, 2'd0};   // pre-emit w0 next
							sc_state  <= SC_NEXT_E_FAST;             // 1 ciclo wait latency
						end
					end
				end

				// SC_NEXT_E_FAST (OPT-B helper): 1 ciclo wait per latency BRAM addr w0,
				// poi emit w1 e transition SC_RW1. Risparmio vs SC_NEXT_E + SC_RW0
				// = 1 ciclo per ogni entry scartata.
				SC_NEXT_E_FAST: begin
					spr_addr <= {entry_idx, 2'd1};   // emit w1 (entry_idx già decrementato)
					sc_state <= SC_RW1;
				end

				SC_ROM_REQ: begin
					// Sprite tile addr in SDRAM region (byte_offset relativo):
					// tile_code * 128 (32 word) + (pf_side ? 64 byte : 0) + eff_row * 4
					rom_addr <= ({4'd0, cur_tile, 7'd0})        // tile*128
					           + (pf_side ? 24'd64 : 24'd0)     // metà dx tile
					           + ({18'd0, eff_row, 2'd0});       // row*4
					rom_req  <= 1'b1;
					sc_state <= SC_ROM_W;
				end

				SC_ROM_W: begin
					if (rom_valid) begin
						pf_rom_data <= rom_data;
						rom_req     <= 1'b0;
						decode_step <= 4'd0;
						sc_state    <= SC_DECODE;
					end
				end

				SC_DECODE: begin
					begin : sp_decode_blk
						reg [31:0] pf_data_eff;
						reg [7:0] byte_a, byte_b;
						reg [1:0] sub;
						reg [2:0] bit_lo, bit_hi;
						reg [3:0] pen;
						reg signed [10:0] dx;
						reg [4:0] eff_col;

						// Mode selection (OSD-driven decode_mode, 32 modalità):
						//   bit 0: byte mapping a↔b
						//   bit 1: sub direction (sub vs 3-sub)
						//   bit 2: bit position (3-sub/7-sub vs sub/4+sub)
						//   bit 3: pf_rom_data byte-reverse (= rotazione 32-bit)
						//   bit 4: nibble-swap dentro ogni byte (HI↔LO 4-bit)

						// bit 3: byte-reverse 32-bit
						pf_data_eff = decode_mode[3] ?
						              {pf_rom_data[7:0], pf_rom_data[15:8], pf_rom_data[23:16], pf_rom_data[31:24]} :
						              pf_rom_data;

						// bit 4: nibble-swap per byte (per ognuno dei 4 byte, swap nibble HI ↔ LO)
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

						// Posizione pixel sullo schermo: sp_x + tile_x_pf*16 + col_in_tile
						// col_in_tile = pf_side*8 + decode_step[2:0]
						// flipX: col_in_tile = 15 - col_in_tile
						eff_col = sp_flipx ? (5'd15 - {pf_side, decode_step[2:0]})
						                   : {pf_side, decode_step[2:0]};
						dx = sp_x + ({6'd0, tile_x_pf, 4'd0}) + {6'd0, eff_col};

						if (pen != 4'd15 && dx >= 0 && dx < 256) begin
							if (active_buf == 1'b0)
								linebuf1[dx[8:0]] <= {sp_color, sp_pri, pen};
							else
								linebuf0[dx[8:0]] <= {sp_color, sp_pri, pen};
						end
					end
					if (decode_step == 4'd7) sc_state <= SC_NEXT_TX;
					else                     decode_step <= decode_step + 4'd1;
				end

				SC_NEXT_TX: begin
					if (pf_side == 1'b0) begin
						// Appena finito metà sx → fai metà dx dello stesso tile
						pf_side  <= 1'b1;
						sc_state <= SC_ROM_REQ;
					end else begin
						// Finito anche metà dx → passa al prossimo tile_x o entry
						pf_side <= 1'b0;
						if (tile_x_pf == sp_w - 4'd1) begin
							sc_state <= SC_NEXT_E;
						end else begin
							tile_x_pf <= tile_x_pf + 4'd1;
							sc_state  <= SC_ROM_REQ;
						end
					end
				end

				SC_NEXT_E: begin
					// Scan decrescente: 511 → 0 (first-win priority).
					if (entry_idx == 9'd0) begin
						sc_state <= SC_DONE;
					end else begin
						entry_idx <= entry_idx - 9'd1;
						spr_addr  <= {entry_idx - 9'd1, 2'd0};
						sc_state  <= SC_NEXT_E_FAST;   // OPT-D: skip SC_RW0
					end
				end

				SC_DONE: begin
					if (gated_new_line) begin
						active_buf <= ~active_buf;
						entry_idx  <= 9'd511;
						tile_x_pf  <= 4'd0;
						clear_idx  <= 9'd0;
						sc_state   <= SC_CLEAR;
					end
				end

				default: sc_state <= SC_IDLE;
			endcase

			// Clear line buffer non-attivo all'inizio del nuovo new_line
			// (in realtà serve clear durante DONE → ma qui non si può scrivere
			// 320 entry per ce_pix; per semplicità il DONE dovrebbe blankarlo
			// — lascio come TODO, sprite "ghost" possibili nelle linee dove
			// nessun sprite scrive)
		end
	end

	// ─── Read side (registrata M10K + lookahead 1 = latenza netta 0) ─────────
	// Legge hpos+1: dato registrato per hpos pronto al display. Bordo pixel-0 primato
	// dai 48 cicli hblank; active_buf stabile nel visibile. Behavior-preserving.
	// Layout linebuf: [9:6]=color (4-bit), [5:4]=pri, [3:0]=pen
	wire [8:0] rd_addr = hpos[8:0] + 9'd1;
	reg [9:0] lb0_q, lb1_q;
	always @(posedge clk) if (ce_pix) begin
		lb0_q <= linebuf0[rd_addr];
		lb1_q <= linebuf1[rd_addr];
	end
	wire  [9:0] read_data = active_buf ? lb1_q : lb0_q;
	wire  [3:0] read_pen   = read_data[3:0];
	wire  [1:0] read_pri   = read_data[5:4];
	wire  [3:0] read_color = read_data[9:6];

	// Output combinatoriale (no latency). priority=0 skip (MAME raiden.cpp:322-324).
	wire pixel_active = de & layer_en & (hpos < 10'd256) & (read_pen != 4'd15) & (read_pri != 2'd0);
	assign opaque    = pixel_active;
	assign pen_index = pixel_active ? (11'h200 + {3'd0, read_color, read_pen}) : 11'd0;
	assign pri_code  = pixel_active ? read_pri : 2'd0;

`ifdef V30_SIM_PROBES
// Probe taglio sprite: righe in cui lo scan NON completa la lista entry
// prima del new_line (= sprite successivi non disegnati su quella riga).
integer spr_cut_n = 0;
always @(posedge clk) begin
	if (new_line && sc_state != SC_DONE && sc_state != SC_IDLE && spr_cut_n < 60) begin
		spr_cut_n <= spr_cut_n + 1;
		$display("[sprcut] vpos=%0d scan INCOMPLETO: state=%0d entry=%0d", vpos, sc_state, entry_idx);
	end
end

// Probe PRIORITA': dump entry attive su UNA riga scelta (vpos==dbg_row), con
// pri grezza da w2. Distingue "pri=0 dal gioco" da "pri corrotta dal buffer".
integer spr_dump_n = 0;
reg [8:0] dbg_row = 9'd120;
always @(posedge clk) begin
	if (sc_state == SC_CHECK && vpos == dbg_row && sp_enable && spr_dump_n < 40) begin
		spr_dump_n <= spr_dump_n + 1;
		$display("[sprpri] e%0d pri=%0d y=%0d x=%0d code=%0h col=%0d w0=%04h w2=%04h",
		         entry_idx, sp_w2[15:14], sp_w0[7:0], sp_w2[8:0], sp_w1[11:0], sp_w0[11:8], sp_w0, sp_w2);
	end
end
`endif

endmodule
