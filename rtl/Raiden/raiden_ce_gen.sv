// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden V30 CE generator — divisore intero selezionabile via OSD.
//
// clk_sys = 80 MHz. V30 spec MAME = 10 MHz esatti.
//
// clk_sel[2:0] (da OSD P2O[22:20]):
//   0=10MHz → div 8  (10.0 MHz esatti)  ← spec MAME
//   1=8MHz  → div 10 (8.0 MHz)
//   2=12MHz → div 6  (13.3 MHz)
//   3=16MHz → div 5  (16.0 MHz)
//   4=24MHz → div 4  (20.0 MHz)
//   5=32MHz → div 2  (26.7 MHz)
//
// ce_4x = OGNI clk_sys (pattern R-Type: ce_4x è il clock interno della CPU,
// non un divisore). cpu.vhd richiede ce_4x ogni clk per il microcode/prefetch.
//
// Stall: durante accesso SDRAM in volo (ls245_en o mem_rq_active) il counter
// non avanza. La CPU resta congelata fino a che il dato arriva.

module raiden_ce_gen
#(
	// Fase iniziale del divisore. Main=0, Sub=4: sfasa i ce dei due V30 di
	// mezzo periodo (4 clk a div 8) -> le richieste ROM dei due processori
	// non cadono MAI nello stesso slot SDRAM -> con prio_mode 2 ogni CPU
	// attende al peggio il residuo di UNA transazione (<=7 clk) + la propria
	// (7) = sempre 0 wait. (Phase-locked: il secondo pagava +8 = 1 Tw.)
	parameter PHASE_INIT = 4'd0
)
(
	input  wire       clk,
	input  wire       reset,
	input  wire       pause,
	input  wire [2:0] clk_sel,
	input  wire       ls245_en,
	input  wire       rd_lat,         // read-latch del FSM ROM (cpu_rd ritardato 1 clk)
	input  wire       mem_rq_active,
	output reg        ce,
	output reg        ce_4x
);

reg [3:0] ce_cnt;
reg [3:0] ce_div;

always @(*) begin
	case (clk_sel)
		3'd0: ce_div = 4'd7;   // 10 MHz spec MAME (80/8)
		3'd1: ce_div = 4'd9;   // 8 MHz  (80/10)
		3'd2: ce_div = 4'd5;   // 13.3 MHz (80/6)
		3'd3: ce_div = 4'd4;   // 16 MHz (80/5)
		3'd4: ce_div = 4'd3;   // 20 MHz (80/4)
		3'd5: ce_div = 4'd1;   // 26.7 MHz (80/2)
		default: ce_div = 4'd7;
	endcase
end

// CE-STALL RIPRISTINATO (via libera utente 2026-08-13). Il READY/Tw di
// 0e86043 e' INCOMPATIBILE con l'ucore: il suo BIU campiona il DATO a T2
// (F57 in v30u_biu: "cur_data = ad_i at T2"; READY ritarda solo il
// completamento). Con la SDRAM reale il dato arriva DOPO T2 -> in coda
// entrano byte 0000 -> istruzioni fantasma (RMW@0 visto ai pin in sim 1:1)
// -> boot deragliato = schermo nero HW. Il riferimento M72 dell'ucore fa
// uguale: "an SDRAM stall defers CPU cycles" (CE-stall, mai Tw reali).
wire stall    = mem_rq_active | (ls245_en & ~rd_lat);
wire allow_ce = ~pause & ~stall;

// CATCH-UP "chasing counter" (pattern M72 upstream, docs/PLAN_catchup_ce.md).
// Il CE-stall DIFFERISCE i cicli, non li perde: il riferimento continua a
// maturare crediti a rate nominale e appena lo stall finisce il core li
// recupera a raffica (1 CE ogni CE_GAP_MIN clk). Cosi' la CPU mantiene la
// media di 10 MHz anche sotto miss SDRAM -> niente rallentamenti (che
// desincronizzavano main<->sub: punteggio corrotto, buffer sfasato) e la
// fase main<->sub si riaggancia dopo ogni stall invece di derivare.
// CE_GAP_MIN=5 tiene VERE tutte le multicycle CE gia' in Template.sdc
// (ce->ce 4/3, ce->ce_half 2/1, ce_half->ce 3/2).
// 5 e non 4: con CE_HALF a +2 clk l'arco ce_half->prossimo ce ha
// (GAP-2) periodi; a GAP=5 sono 3 = quanto dichiarato in Template.sdc.
localparam [2:0] CE_GAP_MIN = 3'd5;
// Tetto ai crediti: 8 = 2 cicli di bus. Uno stall reale dura quanto un accesso
// SDRAM (~8-16 clk = 1-2 T-state), quindi 8 copre TUTTI i casi normali; serve
// solo a impedire raffiche lunghe in contese patologiche, che allineerebbero le
// richieste SDRAM di main e sub rompendo lo sfasamento di 4 clk (2be080b).
localparam [5:0] CE_CREDIT_MAX = 6'd8;
reg [5:0] ce_credit;   // cicli differiti da recuperare (satura a CE_CREDIT_MAX)
reg [2:0] ce_gap;      // clk dall'ultimo CE emesso (satura a 7)

// earn : il riferimento a rate nominale matura un ciclo (matura ANCHE in stall)
// spend: il ciclo puo' partire (bus libero + distanza minima rispettata).
//        Il credito maturato ORA e' spendibile nello stesso clk -> senza stall
//        il comportamento e' IDENTICO al divisore semplice di prima.
wire earn  = ~pause & (ce_cnt >= ce_div);
wire spend = ~pause & ~stall & (ce_gap >= CE_GAP_MIN) & ((ce_credit != 6'd0) | earn);

always @(posedge clk) begin
	if (reset) begin
		ce_cnt    <= PHASE_INIT;
		ce        <= 1'b0;
		ce_4x     <= 1'b0;
		ce_credit <= 6'd0;
		ce_gap    <= CE_GAP_MIN;
	end else begin
		ce    <= 1'b0;
		ce_4x <= ~pause;              // invariato: ogni clk fuori pausa

		// riferimento nominale (non si ferma durante lo stall: i cicli sono
		// DIFFERITI, non persi)
		if (~pause) ce_cnt <= (ce_cnt >= ce_div) ? 4'd0 : (ce_cnt + 4'd1);

		// distanza dall'ultimo CE
		if (ce_gap != 3'd7) ce_gap <= ce_gap + 3'd1;

		// emissione + contabilita' crediti
		if (spend) begin
			ce     <= 1'b1;
			ce_gap <= 3'd0;
		end
		case ({earn, spend})
			2'b10: if (ce_credit != CE_CREDIT_MAX) ce_credit <= ce_credit + 6'd1;
			2'b01: if (ce_credit != 6'd0)  ce_credit <= ce_credit - 6'd1;
			default: ;   // 00 = fermo, 11 = maturato e speso subito
		endcase
	end
end

`ifdef V30_SIM_PROBES
// Velocita' CPU PER FRAME (la media nasconde i cali brevi): conta i cicli
// nominali e quelli emessi in ogni finestra da ~1 frame e stampa il PEGGIORE.
integer nf = 0, ef = 0, wf = 100, cnt = 0, fr = 0;
always @(posedge clk) begin
	if (earn)  nf <= nf + 1;
	if (spend) ef <= ef + 1;
	cnt <= cnt + 1;
	// spia: se la CPU non gira, dire SUBITO perche' (pause o stall perenne)
	if (cnt % 500000 == 0 && cnt != 0)
		$display("[cegen %m] clk%0d pause=%b stall=%b ce_emessi=%0d", cnt, pause, stall, ef);
	if (cnt == 32'd1350000) begin      // ~1 frame a 80 MHz
		cnt <= 0; nf <= 0; ef <= 0; fr <= fr + 1;
		if (nf > 100) begin
			if ((ef*100)/nf < wf) wf <= (ef*100)/nf;
			$display("[cefr %m] frame %0d: %0d%% (peggiore finora %0d%%)", fr, (ef*100)/nf, wf);
		end
	end
end
`endif

endmodule
