// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden shared RAM Main↔Sub — 2 BRAM SPECULARI, no race, no stall.
//
// 4KB logici (2K word) condivisi. Implementati come 2 BRAM coerenti tra loro:
//   - mem_main_*: vista letta da Main
//   - mem_sub_* : vista letta da Sub
//
// Ogni write viene applicato a ENTRAMBE le BRAM per mantenere coerenza.
// Conflict scrittura simultanea sullo stesso byte: SERIALIZZATA (Main subito,
// Sub in pend register committata al ciclo libero successivo — come il PCB
// che arbitra il bus: nessuna write persa).
//
// Letture: ogni CPU legge dalla SUA BRAM, indirizzata col proprio addr.
//   - Latency 1 ciclo (BRAM registered output) uguale alle altre BRAM CPU.
//   - Nessun busy, nessun stall, nessun mux output.
//
// MAME memory map:
//   Main: $08000-$08FFF
//   Sub : $04000-$04FFF

module raiden_shared_ram #(parameter SS_IDX = -1)
(
	input  wire        clk,
	input  wire        reset,

	// Main port (16-bit)
	input  wire [11:1] main_addr,
	input  wire        main_cs,
	input  wire [15:0] main_din,
	output wire [15:0] main_dout,
	input  wire  [1:0] main_we,

	// Sub port (16-bit)
	input  wire [11:1] sub_addr,
	input  wire        sub_cs,
	input  wire [15:0] sub_din,
	output wire [15:0] sub_dout,
	input  wire  [1:0] sub_we,

	// Savestate slave (le due BRAM sono speculari: salvo mem_main, restore su entrambe)
	ssbus_if.slave     ss_shared
);

// Conflict scrittura Main/Sub stesso byte: SERIALIZZATA, non persa.
// Prima: "Main wins" = write Sub DROPPATA. Con i CE phase-locked la finestra
// di write Sub poteva coincidere del tutto con quella Main -> perdita totale
// (il PCB reale serializza entrambe). Ora la write Sub soppressa viene tenuta
// in un registro pend (1 entry per byte-lane) e committata appena la porta
// e' libera (priorita': ss > main > pend > sub). Ordine main->sub preservato.
wire main_wr_lo = main_cs & main_we[0];
wire main_wr_hi = main_cs & main_we[1];
wire sub_want_lo = sub_cs & sub_we[0];
wire sub_want_hi = sub_cs & sub_we[1];
// PORTA A OCCUPATA: ss e main hanno priorita' nella catena di write della BRAM.
// Prima qui c'era `collide` = solo stesso INDIRIZZO: con indirizzo diverso il
// Sub risultava "scritto" ma la catena eseguiva il Main e la write spariva
// (nessun busy, nessuno stall). La condizione giusta e' la PORTA occupata.
wire busy_lo = ss_wr | main_wr_lo;
wire busy_hi = ss_wr | main_wr_hi;

// CODA delle write differite del Sub. Una sola entrata veniva sovrascritta
// prima del commit: la write piu' vecchia spariva. Profondita' 8 = piu' del
// burst massimo osservabile (il Main non tiene la porta piu' di pochi clock).
localparam integer PQD = 8;
reg  [11:1] pq_lo_addr [0:PQD-1], pq_hi_addr [0:PQD-1];
reg  [7:0]  pq_lo_data [0:PQD-1], pq_hi_data [0:PQD-1];
reg  [3:0]  pq_lo_wp, pq_lo_rp, pq_hi_wp, pq_hi_rp;
wire pend_lo = (pq_lo_wp != pq_lo_rp);
wire pend_hi = (pq_hi_wp != pq_hi_rp);
wire full_lo = ((pq_lo_wp - pq_lo_rp) == PQD[3:0]);
wire full_hi = ((pq_hi_wp - pq_hi_rp) == PQD[3:0]);

// drain: la porta e' libera e c'e' qualcosa in coda
wire drain_lo = pend_lo & ~busy_lo;
wire drain_hi = pend_hi & ~busy_hi;
// write Sub live: porta libera, coda VUOTA (altrimenti si scavalcherebbe
// una write piu' vecchia: l'ordine main->sub deve restare)
wire sub_wr_lo  = sub_want_lo & ~busy_lo & ~pend_lo;
wire sub_wr_hi  = sub_want_hi & ~busy_hi & ~pend_hi;
// accodamento: tutto cio' che non e' potuto andare live
wire enq_lo = sub_want_lo & ~sub_wr_lo & ~full_lo;
wire enq_hi = sub_want_hi & ~sub_wr_hi & ~full_hi;

always @(posedge clk) begin
	if (reset) begin
		pq_lo_wp <= 0; pq_lo_rp <= 0;
		pq_hi_wp <= 0; pq_hi_rp <= 0;
	end else begin
		if (enq_lo) begin
			pq_lo_addr[pq_lo_wp[2:0]] <= sub_addr;
			pq_lo_data[pq_lo_wp[2:0]] <= sub_din[7:0];
			pq_lo_wp <= pq_lo_wp + 4'd1;
		end
		if (drain_lo) pq_lo_rp <= pq_lo_rp + 4'd1;

		if (enq_hi) begin
			pq_hi_addr[pq_hi_wp[2:0]] <= sub_addr;
			pq_hi_data[pq_hi_wp[2:0]] <= sub_din[15:8];
			pq_hi_wp <= pq_hi_wp + 4'd1;
		end
		if (drain_hi) pq_hi_rp <= pq_hi_rp + 4'd1;
	end
end

// ── BRAM "mem_main_*" : vista Main ────────────────────────────────────────
// Porta A: write (Main e Sub mux). Porta B: read Main.
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_main_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_main_hi [0:2047];
initial begin
	integer i;
	for (i=0; i<2048; i=i+1) begin mem_main_lo[i]=0; mem_main_hi[i]=0; end
end

// ── Savestate adaptor (write su ENTRAMBE le BRAM, read da mem_main) ──────
// A SS idle: trasparente (write main/sub, read normale). Durante SS: write ssbus
// su idx ssbus, read mem_main[idx] → q_in. Le BRAM restano speculari.
wire        ss_sel = ss_shared.access(SS_IDX);
reg  [15:0] ss_shared_rdata;
wire [10:0] ss_idx  = ss_shared.addr[10:0];
wire        ss_wr   = ss_sel & ss_shared.write;

reg  read_delay_sh;
always @(posedge clk) begin
	ss_shared.setup(SS_IDX, 32'd2048, 1);   // 2048 word, 16 bit
	if (ss_sel) begin
		if (ss_shared.write)     ss_shared.write_ack(SS_IDX);
		else if (ss_shared.read) begin
			if (read_delay_sh) ss_shared.read_response(SS_IDX, {48'd0, ss_shared_rdata});
			read_delay_sh <= 1;
		end
	end else read_delay_sh <= 0;
end

reg [7:0] dout_main_lo_r, dout_main_hi_r;
always @(posedge clk) begin
	if (ss_wr)      mem_main_lo[ss_idx] <= ss_shared.data[7:0];
	else if (main_wr_lo) mem_main_lo[main_addr] <= main_din[7:0];
	else if (drain_lo)   mem_main_lo[pq_lo_addr[pq_lo_rp[2:0]]] <= pq_lo_data[pq_lo_rp[2:0]];
	else if (sub_wr_lo)  mem_main_lo[sub_addr]  <= sub_din[7:0];
	dout_main_lo_r  <= mem_main_lo[main_addr];
	ss_shared_rdata[7:0] <= mem_main_lo[ss_idx];

	if (ss_wr)      mem_main_hi[ss_idx] <= ss_shared.data[15:8];
	else if (main_wr_hi) mem_main_hi[main_addr] <= main_din[15:8];
	else if (drain_hi)   mem_main_hi[pq_hi_addr[pq_hi_rp[2:0]]] <= pq_hi_data[pq_hi_rp[2:0]];
	else if (sub_wr_hi)  mem_main_hi[sub_addr]  <= sub_din[15:8];
	dout_main_hi_r  <= mem_main_hi[main_addr];
	ss_shared_rdata[15:8] <= mem_main_hi[ss_idx];
end

// ── BRAM "mem_sub_*" : vista Sub ──────────────────────────────────────────
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_sub_lo [0:2047];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] mem_sub_hi [0:2047];
initial begin
	integer i;
	for (i=0; i<2048; i=i+1) begin mem_sub_lo[i]=0; mem_sub_hi[i]=0; end
end

reg [7:0] dout_sub_lo_r, dout_sub_hi_r;
always @(posedge clk) begin
	if (ss_wr)      mem_sub_lo[ss_idx] <= ss_shared.data[7:0];   // restore: speculare
	else if (main_wr_lo) mem_sub_lo[main_addr] <= main_din[7:0];
	else if (drain_lo)   mem_sub_lo[pq_lo_addr[pq_lo_rp[2:0]]] <= pq_lo_data[pq_lo_rp[2:0]];
	else if (sub_wr_lo)  mem_sub_lo[sub_addr]  <= sub_din[7:0];
	dout_sub_lo_r <= mem_sub_lo[sub_addr];

	if (ss_wr)      mem_sub_hi[ss_idx] <= ss_shared.data[15:8];
	else if (main_wr_hi) mem_sub_hi[main_addr] <= main_din[15:8];
	else if (drain_hi)   mem_sub_hi[pq_hi_addr[pq_hi_rp[2:0]]] <= pq_hi_data[pq_hi_rp[2:0]];
	else if (sub_wr_hi)  mem_sub_hi[sub_addr]  <= sub_din[15:8];
	dout_sub_hi_r <= mem_sub_hi[sub_addr];
end

`ifdef V30_SIM_PROBES
// CONTATORE WRITE DEL SUB PERSE: sub_wr_* alto insieme a main_wr_* significa
// che la catena di priorita' nella BRAM esegue la write del MAIN e quella del
// SUB non viene ne' eseguita ne' messa in pend (il pend si arma solo su
// collisione di indirizzo o su drain). La write sparisce.
integer sh_lost = 0, sh_clk = 0, sh_subw = 0, sh_mainw = 0;
always @(posedge clk) begin
	sh_clk <= sh_clk + 1;
	if (sub_wr_lo | sub_wr_hi)   sh_subw  <= sh_subw + 1;
	if (main_wr_lo | main_wr_hi) sh_mainw <= sh_mainw + 1;
	if ((sub_wr_lo & main_wr_lo) | (sub_wr_hi & main_wr_hi)) sh_lost <= sh_lost + 1;
	if (sh_clk % 1346570 == 0 && sh_clk != 0) begin
		$display("[shared] frame: write sub %0d, write main %0d, PERSE %0d", sh_subw, sh_mainw, sh_lost);
		sh_lost <= 0; sh_subw <= 0; sh_mainw <= 0;
	end
end
`endif

assign main_dout = {dout_main_hi_r, dout_main_lo_r};
assign sub_dout  = {dout_sub_hi_r,  dout_sub_lo_r};

endmodule
