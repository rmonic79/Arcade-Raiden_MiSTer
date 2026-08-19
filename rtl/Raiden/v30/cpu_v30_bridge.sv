// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
//============================================================================
// cpu_v30_bridge — wrapper attorno al core V30 cycle-accurate (wickerwaka,
// da Arcade-IremM72) tramite l'adapter `v30_bus` (bus muxato max-mode →
// bus lean word-aligned). Sostituisce il vecchio core VHDL WonderSwan cpu.vhd.
//
// La PORT-LIST ESTERNA resta identica: i due top (main/sub) istanziano il
// bridge senza modifiche.
//
// Clocking:
//   * ce      = avanzamento T-state (10 MHz = 1 pulse ogni 8 clk_sys).
//   * ce_half = ce ritardato 1 fabric clock (latch indirizzo T1, come m72.v:
//               count[0] alterna CE/CE_HALF → CE_HALF è l'emit successivo, +1 clk;
//               con la ce /8 fissa di Raiden il ritardo registrato è esatto).
//   * ce_4x   = NON più usato (il core nuovo non ha microcode-tick esterno).
//   * READY tied high: nessun Tw. Gli stall ROM/SDRAM restano gestiti a monte
//     dal CE-freeze di raiden_ce_gen (che toglie ce → e quindi ce_half al clk dopo).
//
// Byte lane M72-native: cpu_be[0]=lane bassa (A0==0), cpu_be[1]=lane alta
//   (~UBE_N); cpu_dout/cpu_din sono già sulla lane corretta → i moduli
//   downstream usano cpu_be[0]/[1] diretti e NON applicano byte_align.
//
// IRQ: il core nuovo vuole il NUMERO di vettore (lo moltiplica ×4 internamente).
//   irq_vector[9:0] è l'indirizzo byte IVT (0x0C8) → numero = >>2 = 0x32.
//   int_req è un livello; il top azzera irq_pending quando vede int_ack
//   (esposto qui su cpu_irqrequest, invariato lato top).
//
// Savestate: regfile nativo 202-entry del core, esposto via v30_bus.ssbus,
//   legato direttamente alla porta ss del sistema (SS_IDX). ss_cpu_reload →
//   ss_restore_done (riporta l'adapter a bus-idle a load completato).
//============================================================================

module cpu_v30_bridge #(parameter SS_IDX = -1) (
	input  logic        clk,
	input  logic        ce,        // ~10 MHz: 1 ogni 8 clk (T-state advance)
	input  logic        ce_4x,     // legacy: non usato dal core nuovo
	input  logic        reset,
	input  logic        rom_wait,  // alto durante fetch ROM/SDRAM (main/sub_rq_active)

	// Bus dati/istruzioni unificato (V30 ha bus singolo)
	output logic [19:0] bus_addr,
	output logic        bus_read,
	output logic        bus_write,
	output logic  [1:0] bus_be,
	output logic [15:0] bus_dout,
	input  logic [15:0] bus_din,

	// Interrupt
	input  logic        irq_req,
	input  logic  [9:0] irq_vector,

	// Status / debug (i top lasciano halt/prefix scollegati)
	output logic        cpu_idle,
	output logic        cpu_halt,
	output logic        cpu_irqrequest,
	output logic        cpu_prefix,

	// Savestate: slave del bus savestate del sistema (regfile nativo del core)
	ssbus_if.slave      ss,
	// pulse post-load dal manager → riporta l'adapter bus a idle
	input  logic        ss_cpu_reload
);

	// ce_half = ce ritardato 1 clk. Se raiden_ce_gen toglie ce (stall ROM/SDRAM),
	// al clk successivo ce_half va a 0 → entrambi i fasi congelati insieme.
	// CE_HALF a +2 clk da CE (era +1). MOTIVO (misurato con STA 2026-08-14):
	// l'upstream genera CE_HALF a META' PERIODO CPU (nec_bus: tick_fall =
	// div_cnt == half-1), quindi il suo multicycle 2 sull'arco ce->ce_half e'
	// vero con margine. Con CE_HALF a +1 clk quel vincolo era FALSO da noi:
	// l'arco ha 1 solo periodo mentre l'SDC ne dichiarava 2 -> STA verde
	// (+1.184) e SILICIO CHE VIOLA di -11.316 ns. E' lo STESSO meccanismo del
	// bug punteggio storico (multicycle 9 falso, -11.03/-11.85 -> score x100).
	// A +2 clk l'arco ha davvero 2 periodi -> il vincolo diventa VERO e lo
	// slack misurato (+1.18) e' reale. CE_GAP_MIN=5 in raiden_ce_gen tiene
	// vero anche l'arco di ritorno ce_half->ce (>=3 periodi).
	reg ce_half_d1, ce_half;
	always @(posedge clk) begin
		ce_half_d1 <= ce;
		ce_half    <= ce_half_d1;
	end

	// READY/Tw per ROM: durante il fetch (rom_wait) READY basso → il BIU inserisce
	// Tw sul suo ciclo di bus ma l'EU CONTINUA dalla coda (overlap = V30 reale).
	// Lag conservativo 2 clk dopo la fine fetch: garantisce il dato ROM stabile su
	// AD (rdata_q, 1 clk) prima del campionamento del BIU (no race). ce non è più
	// congelato (raiden_ce_gen stall=0) → l'EU gira durante il wait.
	// READY FISSO ALTO (via libera utente 2026-08-13): i wait SDRAM sono
	// gestiti dal CE-stall in raiden_ce_gen (contratto ucore: il BIU campiona
	// il dato a T2, quindi il ciclo non deve AVANZARE finche' il dato non e'
	// pronto — F57 v30u_biu + report M72 "an SDRAM stall defers CPU cycles").
	// READY/Tw resta solo come meccanismo del rig upstream (wait artificiali
	// a dato gia' presente), mai per latenza reale. rom_wait qui non serve piu'.
	wire cpu_ready = 1'b1;

	// Strobe separati mem/io del core → bus unico (Raiden è tutto memory-mapped;
	// io_rd/io_wr non dovrebbero attivarsi, OR difensivo).
	wire mem_rd, io_rd, mem_wr, io_wr, code_fetch;
	assign bus_read  = mem_rd | io_rd;
	assign bus_write = mem_wr | io_wr;

	// INTA acknowledge del core → il top lo usa (come cpu_irqrequest) per azzerare
	// il proprio latch irq_pending.
	wire int_ack;
	assign cpu_irqrequest = int_ack;

	// cpu_idle = ss_quiet (BIU bus-quiet = punto sicuro di cattura SS), registrato.
	wire ss_quiet;
	reg  cpu_idle_r;
	always @(posedge clk) cpu_idle_r <= ss_quiet;
	assign cpu_idle   = cpu_idle_r;

	assign cpu_halt   = 1'b0;         // non esposto dal core nuovo; i top non lo usano
	assign cpu_prefix = code_fetch;   // prefetch marker; i top non lo usano

	v30_bus #(.SS_IDX(SS_IDX)) u_core (
		.clk            (clk),
		.ce             (ce),
		.ce_half        (ce_half),
		.reset          (reset),           // active high
		.ready          (cpu_ready),       // Tw durante fetch ROM (l'EU overlappa); 1 sulle region on-chip

		.cpu_addr       (bus_addr),        // bit0 già forzato 0 dal core
		.cpu_be         (bus_be),          // [0]=lane bassa (A0==0), [1]=lane alta (~UBE_N)
		.cpu_dout       (bus_dout),
		.cpu_din        (bus_din),

		.mem_rd         (mem_rd),
		.io_rd          (io_rd),
		.mem_wr         (mem_wr),
		.io_wr          (io_wr),
		.code_fetch     (code_fetch),

		.int_req        (irq_req),
		.int_vector     (irq_vector[9:2]), // 0x0C8 → 0x32 (numero vettore)
		.int_ack        (int_ack),

		.dbg_regs       (),                // solo sim (V30_BACKDOOR)

		.ssbus          (ss),
		.ss_restore_done(ss_cpu_reload),
		.ss_quiet       (ss_quiet)
	);

endmodule
