// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) Umberto Parisi (rmonic79). GPL v3 or later.
//
// Raiden_sub_top — Sub V30 + memory map M72-style.
// Refactor pattern Irem M72:
//  - raiden_addr_sub: address translator
//  - raiden_ce_gen:   CE generator (stall su mem_rq + ls245_en)
//  - raiden_video_subbus: BG/FG/Palette RAM CPU bus + renderer porte B
//  - Sub RAM 8KB BRAM interno
//  - shared RAM bridge: porte verso top
//  - DOUT_VALID mux per cpu_din

module Raiden_sub_top #(
	parameter SS_IDX_BG = -1,
	parameter SS_IDX_FG = -1,
	parameter SS_IDX_PAL = -1,
	parameter SS_IDX_CPU = -1,
	parameter SS_IDX_SUBRAM = -1
) (
	input  wire        clk,
	input  wire        reset,
	input  wire        pause,
	output wire        cpu_idle,        // 1 = V30 a confine istruzione (savestate boundary)
	input  wire  [2:0] clk_sel,         // legacy, ignorato
	// SDRAM Sub ROM bridge
	input  wire [15:0] sub_rom_rdata,
	input  wire        sub_rom_ready,
	output wire [23:0] sub_rom_addr,
	output wire        sub_rom_req,
	// VBLANK per IRQ
	input  wire        vblank_in,
	// Shared RAM bridge — Sub side (porte verso top: raiden_shared_ram)
	output wire [11:1] sub_shared_addr,
	output wire        sub_shared_cs,
	output wire  [1:0] sub_shared_we,
	output wire [15:0] sub_shared_wdata,
	input  wire [15:0] sub_shared_rdata,
	// Renderer porte B (BG/FG/Palette)
	input  wire [10:0] bg_vram_addr,
	output wire [15:0] bg_vram_data,
	input  wire [10:0] fg_vram_addr,
	output wire [15:0] fg_vram_data,
	input  wire [10:0] pal_vram_addr,
	output wire [15:0] pal_vram_data,
	// Probe
	output wire        dbg_irq_pending,

	// Palette overlay tap (Sub V30 → palette $03000-$03FFF) — tied-off, non usato
	output wire [19:0] dbg_cpu_addr,
	output wire [15:0] dbg_cpu_dout,
	output wire  [1:0] dbg_cpu_be,
	output wire        dbg_cpu_wr,
	output wire        dbg_palette_memrq,
	// Savestate slaves (propagati a raiden_video_subbus)
	ssbus_if.slave     ss_bg,
	ssbus_if.slave     ss_fg,
	ssbus_if.slave     ss_pal,
	ssbus_if.slave     ss_cpu,  // V30 sub regs (→ cpu_v30_bridge)
	input  wire        ss_cpu_reload, // reset CPU coordinato (post-load)
	ssbus_if.slave     ss_subram   // Sub work RAM 8KB — porta B dual-port
);

// ─── V30 CPU bus ────────────────────────────────────────────────────────
wire [19:0] cpu_addr;
wire        cpu_rd, cpu_wr;
wire  [1:0] cpu_be;
wire [15:0] cpu_dout;
reg  [15:0] cpu_din;

// ─── IRQ handler ────────────────────────────────────────────────────────
reg  vblank_d;
reg  irq_pending;
wire cpu_irq_active;
always @(posedge clk) begin
	if (reset) begin
		vblank_d    <= 1'b0;
		irq_pending <= 1'b0;
	end else begin
		vblank_d <= vblank_in;
		if (vblank_in && !vblank_d)        irq_pending <= 1'b1;
		if (cpu_irq_active && irq_pending) irq_pending <= 1'b0;
	end
end
assign dbg_irq_pending = irq_pending;

// irq_pending gira a clk LIBERO e il suo cono arriva ai registri di SEGMENTO
// della richiesta nel BIU (r_rq_seg), cioe' dentro il calcolo dell'indirizzo
// fisico del ciclo di bus. MISURATO 2026-08-17 sulla build 15:05, con la
// coperta `irq_pending -> *v30_core:u_core*` rimossa dall'SDC:
//     -3.385 ns   irq_pending -> v30u_biu:u_biu|r_rq_seg[1][0]
// cioe' il percorso chiede 15.885 ns e a clock libero ne ha 12.5. La coperta
// dichiarava 2 periodi dove ce n'e' 1: STA verde e SILICIO CHE SBAGLIA -- la
// stessa forma del bug punteggio storico (multicycle 9 falso, -11.03/-11.85)
// e di CE_HALF a +1 clk (-11.316, vedi cpu_v30_bridge). L'INT arriva a vblank,
// quando il gioco disegna l'HUD: se il fronte cade mentre il BIU forma il
// segmento, il ciclo va all'indirizzo sbagliato -> tile nella cella accanto.
// Registrandolo su ce il lancio e' ce-paced: il percorso diventa CE->CE e ha
// 62.5 ns (CE_GAP_MIN=5) contro i 15.885 richiesti. INT e' un livello che resta
// alto fino all'acknowledge, quindi un ce di ritardo non cambia la semantica.
reg irq_pending_ce;
always @(posedge clk) if (ce) irq_pending_ce <= irq_pending;


// ─── Address translator ─────────────────────────────────────────────────
wire        ls245_en;
wire [23:0] sdr_addr;
wire        ram_memrq, bgram_memrq, fgram_memrq, palette_memrq;
wire        shared_memrq, nopw_a_memrq, nopw_wd_memrq, nopw_b_memrq;
wire        DBEN = cpu_rd | cpu_wr;

raiden_addr_sub u_addr (
	.A             (cpu_addr),
	.DBEN          (DBEN),
	.ls245_en      (ls245_en),
	.sdr_addr      (sdr_addr),
	.ram_memrq     (ram_memrq),
	.bgram_memrq   (bgram_memrq),
	.fgram_memrq   (fgram_memrq),
	.palette_memrq (palette_memrq),
	.shared_memrq  (shared_memrq),
	.nopw_a_memrq  (nopw_a_memrq),
	.nopw_wd_memrq (nopw_wd_memrq),
	.nopw_b_memrq  (nopw_b_memrq)
);

// ─── SDRAM bridge (mem_rq_active local) ─────────────────────────────────
// Pattern M72 m72.v:262-269: addr LATCHED nel FSM, non passa-attraverso
reg sub_rq_active;
reg sub_rd_lat;
reg [15:0] sub_ram_rom_data;
reg [23:0] sub_rom_addr_lat;
always @(posedge clk) begin
	if (reset) begin
		sub_rq_active    <= 1'b0;
		sub_rd_lat       <= 1'b0;
		sub_ram_rom_data <= 16'd0;
		sub_rom_addr_lat <= 24'd0;
	end else begin
		sub_rd_lat <= cpu_rd;
		if (!sub_rq_active) begin
			if (ls245_en && cpu_rd && !sub_rd_lat) begin
				sub_rq_active    <= 1'b1;
				sub_rom_addr_lat <= sdr_addr;
			end
		end else if (sub_rom_ready) begin
			sub_ram_rom_data <= sub_rom_rdata;
			sub_rq_active    <= 1'b0;
		end
	end
end
assign sub_rom_addr = sub_rom_addr_lat;
assign sub_rom_req  = sub_rq_active;


// LAG DI ASSESTAMENTO DOPO IL FETCH (root-cause storico ba9cf5e, adattato).
// sub_ram_rom_data si aggiorna al clock in cui arriva sub_rom_ready; rdata_q in
// v30_bus lo campiona UN clock dopo. Se il CE riparte subito, il BIU consuma
// la parola PRECEDENTE: e' l'immediato letto storto che disallinea indice/NUL
// in itoa (0xfe55) -> il NUL non termina la stringa -> strcpy 0xf150 sfora di
// 2 celle con residuo stantio -> punteggio con 2 celle sporche, RAM corretta.
// Il bridge aveva un lag conservativo di 2 clk per questo motivo; il passaggio
// al CE-stall l'ha perso. Qui lo ripristino: lo stallo resta alto 2 clk dopo
// il ready, cioe' finche' rdata_q non presenta il dato nuovo.
reg [1:0] rom_settle;
always @(posedge clk) begin
	if (reset)                            rom_settle <= 2'd0;
	else if (sub_rq_active && sub_rom_ready)   rom_settle <= 2'd2;
	else if (rom_settle != 2'd0)          rom_settle <= rom_settle - 2'd1;
end
wire sub_stall_eff = sub_rq_active | (rom_settle != 2'd0);

// ─── CE generator pattern M72 ───────────────────────────────────────────
wire ce, ce_4x;
raiden_ce_gen #(.PHASE_INIT(4'd4)) u_ce (   // sfasato di 4 clk vs Main: mai richieste SDRAM simultanee
	.clk           (clk),
	.reset         (reset),
	.pause         (pause),
	.clk_sel       (clk_sel),
	.ls245_en      (ls245_en),
	.rd_lat        (sub_rd_lat),
	.mem_rq_active (sub_stall_eff),
	.ce            (ce),
	.ce_4x         (ce_4x)
);

// ─── V30 CPU bridge ─────────────────────────────────────────────────────
cpu_v30_bridge #(.SS_IDX(SS_IDX_CPU)) u_cpu (
	.clk           (clk),
	.ce            (ce),
	.ce_4x         (ce_4x),
	.reset         (reset),
	.rom_wait      (sub_rq_active),
	.bus_addr      (cpu_addr),
	.bus_read      (cpu_rd),
	.bus_write     (cpu_wr),
	.bus_be        (cpu_be),
	.bus_dout      (cpu_dout),
	.bus_din       (cpu_din),
	.irq_req       (irq_pending_ce),
	.irq_vector    (10'h0C8),
	.cpu_idle      (cpu_idle),
	.cpu_halt      (),
	.cpu_irqrequest(cpu_irq_active),
	.cpu_prefix    (),
	.ss            (ss_cpu),
	.ss_cpu_reload (ss_cpu_reload)
);

// ─── Sub RAM 8KB (4K word) ──────────────────────────────────────────────
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] ram_lo [0:4*1024-1];
(* ramstyle = "M10K,no_rw_check" *) reg [7:0] ram_hi [0:4*1024-1];
initial begin
	integer i;
	for (i = 0; i < 4*1024; i = i + 1) begin ram_lo[i]=0; ram_hi[i]=0; end
end

wire [11:0] ram_word_addr = cpu_addr[12:1];
// M72-native byte lanes: cpu_be[0]=lane bassa, cpu_be[1]=lane alta; cpu_dout già allineato.
wire cpu_we_lo = cpu_wr && cpu_be[0];
wire cpu_we_hi = cpu_wr && cpu_be[1];

// Savestate su porta B dedicata (dual-port M10K): la porta A CPU resta
// IDENTICA all'originale → zero disturbo al path vivo (non tocca il fit).
// A gioco acceso ss_sub_sel=0: porta B inattiva, RAM = single-port di sempre.
wire        ss_sub_sel = ss_subram.access(SS_IDX_SUBRAM);
wire [11:0] ss_sub_idx = ss_subram.addr[11:0];
wire        ss_sub_wr  = ss_sub_sel & ss_subram.write;
reg  [15:0] ss_sub_rdata;
reg         ss_sub_rdel;

// Template true-dual-port M10K: UN always per porta (ognuno legge/scrive
// il PROPRIO indirizzo). Due always separati sullo stesso array = pattern
// riconosciuto da Quartus → mappa su M10K true dual-port.
reg [15:0] ram_rdata;
// Porta A — CPU sub (live, invariata)
always @(posedge clk) begin
	if (ram_memrq) begin
		if (cpu_we_lo) ram_lo[ram_word_addr] <= cpu_dout[7:0];
		if (cpu_we_hi) ram_hi[ram_word_addr] <= cpu_dout[15:8];
	end
	ram_rdata <= {ram_hi[ram_word_addr], ram_lo[ram_word_addr]};
end
// Porta B — savestate (write al restore, read al save)
always @(posedge clk) begin
	if (ss_sub_wr) begin
		ram_lo[ss_sub_idx] <= ss_subram.data[7:0];
		ram_hi[ss_sub_idx] <= ss_subram.data[15:8];
	end
	ss_sub_rdata <= {ram_hi[ss_sub_idx], ram_lo[ss_sub_idx]};
end

// Handshake ssbus (controllo, non RAM)
always @(posedge clk) begin
	ss_subram.setup(SS_IDX_SUBRAM, 32'd4096, 1);   // 4096 word, 16 bit
	if (ss_sub_sel) begin
		if (ss_subram.write)     ss_subram.write_ack(SS_IDX_SUBRAM);
		else if (ss_subram.read) begin
			if (ss_sub_rdel) ss_subram.read_response(SS_IDX_SUBRAM, {48'd0, ss_sub_rdata});
			ss_sub_rdel <= 1;
		end
	end else ss_sub_rdel <= 0;
end

// ─── Video sub-bus (BG + FG + Palette) ──────────────────────────────────
wire [15:0] vbus_DOUT;
wire        vbus_DOUT_VALID;
raiden_video_subbus #(
	.SS_IDX_BG(SS_IDX_BG), .SS_IDX_FG(SS_IDX_FG), .SS_IDX_PAL(SS_IDX_PAL)
) u_vbus (
	.clk           (clk),
	.reset         (reset),
	.cpu_addr      (cpu_addr),
	.cpu_rd        (cpu_rd),
	.cpu_wr        (cpu_wr),
	.cpu_be        (cpu_be),
	.cpu_dout      (cpu_dout),
	.bgram_memrq   (bgram_memrq),
	.fgram_memrq   (fgram_memrq),
	.palette_memrq (palette_memrq),
	.DOUT          (vbus_DOUT),
	.DOUT_VALID    (vbus_DOUT_VALID),
	.bg_vram_addr  (bg_vram_addr),
	.bg_vram_data  (bg_vram_data),
	.fg_vram_addr  (fg_vram_addr),
	.fg_vram_data  (fg_vram_data),
	.pal_vram_addr (pal_vram_addr),
	.pal_vram_data (pal_vram_data),
	.ss_bg         (ss_bg),
	.ss_fg         (ss_fg),
	.ss_pal        (ss_pal)
);

// ─── Shared RAM bridge (Sub side) ──────────────────────────────────────
assign sub_shared_addr  = cpu_addr[11:1];
assign sub_shared_cs    = shared_memrq;
assign sub_shared_we    = (shared_memrq && cpu_wr) ? {cpu_be[1], cpu_be[0]} : 2'b00;
assign sub_shared_wdata = cpu_dout;

// ─── DOUT_VALID mux for cpu_din ────────────────────────────────────────
reg ram_rd_lat, shared_rd_lat;

always @(posedge clk) begin
	if (reset) begin
		ram_rd_lat      <= 1'b0;
		shared_rd_lat   <= 1'b0;
	end else begin
		ram_rd_lat      <= cpu_rd & ram_memrq;
		shared_rd_lat   <= cpu_rd & shared_memrq;
	end
end

// Core M72 lane-aware: word naturale, il core seleziona il byte (niente byte_align).
always @(*) begin
	if      (vbus_DOUT_VALID) cpu_din = vbus_DOUT;
	else if (ram_rd_lat)      cpu_din = ram_rdata;
	else if (shared_rd_lat)   cpu_din = sub_shared_rdata;
	else                       cpu_din = sub_ram_rom_data;    // fallback ROM (SDRAM)
end

// ─── Debug taps (palette_overlay e simili) — tied-off ─────────────────
assign dbg_cpu_addr      = cpu_addr;
assign dbg_cpu_dout      = cpu_dout;
assign dbg_cpu_be        = cpu_be;
assign dbg_cpu_wr        = cpu_wr;
assign dbg_palette_memrq = palette_memrq;

`ifdef V30_SIM_PROBES
// ── TEMPO DI SERVIZIO DEL SUB (quanto il MAIN lo aspetta) ────────────────
// Il main manda un comando in shared RAM (sub 0x4000 = main 0x8000), poi
// SPIN-ASPETTA a FB2B7 che il sub scriva [0x4000]=0 (FCD80 "ho finito").
// Misuro per ogni comando: clk fra la PRIMA lettura del sub di [0x4000]
// che trova != 0 (FCCE2 esce dal suo spin) e la scrittura [0x4000]=0.
// Se questo tempo cresce con la scena e si avvicina/supera il frame
// (1.346.560 clk), il main e' fermo ad aspettare = RALLENTAMENTO.
integer sv_t0 = -1, sv_n = 0, sv_max = 0, sv_sum = 0, sv_clk = 0;
reg     sv_busy = 0;
always @(posedge clk) begin
    sv_clk <= sv_clk + 1;
    // lettura sub di [4000] con dato != 0: il sub ha preso il comando
    if (shared_memrq && cpu_rd && cpu_addr[11:1] == 11'd0 && !sv_busy &&
        u_cpu.u_core.t_state == 3'd3 && u_cpu.u_core.lat_type == 3'b101 &&
        sub_shared_rdata != 16'd0) begin
        sv_busy <= 1'b1; sv_t0 <= sv_clk;
    end
    // scrittura sub di [4000]=0: finito
    if (shared_memrq && cpu_wr && cpu_addr[11:1] == 11'd0 && sv_busy &&
        cpu_dout == 16'd0) begin
        sv_busy <= 1'b0; sv_n <= sv_n + 1;
        sv_sum <= sv_sum + (sv_clk - sv_t0);
        if (sv_clk - sv_t0 > sv_max) sv_max <= sv_clk - sv_t0;
        if (sv_clk - sv_t0 > 32'd400000)
            $display("[subsvc] comando servito in %0d clk = %0d%% frame", sv_clk - sv_t0, ((sv_clk - sv_t0)*100)/1346560);
    end
    if (sv_clk % 1346560 == 0 && sv_clk != 0) begin
        $display("[subsvc] frame: comandi=%0d  medio=%0d clk  max=%0d clk (%0d%% frame)",
                 sv_n, (sv_n != 0) ? sv_sum / sv_n : 0, sv_max, (sv_max*100)/1346560);
        sv_n <= 0; sv_sum <= 0; sv_max <= 0;
    end
end
`endif

endmodule
