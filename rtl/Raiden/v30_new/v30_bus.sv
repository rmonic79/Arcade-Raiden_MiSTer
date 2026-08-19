//============================================================================
//  Irem M72 for MiSTer FPGA - V30 core bus adapter
//
//  Wraps the cycle-accurate nec_test v30_core (max-mode, muxed AD/BS bus) and
//  presents a lean, word-aligned bus to m72.v: address+byte-enables, separate
//  read/write strobes for memory and IO, an INTA/vector handshake, and a
//  code-fetch flag (BS==CODE) so the sim DebugLink can tell prefetch from data
//  reads.
//
//  Timing model (CE-freeze with catch-up):
//    ce      : advances a T-state; 8MHz average rate
//    ce_half : the T1 address-latch strobe, one fabric clock after each ce
//  The core is frozen by withholding ce/ce_half while an SDRAM access is
//  outstanding, then catches up at one phase per fabric clock (16MHz burst)
//  until it matches the steady 8MHz reference (m72.v owns that pacing).
//  READY (m72.v-driven) inserts real Tw states for the sprite/tile RAM
//  wait-state feature; it is orthogonal to the SDRAM CE-freeze above.
//
//  Distilled from nec_test/hdl/rtl/nec_bus.sv T-state tracker (large mode,
//  minus wait-states / random / capture / power sequencing / harness), and
//  the v30_core instantiation in nec_test/hdl/rtl/system_large.sv.
//============================================================================

module v30_bus #(
    parameter SS_IDX = -1
) (
    input             clk,
    input             ce,          // CPU-clock advance strobe (T-state)
    input             ce_half,     // T1 address-latch strobe (2 clks after ce)
    input             reset,       // active high
    input             ready,       // V30 READY: low at T3/Tw inserts Tw (M72 wait-states)

    // word-aligned unified bus
    output     [19:0] cpu_addr,    // bit0 forced 0
    output      [1:0] cpu_be,      // [0]=low byte (A0==0), [1]=high byte (~UBE_N)
    output     [15:0] cpu_dout,    // write data (valid during a write cycle)
    input      [15:0] cpu_din,     // read data (sampled by the core at T3/T4)

    // access strobes
    output            mem_rd,      // level, T1-half .. T4 (CODE or MEMR)
    output            io_rd,       // level, T1-half .. T4 (IOR)
    output            mem_wr,      // level, held during T3 (MEMW)
    output            io_wr,       // level, held during T3 (IOW)
    output            code_fetch,  // current cycle is a prefetch (BS==CODE)

    // interrupt handshake
    input             int_req,
    input       [7:0] int_vector,
    output            int_ack,     // level, held during T3 of the 2nd INTA

    // full register file for the sim CPU window (zeros unless V30_BACKDOOR)
    output    [223:0] dbg_regs,

    // savestate: streams the core's 202-entry SS register file
    ssbus_if.slave    ssbus,
    input             ss_restore_done, // pulse: reset this adapter to bus-idle
    output            ss_quiet         // core BIU is bus-quiet (safe to freeze)
);

import v30_ss_pkg::*;

// Bus status codes (8086 S2-S0 compatible)
localparam bit [2:0] BS_INTA = 3'b000;
localparam bit [2:0] BS_IOR  = 3'b001;
localparam bit [2:0] BS_IOW  = 3'b010;
localparam bit [2:0] BS_HALT = 3'b011;
localparam bit [2:0] BS_CODE = 3'b100;
localparam bit [2:0] BS_MEMR = 3'b101;
localparam bit [2:0] BS_MEMW = 3'b110;
localparam bit [2:0] BS_PASV = 3'b111;

// T-state encoding (ST_TW value matches the BIU's, v30_biu.sv ST_TW=3'd4)
localparam bit [2:0] ST_TI = 3'd0;
localparam bit [2:0] ST_T1 = 3'd1;
localparam bit [2:0] ST_T2 = 3'd2;
localparam bit [2:0] ST_T3 = 3'd3;
localparam bit [2:0] ST_TW = 3'd4;
localparam bit [2:0] ST_T4 = 3'd5;

//----------------------------------------------------------------------------
// Core instance (private muxed AD net)
//----------------------------------------------------------------------------
// BUS DE-MUXATO: nessun pad condiviso da emulare. Il dato letto entra dal
// core via DATA_I, l'indirizzo e il dato scritto escono da ADDR_O/DATA_O.
wire  [2:0] BS;
wire        RD_N, UBE_N;
// Porte pubblicate dal core in OGNI configurazione (v30u_biu "THE DE-MUXED
// BUS"): addr_o e' l'indirizzo lineare del ciclo che possiede il bus, VALIDO
// dall'annuncio per tutto T1/T2/Tw/T3/T4; data_o e' la parola da scrivere.
// Sono gli stessi registri da cui il core COMPONE il bus muxato ("a RE-SLICING
// OF A MUX, not a new mechanism, adds NO FLOP"), quindi i valori sono identici
// a quelli che prima ricavavamo campionando i pin.
wire [19:0] ADDR_O;
wire [15:0] DATA_O;
wire  [3:0] STATUS_O;
reg  [15:0] rdata_q;

`ifdef V30_BACKDOOR
wire [223:0] core_dbg_regs;
assign dbg_regs = core_dbg_regs;
`else
assign dbg_regs = 224'd0;
`endif

// Savestate regfile access.  Contract (v30_core.sv assertions): SS_WE only
// while CE is withheld (the core is frozen whenever the ssbus is active),
// exactly one clk per entry, and the 2-deep staging must drain before CE
// resumes (the m72 controller's post-restore drain covers this).  SS_RDATA
// is valid 2 clks after SS_ADDR presents.
wire [8:0]  ss_core_addr = ss_addr_of(int'(ssbus.addr));
wire [15:0] ss_core_rdata;
reg  [1:0]  ss_rd_delay;
reg  [4:0]  ss_wr_delay;   // staging: 2 clk addr/dato stabili, poi SS_WE largo
                           // 3 clk (deviazione DOCUMENTATA dal contratto "1 clk":
                           // tutte le ss-write sono ASSEGNAMENTI PURI (verificato
                           // eu_ss_write.svh + biu: 0 costrutti non-idempotenti,
                           // TAG/ERR = compare) -> doppio load identico innocuo.
                           // Il 2o capture avviene con WE/decode stabili da 1 clk
                           // -> multicycle 2 VERO sull'arco ss_we_q->regfile
                           // (fanout ~18ns che a 80MHz non chiude single-cycle;
                           // upstream gira a 45MHz e non lo vede: da segnalare).
reg         ss_wr_done;
wire        ss_core_we = ssbus.access(SS_IDX) & ssbus.write & ss_wr_delay[1] & ~ss_wr_done;
wire        ss_err /* verilator public_flat */;

`ifdef V30_SIM_PROBES
wire [1:0] qs_dbg;   // solo probe sim (QS = queue status)
`endif
v30_core u_core (
    .CLK        (clk),
    .CE         (ce),
    .RESET      (reset),
    .READY      (ready),
    .INT        (int_req),
    .NMI        (1'b0),
    .POLL_N     (1'b1),
    .DATA_I     (rdata_q),
    .ADDR_O     (ADDR_O),
    .DATA_O     (DATA_O),
    .STATUS_O   (STATUS_O),
`ifdef V30_SIM_PROBES
    .QS         (qs_dbg),
`else
    .QS         (),
`endif
    .BS         (BS),
    .RD_N       (RD_N),
    .UBE_N      (UBE_N),
    .BUSLOCK_N  (),
    .SS_ADDR    (ss_core_addr),
    .SS_WDATA   (ssbus.data[15:0]),
    .SS_WE      (ss_core_we),
    .SS_RDATA   (ss_core_rdata),
    .SS_ERR     (ss_err),
    .SS_BUS_QUIET(ss_quiet)
`ifdef V30_BACKDOOR
    ,
    .bkd_load     (1'b0),
    .bkd_regs     (224'd0),
    .bkd_queue    (48'd0),
    .bkd_qlen     (3'd0),
    .bkd_fetch_ip (16'd0),
    .scr_en       (1'b0),
    .scr_qop      (2'd0),
    .dbg_regs     (core_dbg_regs),
    .dbg_first_pop(),
    .dbg_pend     ()
`endif
);

//----------------------------------------------------------------------------
// Savestate slave: 202 x 16-bit regfile entries streamed via the ssbus.
// Reads respect the 2-clk SS_ADDR->SS_RDATA staging; writes pulse SS_WE for
// exactly one clk per entry (ss_wr_done holds it off while the master waits
// for the ack to propagate through the mux).
//----------------------------------------------------------------------------
always_ff @(posedge clk) begin
    ssbus.setup(SS_IDX, v30_ss_pkg::SS_COUNT, 1);

    if (ssbus.access(SS_IDX)) begin
        if (ssbus.write) begin
            // clk1-2: staging addr/dato. clk3-6: SS_WE alto (4 clk, idempotente:
            // il capture BUONO e' il 4o, con WE/decode/dato stabili da 3 clk).
            // Ack al clk6 -> il master tiene tutto stabile per l'intera finestra.
            // 4 e non 3: il fanout della WE verso il regfile e i blocchi M10K
            // della ROM di microcodice e' ~39 ns e con 3 periodi (37.5 ns) non
            // chiudeva (misurato -1.737 ns). Un clock di staging in piu' rende
            // VERO il multicycle 4 in Template.sdc. Il restore e' fuori tempo
            // reale: un clock in piu' per entry non ha alcun effetto visibile,
            // e lo STREAM del savestate resta identico.
            ss_wr_delay <= { ss_wr_delay[3:0], 1'b1 };
            if (ss_wr_delay[4] && !ss_wr_done) begin
                ss_wr_done <= 1;
                ssbus.write_ack(SS_IDX);
            end
        end else if (ssbus.read) begin
            ss_rd_delay <= { ss_rd_delay[0], 1'b1 };
            if (ss_rd_delay[1]) begin
                ssbus.read_response(SS_IDX, { 48'd0, ss_core_rdata });
            end
        end
    end else begin
        ss_rd_delay <= 0;
        ss_wr_delay <= 0;
        ss_wr_done <= 0;
    end
end

//----------------------------------------------------------------------------
// Registered pin samples (mirrors nec_bus: status/data are one clk old at the
// FSM edge, reproducing the verified end-of-cycle timing relationship).
//----------------------------------------------------------------------------
reg  [2:0] bs_q;
reg        ube_n_q;

`ifdef MISTER_SIM_SDC_STRESS
// STRESS SDC (solo sim): modella il caso PEGGIORE concesso dalle eccezioni
// boundary 2/1 — nel primo clk dopo il lancio CE il cono puo' non essere
// assestato, quindi la cattura a L+1 e' SPAZZATURA (qui: inversione bitwise,
// avversariale). Se il boot sopravvive, l'eccezione e' funzionalmente sana
// ("la cattura a L+1 non e' mai letta"); se muore, l'SDC e' sbagliata.
reg ce_d;
always_ff @(posedge clk) ce_d <= ce;
always_ff @(posedge clk) begin
    bs_q    <= ce_d ? ~BS    : BS;
    ube_n_q <= ce_d ? ~UBE_N : UBE_N;
end
`else
// CATTURA A +4 CLK DAL CE, non a ogni clock. MISURATO 2026-08-17 con l'STA,
// riportando a ciclo singolo il boundary 2/1 che la copriva:
//     -8.993 ns   v30u_ucrom (porta M10K) -> bs_q
// cioe' il cono che alimenta BS chiede ~21,5 ns e a clock libero ne ha 12,5.
// Da bs_q escono `lat_type` (TIPO del ciclo: CODE/MEMR/MEMW) e `bs_active`
// (avvio del T-state): un valore latchato male puo' far perdere il
// riconoscimento di un ciclo di SCRITTURA -> la write non viene emessa e la
// cella resta col contenuto vecchio.
// TUTTI i consumatori leggono a un CE (`lat_type <= bs_q` all'ingresso in T1,
// `bs_active` in next_t) e il CE successivo arriva >= CE_GAP_MIN = 5 clk dopo:
// catturando a +4 il valore e' pronto PRIMA di essere letto, il valore letto e'
// identico a prima, e l'arco ha 4 periodi VERI -> il vincolo non e' piu' una
// copertura. E' la stessa cura di CE_HALF portato da +1 a +2 clk
// (cpu_v30_bridge.sv:66-82), che chiuse una violazione da -11.316 ns.
reg [3:0] ce_pipe;
always_ff @(posedge clk) ce_pipe <= {ce_pipe[2:0], ce};
always_ff @(posedge clk) if (ce_pipe[3]) begin
    bs_q    <= BS;
    ube_n_q <= UBE_N;
end
`endif

// RIMOSSA la cattura sul FRONTE DI DISCESA (era: addr_neg/ube_neg campionati
// a meta' clock). Quello schema serviva quando `t1_half2` del core era una
// flop NEGEDGE: si campionava nella mezza finestra fra il CE che entra in T1 e
// il fronte con cui il core processava CE_HALF. Upstream ha ri-landato
// `t1_half2` come flop POSEDGE (nec_test 638ed014): quella mezza finestra non
// esiste piu' e campionare li' e' FUORI FASE -> indirizzo o byte-enable del
// ciclo sbagliato.
// Ora indirizzo e byte-enable si prendono dalle porte che il core pubblica
// (ADDR_O / UBE_N), latchati sul posedge all'ingresso in T1: un arco normale
// da 1 periodo, senza mezzi fronti e senza eccezioni SDC.

//----------------------------------------------------------------------------
// T-state tracker (advances under ce)
//----------------------------------------------------------------------------
reg  [2:0] t_state;
reg  [2:0] lat_type;
reg        is_read_cycle, is_write_cycle;
reg        addr_valid;
reg        inta_prev, inta_second;
reg [19:0] addr_lat;
reg  [1:0] be_lat;
reg        a0_lat;
reg [15:0] dout_lat;

wire bs_active = bs_q != BS_PASV;

// READY REGISTRATO al clock CPU (CE), come il BIU dell'ucore (contratto M2r,
// v30u_biu ready_prev: "the CPU registers READY at the end of every clock").
// Usare `ready` LIVE qui desincronizzava tracker e BIU quando READY cambia
// tra i due istanti di campionamento (SOLO sotto latenza SDRAM reale: con lo
// stub a latenza 1 READY era sempre alto): T-state sfasati -> decodifiche
// bus fantasma (MEMR@0 al ciclo 2 in sim 1:1) -> boot deragliato = schermo
// nero su hardware con STA verde.
reg ready_q;
always_ff @(posedge clk) begin
    if (reset)   ready_q <= 1'b1;
    else if (ce) ready_q <= ready;
end

wire [2:0] next_t =
    (t_state == ST_TI) ? (bs_active ? ST_T1 : ST_TI) :
    (t_state == ST_T1) ? ST_T2 :
    (t_state == ST_T2) ? ST_T3 :
    (t_state == ST_T3) ? (ready_q ? ST_T4 : ST_TW) : // READY registrato -> Tw (come BIU)
    (t_state == ST_TW) ? (ready_q ? ST_T4 : ST_TW) : // loop Tw finche' ready_prev basso
    /* ST_T4 */          (bs_active ? ST_T1 : ST_TI);

wire read_type  = (bs_q == BS_CODE) || (bs_q == BS_MEMR) ||
                  (bs_q == BS_IOR)  || (bs_q == BS_INTA);
wire write_type = (bs_q == BS_MEMW) || (bs_q == BS_IOW);

always_ff @(posedge clk) begin
    if (reset || ss_restore_done) begin
        // On ss_restore_done the adapter returns to bus-idle: the savestate
        // quiesce guarantees no cycle was in flight at save time, so every
        // other adapter register is dead state until the next T1.
        t_state        <= ST_TI;
        lat_type       <= BS_PASV;
        is_read_cycle  <= 1'b0;
        is_write_cycle <= 1'b0;
        addr_valid     <= 1'b0;
        inta_prev      <= 1'b0;
        inta_second    <= 1'b0;
    end else begin
        if (ce) begin
            t_state <= next_t;

            if (next_t == ST_T1) begin
                lat_type       <= bs_q;
                is_read_cycle  <= read_type;
                is_write_cycle <= write_type;
                if (bs_q == BS_INTA) begin
                    inta_second <= inta_prev;
                    inta_prev   <= 1'b1;
                end else begin
                    inta_second <= 1'b0;
                    inta_prev   <= 1'b0;
                end
            end

            // BYTE-ENABLE: un ce DOPO l'indirizzo, cioe' al confine T1->T2.
            // Il core lo documenta (v30u_biu, F53b): "UBE is NOT part of the
            // status register -- it changes ONE CLOCK AFTER the status does" e
            // vale `r_cur_ube_n` solo mentre `r_ts == TS_T1`. Campionarlo sul
            // fronte in cui si ENTRA in T1 restituisce `last_ube`, cioe' il
            // byte-enable del ciclo PRECEDENTE -> scritture sulla corsia
            // sbagliata (sprite rotti). Qui t_state e' T1 e ube_n e' quello
            // giusto; be_lat serve a T3 (write) quindi arriva largamente prima.
            //
            // 2026-08-17: PROVATO DI NUOVO a spostarlo insieme all'indirizzo e
            // ha riprodotto ESATTAMENTE la regressione descritta qui sopra
            // (hi-score a 5, comandi morti, sprite scomparsi). NON RIPETERE.
            // Le due corsie vengono da segnali con finestre di validita'
            // DIVERSE, quindi si prendono dove ognuna e' garantita:
            //  - UBE_N vale solo mentre il core e' in T1 (v30u_biu:1017) -> qui;
            //  - A0 e' l'indirizzo, valido per TUTTO il ciclo, e lo abbiamo gia'
            //    catturato all'ingresso in T1 insieme al resto (a0_lat).
            // Prendere anche A0 in questo istante significa rileggere ADDR_O in
            // un punto diverso da quello in cui e' stato latchato l'indirizzo:
            // se le due letture non cadono sullo stesso ciclo la corsia bassa si
            // perde -> be=10 (scrive solo l'attributo, carattere vecchio) o
            // be=00 (non scrive niente). E' l'impronta misurata nel savestate.
            if (t_state == ST_T1)
                be_lat <= {~UBE_N, ~a0_lat};

            // dato da scrivere: dalla porta del core (stessa sorgente da cui
            // componeva AD nella fase dati), non piu' dal campione dei pin
            if ((t_state == ST_T2 || t_state == ST_T3) && is_write_cycle)
                dout_lat <= DATA_O;

        end

        // INDIRIZZO: latchato all'ingresso in T1. ADDR_O e' gia' quello del
        // ciclo annunciato ("VALID FROM the announcement clock"), quindi la
        // richiesta di memoria parte presto come prima.
        if (ce && next_t == ST_T1) begin
            addr_lat   <= {ADDR_O[19:1], 1'b0};
            a0_lat     <= ADDR_O[0];      // parita' dello STESSO ciclo dell'indirizzo
            addr_valid <= 1'b1;
        end else if (ce && (next_t == ST_T4 || next_t == ST_TI)) begin
            addr_valid <= 1'b0;
        end
    end
end

// dato letto verso il core (DATA_I): nei cicli INTA e' il vettore
always_ff @(posedge clk)
    rdata_q <= (lat_type == BS_INTA) ? {8'h00, int_vector} : cpu_din;

//----------------------------------------------------------------------------
// Outputs
//----------------------------------------------------------------------------
assign cpu_addr   = addr_lat;
assign cpu_be     = be_lat;
assign cpu_dout   = dout_lat;
assign code_fetch = (lat_type == BS_CODE);

// read strobes: level from T1-half through T3 (addr_valid), cleared at T4
assign mem_rd = addr_valid && ((lat_type == BS_CODE) || (lat_type == BS_MEMR));
assign io_rd  = addr_valid && (lat_type == BS_IOR);

// write strobes: level held across T3 and any Tw. At zero waits there is no Tw
// so this is the single-T3 pulse as before; under waits it holds the request so
// the wait-stated sprite/tile RAM can commit the write at its own gate.
assign mem_wr = (lat_type == BS_MEMW) && (t_state == ST_T3 || t_state == ST_TW);
assign io_wr  = (lat_type == BS_IOW)  && (t_state == ST_T3 || t_state == ST_TW);

// INTA acknowledge: level held during T3 of the second INTA cycle
assign int_ack = (lat_type == BS_INTA) && inta_second && (t_state == ST_T3);

`ifdef V30_SIM_PROBES
// ── VERIFICA DELLE CORSIE DI BYTE ────────────────────────────────────────
// Upstream F53b: "UBE IS LOADED BY THE ADDRESS PHASE AND THEN HELD".  Quindi
// per tutto il ciclo il pin deve valere l'UBE di QUESTO ciclo, e `be_lat`
// (catturato a T1-body) deve coincidere con esso anche a T3, quando la write
// viene commessa.  Se differisce, be_lat ha preso la corsia di un ALTRO ciclo:
// su una scrittura di UN byte questo accende anche la corsia adiacente e la
// riempie col byte alto del dato (0x00) -> un NUL in mezzo al buffer di itoa
// -> terminatore fuori posto -> celle sporche in coda al campo punteggio.
// Impronta attesa nel savestate: parola 0x0030 dove servivano due cifre.
integer be_bad = 0, be_tot = 0;
always @(posedge clk) if (ce && t_state == ST_T3 && is_write_cycle) begin
    be_tot <= be_tot + 1;
    if (be_lat != {~UBE_N, ~a0_lat}) begin
        be_bad <= be_bad + 1;
        if (be_bad < 24)
            $display("[belane %m] MISMATCH addr=%05h be_lat=%b vivo=%b dato=%04h",
                     addr_lat, be_lat, {~UBE_N, ~a0_lat}, dout_lat);
    end
end
integer be_win = 0;
always @(posedge clk) begin
    be_win <= be_win + 1;
    if (be_win == 4_000_000) begin
        be_win <= 0;
        $display("[belane %m] write=%0d mismatch=%0d", be_tot, be_bad);
    end
end

// ── PROVENIENZA DELLA WRITE (lockstep tracker ↔ BIU) ─────────────────────
// Proprieta' da dimostrare: al CE che CHIUDE il T3 di una scrittura, il BIU
// deve essere in T3 dello STESSO ciclo e i tre latch dell'adattatore devono
// coincidere con i registri del ciclo che CORRE nel BIU:
//   addr_lat == r_cur_addr (word)
//   be_lat   == {~r_cur_ube_n, ~r_cur_addr[0]}
//   dout_lat == DATA_O  (= r_cur_data, o il pairing live dell'EU)
// Se il tracker e' sfasato anche di UN solo CE rispetto al BIU, dout_lat
// firma la write col dato del ciclo PRECEDENTE (impronta 01/8d: la parola
// della cornice appena scritta ricommessa sulla cella dopo) oppure con le
// corsie di un altro ciclo (impronta fb/02: attributo scritto, carattere
// vecchio).  Controlla OGNI scrittura: non aspetta quella colpevole.
integer wp_tot = 0, wp_bad = 0, wp_t1_lag = 0;
always @(posedge clk) if (ce) begin
    if (t_state == ST_T3 && is_write_cycle) begin
        wp_tot <= wp_tot + 1;
        if (!(u_core.u_biu.r_run && u_core.u_biu.r_cur_wr &&
              u_core.u_biu.r_ts == 3'd3) ||
            (addr_lat != {u_core.u_biu.r_cur_addr[19:1], 1'b0}) ||
            (dout_lat != DATA_O) ||
            (be_lat   != {~u_core.u_biu.r_cur_ube_n,
                          ~u_core.u_biu.r_cur_addr[0]})) begin
            wp_bad <= wp_bad + 1;
            if (wp_bad < 40)
                $display("[wprov %m] MISMATCH addr_lat=%05h dout_lat=%04h be=%b | biu: run=%b wr=%b ts=%0d cur_addr=%05h cur_data=%04h ube_n=%b DATA_O=%04h cur_bs=%0d cmt_v=%b cdage=%0d",
                         addr_lat, dout_lat, be_lat,
                         u_core.u_biu.r_run, u_core.u_biu.r_cur_wr,
                         u_core.u_biu.r_ts, u_core.u_biu.r_cur_addr,
                         u_core.u_biu.r_cur_data, u_core.u_biu.r_cur_ube_n,
                         DATA_O, u_core.u_biu.r_cur_bs,
                         u_core.u_biu.r_cmt_valid, u_core.u_biu.r_cdage);
        end
    end
    // sfasamento all'apertura: tracker in T1 ma il BIU non ha (ancora) aperto
    // il T1 del ciclo — 1 clk di annuncio-in-attesa e' fisiologico, di piu' no
    if (t_state == ST_T1 && !(u_core.u_biu.r_run &&
                              u_core.u_biu.r_ts == 3'd1))
        wp_t1_lag <= wp_t1_lag + 1;
end
// ── WRITE PERSE (il caso che [wprov] NON copre) ──────────────────────────
// [wprov] verifica le write che il tracker EMETTE.  Una write PERSA e' il
// contrario: il BIU corre il T3 di un MEMW ma il tracker non e' in T3 di un
// ciclo di scrittura -> mem_wr resta basso -> la cella tiene il contenuto
// VECCHIO (impronta 01/8d se prima li' c'era la cornice legittima mai
// ripulita).  Anche questo si controlla su OGNI ciclo del BIU.
integer wl_tot = 0, wl_bad = 0;
always @(posedge clk) if (ce) begin
    if (u_core.u_biu.r_run && u_core.u_biu.r_cur_wr &&
        u_core.u_biu.r_ts == 3'd3 && u_core.u_biu.r_cur_bs == 3'd6) begin
        wl_tot <= wl_tot + 1;
        if (!(t_state == ST_T3 && is_write_cycle && lat_type == BS_MEMW)) begin
            wl_bad <= wl_bad + 1;
            if (wl_bad < 40)
                $display("[wlost %m] WRITE PERSA biu cur_addr=%05h cur_data=%04h | tracker t=%0d lat_type=%0d is_wr=%b addr_lat=%05h",
                         u_core.u_biu.r_cur_addr, u_core.u_biu.r_cur_data,
                         t_state, lat_type, is_write_cycle, addr_lat);
        end
    end
end
// ── PRECONDIZIONE DELLA WRITE PERSA: annuncio MEMW mascherato sul pin ────
// I rami inta_preview / vector_follow_preview / flush_fast del mux di BS
// (v30u_biu:994-999) hanno priorita' su `display`.  Se uno di essi e' alto
// mentre sta in piedi l'annuncio di un MEMW, il pin BS mostra INTA/MEMR/CODE:
// il tracker leggerebbe quel clock e tipizzerebbe il ciclo come lettura ->
// write mai emessa.  Qui conto la SOLA precondizione (molto piu' frequente
// del danno visibile, che richiede in piu' di cadere su una cella mai
// riscritta): se il contatore resta 0 su milioni di cicli, il meccanismo
// non e' quello; se sale, [wlost] dice quali write si perdono davvero.
integer msk_n = 0;
always @(posedge clk) if (ce) begin
    if (u_core.u_biu.r_cmt_valid && u_core.u_biu.r_cmt_bs == 3'd6 &&
        BS != BS_MEMW) begin
        msk_n <= msk_n + 1;
        if (msk_n < 24)
            $display("[bsmask %m] annuncio MEMW mascherato: bs_pin=%0d cdage=%0d cmt_addr=%05h",
                     BS, u_core.u_biu.r_cdage, u_core.u_biu.r_cmt_addr);
    end
end
integer wp_win = 0;
always @(posedge clk) begin
    wp_win <= wp_win + 1;
    if (wp_win == 4_000_000) begin
        wp_win <= 0;
        $display("[wprov %m] write=%0d mismatch=%0d t1_lag=%0d | biu_wr=%0d perse=%0d mask=%0d", wp_tot, wp_bad, wp_t1_lag, wl_tot, wl_bad, msk_n);
    end
end
`endif

`ifdef V30_SIM_PROBES
// Probe boot-gate: SOLO con --define V30_SIM_PROBES=1 (mai in Quartus, e
// spente di default anche in sim). Primi cicli di bus + heartbeat + raw pin.
integer dbg_cycles = 0;
integer dbg_clks = 0;
integer dbg_raw = 0;
integer dbg_inta_n = 0;
integer dbg_pc_s = 0;
integer dbg_boot_n = 0;
reg [19:0] last_code_addr = 20'h0;
integer dbg_rd_s = 0;
integer dbg_cyc = 0;
integer dbg_wn = 0;
reg [15:0] dbg_pc_prev = 16'h0;
integer dbg_ins = 0;
integer dbg_pollm = 0;
integer dbg_polls = 0;
integer dbg_fr = 0;
integer dbg_anat = 0;
reg dbg_arm = 1'b0;
integer dbg_isr_t0 = 0;
reg dbg_isr_on = 1'b0;
integer dbg_grd = 0;
integer dbg_in = 0;
integer dbg_path = 0;
integer dbg_f422 = 0;
reg dbg_inta_d;
always @(posedge clk) begin
    dbg_clks <= dbg_clks + 1;
    if (ce && t_state == ST_T1 && dbg_cycles < 48) begin
        dbg_cycles <= dbg_cycles + 1;
        $display("[v30 %m] cyc%0d clk%0d bs=%0d addr=%05h be=%b", dbg_cycles, dbg_clks, lat_type, addr_lat, be_lat);
    end
    if (ce && t_state == ST_T3 && is_read_cycle && dbg_cycles < 48)
        $display("[v30 %m]   T3 rd data=%04h ready=%b", rdata_q, ready);
    // RAW pin dump: BS/AD/RD_N direttamente dai pin a ogni CE (bypassa il
    // tracker) — distingue "core deragliato" da "tracker che decodifica male".
    if (ce && dbg_raw < 120) begin
        dbg_raw <= dbg_raw + 1;
        $display("[v30raw %m] ce#%0d t=%0d BS=%0d ADDR=%05h rdn=%b rdy=%b rdyq=%b qs=%0d", dbg_raw, t_state, BS, ADDR_O, RD_N, ready, ready_q, qs_dbg);
    end
    // CAMPIONAMENTO PC: stampa l'indirizzo di fetch ogni 256 cicli CPU.
    // Serve a vedere DOVE passa il tempo: un ciclo di attesa (polling) si
    // riconosce perche' pochi indirizzi dominano il campione.
    // PRIMI FETCH DAL RESET: indirizzo e dato, per vedere dove deraglia
    if (ce && t_state == ST_T3 && lat_type == BS_CODE && dbg_boot_n < 24) begin
        dbg_boot_n <= dbg_boot_n + 1;
        $display("[boot %m] #%0d fetch @%05h -> %04h", dbg_boot_n, addr_lat, rdata_q);
    end
    if (ce && t_state == ST_T1 && lat_type == BS_CODE) last_code_addr <= addr_lat;
    if (ce && t_state == ST_T1 && lat_type == BS_CODE) begin
        dbg_pc_s <= dbg_pc_s + 1;
        if (dbg_pc_s % 64 == 0) $display("[pc %m] %05h", addr_lat);
    end
    // e COSA legge: se sta aspettando qualcosa, un indirizzo dati domina
    if (ce && t_state == ST_T1 && lat_type == BS_MEMR) begin
        dbg_rd_s <= dbg_rd_s + 1;
        if (dbg_rd_s % 64 == 0) $display("[rd %m] %05h", addr_lat);
    end
    // COSTO IN CICLI CPU del codice di boot (deterministico, identico a MAME).
    // Stampo il numero di CE (= clock CPU) alla n-esima scrittura nella finestra
    // 00400-0040F: gli stessi eventi misurati in MAME con raiden_cycles.lua.
    // Confrontando i DELTA si vede se la nostra CPU costa piu' cicli della sua.
    if (ce) dbg_cyc <= dbg_cyc + 1;
    if (ce && t_state == ST_T3 && lat_type == BS_MEMW &&
        addr_lat >= 20'h00400 && addr_lat <= 20'h0040F) begin
        dbg_wn <= dbg_wn + 1;
        if (dbg_wn < 40)
            $display("[cyc %m] #%02d addr %05h dato %04h cicli=%0d", dbg_wn+1, addr_lat, dout_lat, dbg_cyc);
    end
`ifdef V30_BACKDOOR
    // TRACCIA PER ISTRUZIONE: l'ucore espone in dbg_regs l'IP dell'istruzione
    // RITIRATA (slot pc = bit 207:192). Ogni volta che cambia e' un'istruzione
    // completata: stampo ciclo + IP nella stessa finestra tracciata in MAME,
    // cosi' il costo istruzione per istruzione si confronta uno a uno.
    dbg_pc_prev <= core_dbg_regs[207:192];
    if (core_dbg_regs[207:192] != dbg_pc_prev) begin
        dbg_ins <= dbg_ins + 1;
        if (dbg_cyc >= 229800 && dbg_cyc <= 232600)
            $display("[ins %m] %0d %04h", dbg_cyc, core_dbg_regs[207:192]);
    end
    // MARGINE DI CPU PER FRAME, con il metro del gioco stesso: quante volte
    // rilegge il flag di attesa. Main = 00410, Sub = 04008 (RAM condivisa).
    // Riferimento MAME da avvio: main ~4106/4186 per frame (con calo a ~3 ogni
    // 6 frame), sub ~822/2992 alternati. Meno letture = meno margine.
    // ANATOMIA DEL GIRO: stampo lo stato del bus a OGNI ce per un tratto del
    // frame 85. Cosi' si contano i cicli di bus (T1), il loro tipo e i clock
    // in cui il bus e' fermo (TI): il ciclo di attesa DEVE costare 4 fetch +
    // 1 lettura = 5 cicli di bus = 20 clock. Tutto il resto e' esecuzione.
    // aggancio: dal primo fetch del ciclo di attesa dopo il frame 78
    if (ce && t_state == ST_T1 && lat_type == BS_CODE && addr_lat == 20'hf2d82
        && dbg_fr >= 78) dbg_arm <= 1'b1;
    if (ce && dbg_arm && dbg_anat < 160) begin
        dbg_anat <= dbg_anat + 1;
        $display("[anat %m] c=%0d t=%0d bs=%0d a=%05h", dbg_cyc, t_state, lat_type, addr_lat);
    end
    if (ce && t_state == ST_T1 && lat_type == BS_MEMR && addr_lat == 20'h00410) begin
        dbg_pollm <= dbg_pollm + 1;
        // costo di UN giro del ciclo di attesa: in MAME e' 23 cicli esatti
        // (cmp [410h],5 = 10, blt = 13). Se da noi costa di piu', a parita' di
        // margine il conteggio delle letture scende: va scalato.
        if (dbg_fr >= 80 && dbg_fr <= 100 && dbg_pollm < 6)
            $display("[loop %m] lettura 00410 al ciclo %0d", dbg_cyc);
    end
    if (ce && t_state == ST_T1 && lat_type == BS_MEMR && addr_lat == 20'h04008) begin
        dbg_polls <= dbg_polls + 1;
        if (dbg_fr >= 80 && dbg_fr <= 100 && dbg_polls < 6)
            $display("[loop %m] lettura 04008 al ciclo %0d", dbg_cyc);
    end
    if (dbg_clks % 1346570 == 0 && dbg_clks != 0) begin
        dbg_fr <= dbg_fr + 1;
        $display("[wf %m] frame %0d: 00410 %0d  04008 %0d  istr %0d",
                 dbg_fr+1, dbg_pollm, dbg_polls, dbg_ins);
        dbg_pollm <= 0; dbg_polls <= 0; dbg_ins <= 0;
    end
`endif
    // [422h] = flag che abilita il lavoro del frame. In MAME viene messo a 1
    // una volta sola da EECE7 e da li' in poi la ISR lavora. Se da noi la
    // scrittura non arriva o non resta, il gioco salta il lavoro per sempre.
    if (ce && t_state == ST_T3 && lat_type == BS_MEMW && addr_lat == 20'h00422)
        $display("[f422 %m] SCRITTO %04h (ciclo %0d)", dout_lat, dbg_cyc);
    if (ce && t_state == ST_T3 && lat_type == BS_MEMR && addr_lat == 20'h00422
        && dbg_f422 < 12) begin
        dbg_f422 <= dbg_f422 + 1;
        $display("[f422 %m] letto %04h (ciclo %0d)", rdata_q, dbg_cyc);
    end
    // INGRESSI E DIP visti dal main: 0E000 = P1_P2, 0E002 = DSW. Se in sim
    // arrivano sbagliati (attivi bassi!), il gioco va in un'altra modalita' e
    // tutto il confronto con MAME non vale niente.
    if (ce && t_state == ST_T3 && lat_type == BS_MEMR &&
        (addr_lat == 20'h0E000 || addr_lat == 20'h0E002) && dbg_in < 10) begin
        dbg_in <= dbg_in + 1;
        $display("[in %m] %05h = %04h", addr_lat, rdata_q);
    end
    // PERCORSO DENTRO LA ISR: stampo ogni fetch di codice dal riconoscimento
    // dell'interrupt in poi, per un frame. Confrontato con la traccia di MAME
    // dice l'istruzione esatta in cui i due si separano.
    if (dbg_isr_on && ce && t_state == ST_T1 && lat_type == BS_CODE && dbg_path < 400
        && dbg_fr >= 78) begin
        dbg_path <= dbg_path + 1;
        $display("[path %m] %05h", addr_lat);
    end
    // GUARDIA DELLA ISR: che valore legge la CPU a [400h]. Se e' diverso da 0
    // la ISR salta tutto il lavoro del frame (bne FD8C8).
    if (ce && t_state == ST_T3 && lat_type == BS_MEMR && addr_lat == 20'h00400
        && dbg_grd < 16) begin
        dbg_grd <= dbg_grd + 1;
        $display("[guard %m] [400h] = %04h (ciclo %0d)", rdata_q, dbg_cyc);
    end
    // DURATA DELLA ISR DI VBLANK: dal riconoscimento dell'interrupt fino al
    // rientro nel ciclo di attesa. In MAME sono ~73.380 cicli (44% del frame):
    // e' il lavoro vero del frame. Se da noi e' un'inezia, la ISR sta uscendo
    // subito dalla guardia di rientranza (cmp [400h],0 / bne) e il gioco non
    // fa il lavoro sul main.
    if (int_ack && !dbg_inta_d) begin dbg_isr_t0 <= dbg_cyc; dbg_isr_on <= 1'b1; end
    // FINE ISR = fetch di FD8C8 (pop ds1, comune ai due rami). NON la lettura
    // di 00410: dentro la ISR c'e' `inc [410h]`, che chiudeva la misura a meta'.
    if (dbg_isr_on && ce && t_state == ST_T1 && lat_type == BS_CODE &&
        addr_lat == 20'hfd8c8) begin
        dbg_isr_on <= 1'b0;
        $display("[isr %m] durata = %0d cicli", dbg_cyc - dbg_isr_t0);
    end
    // conteggio ESATTO: un solo evento per interrupt (fronte), riportato per frame
    dbg_inta_d <= int_ack;
    if (int_ack && !dbg_inta_d) dbg_inta_n <= dbg_inta_n + 1;
    if (dbg_clks % 1350000 == 0 && dbg_clks != 0) begin
        $display("[irq %m] interrupt nell'ultimo frame = %0d (atteso 1)", dbg_inta_n);
        dbg_inta_n <= 0;
    end
    if (dbg_clks != 0 && dbg_clks % 500000 == 0)
        $display("[v30 %m] hb clk%0d t=%0d bs=%0d addr=%05h rst=%b rdy=%b", dbg_clks, t_state, lat_type, addr_lat, reset, ready);
end
`endif

endmodule
