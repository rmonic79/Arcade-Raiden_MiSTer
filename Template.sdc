derive_pll_clocks
derive_clock_uncertainty

# core specific constraints

# ============================================================
# Audio subsystem runs at ce_4m (96MHz/24 = 4MHz)
# All internal paths are CE-gated with 24 cycles between active edges.
# Multicycle = 24 for setup, 23 for hold.
# Target everything under Raiden_audio_z80 module (jt51, T80, mixer, jt6295...).
# FIX residuo Darius: il target era *darius_audio_z80* (nome pre-fork) che NON
# matchava il modulo reale Raiden_audio_z80 → multicycle audio MAI applicato →
# path audio valutati single-cycle → timing negativo. Trovato anche in altri core.
# ============================================================
set_multicycle_path -setup -from [get_registers {*Raiden_audio_z80*}] -to [get_registers {*Raiden_audio_z80*}] 24
set_multicycle_path -hold  -from [get_registers {*Raiden_audio_z80*}] -to [get_registers {*Raiden_audio_z80*}] 23

# ============================================================
# Raiden V30 CPUs (core nuovo cycle-accurate v30_core: biu+eu).
# I registri interni del V30 avanzano su CE (raiden_ce_gen: 10 MHz = 1 ogni 8
# clk_sys), quindi ogni path reg->reg CE->CE ha ~8 clk fisici per assestarsi.
# Worst reale: v30_eu|opc -> v30_eu|psw (~18 ns), valutato single-cycle -> -6.5.
# Multicycle 2 (25 ns > 18) chiude ed e' CONSERVATIVO (2 << 8) -> NON masking.
# Target: registri DENTRO v30_core.
# ============================================================
# La vecchia versione era una COPERTA (*v30_core:u_core* verso se stesso, senza
# distinzioni) e nel 2026-08-14 l'avevo commentata per non nascondere niente.
# Il vincolo CE onesto c'e' gia' piu' sotto, nella forma DERIVATA di upstream
# (collezione v30u_eu + v30u_biu + porte della ROM di microcodice, con t1_half2
# tenuto FUORI): vedi il blocco "Raiden V30 CE-gated" e non duplicarlo qui.

# addr_neg/ube_neg (adapter v30_bus): indirizzo catturato sul NEGEDGE gated da
# ce_half. QUI C'ERA una multicycle -setup 2 / -hold 1, giustificata col vecchio
# contratto C-b (ce_half = ce+2 clk -> "finestra vera" di 2.5 clk).
#
# RIMOSSA 2026-08-15 allineando alla correzione upstream dell'ucore
# (wickerwaka/nec_test 5f63289d "ce/ce_half CONTRACT CORRECTION", user ruling
# 2026-08-13): C-b e' CANCELLATA, resta solo C-a (ce e ce_half mai sullo stesso
# clock), e le DUE eccezioni cross-phase vanno tolte perche' quei path sono da
# 1.0 periodo -- il controllo di default -- e un -setup 2 li' e' un PASS FALSO.
#
# E' la stessa classe di bugia che aveva gia' rotto il punteggio su questo core
# (multicycle 9 falsa su *u_cpu*cpu*): STA verde e silicio che sbaglia la
# cattura di indirizzo/byte-enable -> scritture all'indirizzo o sulla corsia
# sbagliata. Se ora questi path falliscono si ritempora il latch: NON si
# rimette l'eccezione.

# READY path: main/sub_rq_active -> cpu_ready -> READY del core. READY e' un input
# di WAIT-STATE: se arriva in ritardo il core inserisce solo un Tw in piu' (lo
# tollera by-design, nessun errore ne' freeze). Quindi READY->core e' latency-
# tolerant -> multicycle 2 SICURO (NON una maschera dannosa: un READY tardivo non
# causa dato/istruzione sbagliata, solo un ciclo di attesa). Nessuna ce-sync RTL.
# RIMOSSO 2026-08-14 (audit anti-bugie). Scritto quando rq_active pilotava
# READY (input di wait-state, tollerante al ritardo). OGGI rq_active pilota lo
# STALL del CE in raiden_ce_gen, cioe' l'ENABLE dei registri del core: un
# enable in ritardo fa avanzare un T-state quando non dovrebbe (dato campionato
# nell'istante sbagliato). Su un enable il multicycle NON e' tollerante: e' una
# bugia. Deve chiudere single-cycle.
# set_multicycle_path -setup 2 -from [get_registers {*rq_active*}] -to [get_registers {*v30_core:u_core*}]
# set_multicycle_path -hold  1 -from [get_registers {*rq_active*}] -to [get_registers {*v30_core:u_core*}]

# Video (pre-esistente, non V30): vpos (Raiden_video_timing) avanza a rate di RIGA
# (una volta per scanline = stabile per centinaia di clk_sys) -> vpos -> tile_col
# (Raiden_tile_layer) e' multicycle di fatto (come i path hpos sotto). Multicycle 4
# conservativo -> REALE (vpos stabile ben oltre 4 clk).
set_multicycle_path -setup 4 -from [get_registers {*Raiden_video_timing*vpos*}] -to [get_registers {*Raiden_tile_layer*tile_col_pf*}]
set_multicycle_path -hold  3 -from [get_registers {*Raiden_video_timing*vpos*}] -to [get_registers {*Raiden_tile_layer*tile_col_pf*}]

# ============================================================
# Video timing → palette RAM address.
# hpos avanza a ce_pix (clk_sys/16), quindi resta STABILE per 16 clk_sys.
# Il path hpos → composite_pen → pal_b_addr → palette RAM portb ha 16 clk
# reali per stabilizzarsi (la sorgente hpos non cambia tra due ce_pix).
# Quartus lo valuta single-cycle (worst -1.3 ns) ma e' un multicycle di
# fatto. Multicycle 4 (conservativo, 4 << 16) chiude il timing senza
# nascondere path realmente lenti.
# ============================================================
set_multicycle_path -setup -from [get_registers {*Raiden_video_timing*hpos*}] -to [get_registers {*raiden_video_subbus*pal_*}] 4
set_multicycle_path -hold  -from [get_registers {*Raiden_video_timing*hpos*}] -to [get_registers {*raiden_video_subbus*pal_*}] 3
# stesso path verso il registro pal_b_addr (indirizzo palette lato emu) e
# ctrl_flipscreen (stabile per frame): anch'essi hpos/ce_pix-paced.
set_multicycle_path -setup -from [get_registers {*Raiden_video_timing*hpos*}] -to [get_registers {*pal_b_addr*}] 4
set_multicycle_path -hold  -from [get_registers {*Raiden_video_timing*hpos*}] -to [get_registers {*pal_b_addr*}] 3
# ctrl_reg[6] = flip_screen (DIP): STABILE per l'intero frame (cambia solo su
# scrittura CPU rara). Il path flip_screen → hpos_for_read (255-hpos) →
# pal_b_addr era il worst (-0.238 ns) valutato single-cycle. Multicycle 4.
set_multicycle_path -setup -from [get_registers {*Raiden_main_top*ctrl_reg[6]*}] -to [get_registers {*pal_b_addr*}] 4
set_multicycle_path -hold  -from [get_registers {*Raiden_main_top*ctrl_reg[6]*}] -to [get_registers {*pal_b_addr*}] 3

# ============================================================================
# Rilettura savestate (*_ss_rdata) <- FSM del controller savestate.
# Protocollo ssbus (rtl/common/savestate_ram.sv): mentre `ssbus.access` e' alto
# l'indirizzo resta STABILE e la `read_response` viene emessa solo dopo
# `read_delay`, cioe' un clock dopo. La cattura che conta e' quindi la SECONDA:
# la prima non viene mai letta. Setup 2 / hold 1 DERIVATI dal protocollo.
# (misurato: senza, il path FSM -> scroll_ss_rdata sta a -1.617 ns)
set ss_ctrl_regs [get_registers -nowarn {*save_state_data*memory_stream*state[*]}]
set ss_rdata_regs [get_registers -nowarn {*|*_ss_rdata[*]}]
if {[get_collection_size $ss_ctrl_regs] > 0 && [get_collection_size $ss_rdata_regs] > 0} {
    set_multicycle_path -setup 2 -from $ss_ctrl_regs -to $ss_rdata_regs
    set_multicycle_path -hold  1 -from $ss_ctrl_regs -to $ss_rdata_regs
    post_message -type info         "Template.sdc: ss readback 2/1 su [get_collection_size $ss_rdata_regs] registri"
}

# ============================================================================
# V30 UCORE — multicycle CE-paced (da nec_test/hdl/nec_test.sdc, derivazione
# completa nel file upstream; contratto portabile C-a/C-b/C-c: ce e ce_half
# mai nello stesso clk, assert >=2 clk di distanza, >=1 ce_half tra due ce.
# Il nostro train (raiden_ce_gen /8, ce_half=ce+1clk) lo soddisfa.
#   ce -> ce           : setup 4 / hold 3
#   ce -> ce_half      : setup 2 / hold 1   (t1_half2 = UNICO flop negedge)
#   ce_half -> ce      : setup 3 / hold 2
# I path di confine (bridge/ssbus dentro, uscite core verso fabric) restano
# single-cycle: i loro launch reg non sono CE-gated. NON estendere a coperta.
# ============================================================================
# ─────────────────────────────────────────────────────────────────────────
# CONTRATTO ce/ce_half (nec_test, dichiarato "operating envelope" 2026-08-13).
# Clausole verificate dal gate upstream hdl/tb/ce_contract_check.sv:
#   C-a: ce e ce_half MAI sullo stesso fabric clock
#   C-b: MAI su clock adiacenti (almeno 1 clock vuoto in mezzo)
#   C-c: MAI due ce senza un ce_half in mezzo
# NOSTRE FASI (cpu_v30_bridge + raiden_ce_gen):
#   ce a 0, ce_half a +2, ce successivo a >= +5 (CE_GAP_MIN, anche in catch-up)
#   -> C-a ok, C-b ok (1 clock vuoto tra ce e ce_half, 2 tra ce_half e ce),
#      C-c ok. Prima ce_half era a +1: violava C-b E rendeva FALSO il
#      multicycle 2 qui sotto (1 solo periodo reale) -> STA verde +1.18 con il
#      silicio che violava di -11.3 ns = bug punteggio.
# ─────────────────────────────────────────────────────────────────────────
# La collezione include ANCHE le porte interne della ROM di microcodice.
# ucdecode/ucrom sono letture combinatorie in RTL; Quartus le mappa in M10K
# (OPERATION_MODE=ROM) e vi ASSORBE il registro di indirizzo a monte, che e' un
# registro dell'EU -> gated da CE come tutto il core, quindi la multicycle e'
# onesta anche li'. Ma get_registers NON restituisce le porte delle memorie:
# serve get_keepers. Senza, quei path restano single-cycle -> setup -5.886 ns
# (build 2026-08-16 00:25, TUTTI i peggiori dentro ucdecode del sub).
# Upstream non lo vede perche' chiude a 50 MHz: a 20 ns quel path passa lo
# stesso, a 12.5 ns no.
# get_keepers e NON get_registers su TUTTI i termini: add_to_collection rifiuta
# di mescolare tipi ("requires type ( reg ), but found type kpr") e i nodi che
# ci servono -- le porte interne dell'M10K della ROM di microcodice -- sono
# keeper, non register. Tipo uniforme = collezione valida.
# Il pattern half usa t1_half2* per prendere anche il t1_half2~DUPLICATE che il
# fitter crea sul sub: upstream lo tiene stretto per non ri-basare le proprie
# misure, ma se il duplicato finisce fra i vincolati si prende una multicycle
# su un arco cross-phase da 1.0 periodo = PASS falso.
set v30u_regs [add_to_collection \
                   [get_keepers -nowarn {*|v30u_eu:*|*}] \
                   [get_keepers -nowarn {*|v30u_biu:*|*}]]
# Registri dell'ADATTATORE che avanzano sullo STESSO ce del core (tutti dentro
# `if (ce)` in v30_bus.sv, verificato riga per riga). Il lancio e' CE-gated e la
# cattura pure: la finestra reale e' 8 clk, dichiararne 4 e' conservativo --
# stessa identica derivazione gia' applicata ai registri interni del core.
# Elencati UNO PER UNO: non e' una coperta su *v30_bus*, che prenderebbe anche
# i registri free-running (rdata_q, bs_q, ube_n_q) dove sarebbe una bugia.
set v30bus_ce [get_keepers -nowarn {*|v30_bus:*|addr_lat[*] *|v30_bus:*|be_lat[*]                                     *|v30_bus:*|dout_lat[*] *|v30_bus:*|t_state[*]                                     *|v30_bus:*|lat_type[*] *|v30_bus:*|is_read_cycle                                     *|v30_bus:*|is_write_cycle *|v30_bus:*|addr_valid *|v30_bus:*|a0_lat                                     *|v30_bus:*|inta_prev *|v30_bus:*|inta_second}]
if {[get_collection_size $v30bus_ce] > 0} {
    set v30u_regs [add_to_collection $v30u_regs $v30bus_ce]
}
set v30u_half [get_keepers -nowarn {*|v30u_biu:*|t1_half2*}]
# Nel bus de-muxato t1_half2 NON ESISTE: la collezione half e' vuota e
# remove_from_collection con una collezione vuota da errore. Guardia esplicita.
if {[get_collection_size $v30u_half] > 0} {
    set v30u_ce [remove_from_collection $v30u_regs $v30u_half]
} else {
    set v30u_ce $v30u_regs
}
if {[get_collection_size $v30u_ce] > 0} {
    # 4/3 e non il 2/1 di upstream: loro derivano 2 dal treno MINIMO legale
    # (div 2). Il NOSTRO treno e' fisso a 1 ce ogni 8 clk e non scende sotto
    # CE_GAP_MIN=5 nemmeno in catch-up -> 4 periodi (50 ns) restano dentro la
    # finestra fisica reale (>=62.5 ns). Vero, e piu' rilassato per l'Fmax.
    set_multicycle_path -setup 4 -from $v30u_ce -to $v30u_ce
    set_multicycle_path -hold  3 -from $v30u_ce -to $v30u_ce
    post_message -type info \
        "Template.sdc: CE multicycle 4/3 su [get_collection_size $v30u_ce] registri v30u ce-gated"
}
# ⚠ LE DUE ECCEZIONI CROSS-PHASE SONO CANCELLATE, NON MANCANTI.
# Erano `ce -> ce_half` e `ce_half -> ce` a 2/1. Upstream le ha cancellate con
# la correzione del contratto (nec_test 5f63289d, user ruling 2026-08-13) e lo
# scrive esplicitamente: "DO NOT RESTORE THEM". Motivo: sul contratto corretto
# gli enable ADIACENTI sono legali, quindi quegli archi valgono 1.0 periodo --
# che E' il controllo di default -- e un -setup 2 li' e' un PASS falso di un
# fattore due, nella direzione ottimista, proprio sull'arco che lo split della
# collezione esiste per proteggere.
# NON riderivarle "perche' il nostro treno e' piu' largo": si segue upstream.
if {[get_collection_size $v30u_half] > 0} {
    post_message -type info \
        "Template.sdc: [get_collection_size $v30u_half] flop ce_half TENUTI FUORI dalla\
         multicycle CE; archi cross-phase single-cycle, come upstream"
}

# status[] HPS -> Raiden_audio_z80: SOLO selettori volume OSD (quasi-statici:
# cambiano su azione utente nel menu; il pause arriva da paused_safe, registro
# derivato, NON da status raw -> questi archi non portano controllo critico).
# La catena gain_resolve*mix*softclip (3 moltiplicatori) e' ~21.7ns: single-cycle
# non chiude ne' deve — al cambio slider al peggio 1 sample transitorio.
set_false_path -from [get_registers {*|hps_io:*|status[*]}] -to [get_registers {*|Raiden_audio_z80:*|*}]

# v30u (CE-gated) -> capture free-running del v30_bus (ad_q/bs_q/ube_n_q):
# ricatturano OGNI clk ma il valore e' consumato solo a istanti ce-paced
# (dout_lat in T2/T3, commit su transizioni T-state = >=8 clk dal lancio CE).
# La cattura a L+1 non e' mai letta -> setup 2 / hold 1 derivato dal NOSTRO
# train fisso /8 (non dal contratto di portabilita' upstream, che qui non serve).
set v30bus_cap [get_registers -nowarn {*|v30_bus:*|ad_q[*] *|v30_bus:*|bs_q[*] *|v30_bus:*|ube_n_q}]
if {[get_collection_size $v30u_ce] > 0 && [get_collection_size $v30bus_cap] > 0} {
    # 2026-08-17: la cattura ora avviene a +4 clk dal CE (v30_bus, ce_pipe[3]),
    # quindi l'arco ha 4 periodi REALI e il vincolo li dichiara. Prima erano 2
    # dichiarati su 1 disponibile: MISURATO -8.993 ns una volta rimossa la
    # copertura (ucrom -> bs_q). Ora non maschera niente.
    set_multicycle_path -setup 4 -from $v30u_ce -to $v30bus_cap
    set_multicycle_path -hold  3 -from $v30u_ce -to $v30bus_cap
    post_message -type info \
        "Template.sdc: boundary 2/1 su [get_collection_size $v30bus_cap] capture reg v30_bus"
}

# RIMOSSO 2026-08-16: qui c'era un set_false_path EU -> addr_neg/ube_neg.
# Quei registri NON ESISTONO PIU' (cattura negedge eliminata col bus
# de-muxato): l'eccezione non agganciava piu' nulla e restava a dire il falso
# a chi legge. Un false_path verso registri inesistenti e' rumore, e il rumore
# nell'SDC e' esattamente come nascono le bugie che costano mesi.

# ssbus -> registri core (write restore): v30_bus ora tiene addr/dato stabili
# 2 clk PRIMA di SS_WE (ss_wr_delay) -> il cono di decode (~30 ns) ha 3 periodi
# VERI. La destinazione cattura solo al clk di SS_WE (3o) -> setup 3 / hold 2
# derivati, non maschera.
set ss_addr_regs [get_registers -nowarn {*|v30_core:*|ss_addr_q[*]}]
if {[get_collection_size $ss_addr_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    # 4/3 (era 3/2) dopo l'allungamento della staging in v30_bus.sv: addr/dato
    # restano stabili 3 periodi PRIMA del capture buono, che ora e' il 4o fronte
    # di SS_WE. Derivato dalla staging, non assunto.
    set_multicycle_path -setup 4 -from $ss_addr_regs -to $v30u_regs
    set_multicycle_path -hold  3 -from $ss_addr_regs -to $v30u_regs
}

# ss_we_q (interno v30_core) -> regfile v30u: fanout WE ~18ns, single-cycle non
# chiude a 80MHz (upstream 45MHz non lo vede). SS_WE ora largo 2 clk in v30_bus
# (write tutte idempotenti, verificato) -> il capture buono e' il 2o, con WE e
# decode stabili da 1 clk -> setup 2 / hold 1 DERIVATO.
set ss_we_regs [get_registers -nowarn {*|v30_core:*|ss_we_q}]
if {[get_collection_size $ss_we_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 2 -from $ss_we_regs -to $v30u_regs
    set_multicycle_path -hold  1 -from $ss_we_regs -to $v30u_regs
}

# ss_we_q e ss_wdata_q -> regfile v30u: cono ~39 ns (WE verso regfile + blocchi
# M10K della ROM di microcodice). SS_WE ora largo 4 clk (v30_bus, write tutte
# idempotenti): il capture valido e' il 4o fronte, con WE, decode e dato stabili
# da 3 periodi -> setup 4 / hold 3 DERIVATI dalla staging in v30_bus.sv, non
# assunti. Con 3 periodi (37.5 ns) mancavano 1.737 ns: misurato, non stimato.
set ss_wd_regs [get_registers -nowarn {*|v30_core:*|ss_wdata_q[*]}]
if {[get_collection_size $ss_we_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 4 -from $ss_we_regs -to $v30u_regs
    set_multicycle_path -hold  3 -from $ss_we_regs -to $v30u_regs
}
if {[get_collection_size $ss_wd_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 3 -from $ss_wd_regs -to $v30u_regs
    set_multicycle_path -hold  2 -from $ss_wd_regs -to $v30u_regs
}

# irq_pending (bridge, clk-paced) -> core: INT e' ASINCRONO by-design (il V30
# reale lo campiona per-istruzione): arrivare 1 CE dopo = interrupt esterno
# leggermente posticipato, indistinguibile e corretto. Stesso argomento del
# READY (righe sopra): latency-tolerant -> 2/1 SICURO, non maschera.
# RISTRETTO 2026-08-16. Il bersaglio era `*v30_core:u_core*` = TUTTI i registri
# del core: una COPERTA, la stessa forma della multicycle 9 falsa che nascose il
# bug punteggio la prima volta. L'argomento "INT e' latency-tolerant" vale SOLO
# per la pipeline che campiona il pin (`int_p`, v30u_eu:302 "int_p[k] is the
# level of clock c-1-k"). NON vale per l'uso VIVO dello stesso pin:
# v30u_eu:2345 `assign flush_int_live = pin_int;` -- quello e' combinatorio e
# deve chiudere in UN periodo. Dandogli 2 si mascherava una violazione su un
# percorso che decide un flush.
# irq_pending e' clockato a clk LIBERO (Raiden_main_top:88-89, set su vblank e
# clear su cpu_irq_active), quindi puo' cambiare 1 clock prima della cattura:
# 2 periodi non glieli garantisce nessuno.
# 2026-08-17. La sorgente verso il core ora e' irq_pending_ce, registrata SU ce
# nei due top: lancio ce-paced -> cattura ce-paced, gap CE >= 5 clk
# (raiden_ce_gen CE_GAP_MIN) = 62.5 ns per un percorso che MISURA 15.885 ns.
# Il 4/3 e' lo stesso vincolo VERO usato per tutto l'ucore, non una coperta.
# Il registro irq_pending grezzo non entra piu' nel core: nessuna eccezione.
# MOTIVO (misurato sulla build 15:05, coperta rimossa):
#   -3.385 ns  irq_pending -> v30u_biu:u_biu|r_rq_seg[1][0]
# cioe' il cono dell'INT entra nel SEGMENTO della richiesta del BIU (indirizzo
# fisico del ciclo). A clock libero violava di 3.385 ns e la vecchia coperta
# `irq_pending -> *v30_core:u_core*` lo dichiarava chiuso: STA verde, silicio
# che sbaglia l'indirizzo. Stessa forma del bug punteggio storico.
set irq_ce_regs [get_registers -nowarn {*irq_pending_ce*}]
if {[get_collection_size $irq_ce_regs] > 0 && [get_collection_size $v30u_regs] > 0} {
    set_multicycle_path -setup 4 -from $irq_ce_regs -to $v30u_regs
    set_multicycle_path -hold  3 -from $irq_ce_regs -to $v30u_regs
}

# ============================================================================
# Filtro audio di uscita (IIR_filter u_audio_lpf).
# Tutti i suoi registri interni avanzano su `ce` (sys/iir_filter.v: gli always
# sono `@(posedge clk) if (ce)`); solo il registro di uscita usa sample_ce.
# flt_ce e' generato con l'accumulatore del framework (cnt += 7.056e6*2 contro
# 80e6): gli intervalli fra due ce sono 5 o 6 clock, mai meno di 5. Lancio e
# cattura sono quindi entrambi ce-paced e la finestra fisica reale e' >= 62,5 ns
# contro i 22,7 ns che il cono moltiplicatore+accumulatore richiede.
# Multicycle 4/3 = 50 ns: vero e conservativo (4 < 5).
# ============================================================================
# Il registro di uscita `out` NON e' ce-gated: avanza su sample_ce (48 kHz,
# sys/iir_filter.v:142). Il suo arco di ingresso parte da out_l/out_r (ce) e
# arriva a out (sample_ce): due enable SCORRELATI, quindi la cattura puo' cadere
# anche 1 clock dopo il lancio. Un multicycle li' sarebbe FALSO, quindi `out` e'
# tenuto FUORI dalla collezione e resta a ciclo singolo (e chiude: e' una copia
# registro-registro). Tutto il resto (inp, inp_m, ch, out_l/r/m e gli intreg dei
# tre tap) e' dentro `if (ce)`.
set lpf_all [get_registers -nowarn {*IIR_filter:u_audio_lpf|*}]
set lpf_out [get_registers -nowarn {*IIR_filter:u_audio_lpf|out[*]}]
if {[get_collection_size $lpf_all] > 0} {
    if {[get_collection_size $lpf_out] > 0} {
        set lpf_regs [remove_from_collection $lpf_all $lpf_out]
    } else {
        set lpf_regs $lpf_all
    }
    if {[get_collection_size $lpf_regs] > 0} {
        set_multicycle_path -setup 4 -from $lpf_regs -to $lpf_regs
        set_multicycle_path -hold  3 -from $lpf_regs -to $lpf_regs
        post_message -type info             "Template.sdc: CE multicycle 4/3 su [get_collection_size $lpf_regs] registri del filtro audio (out escluso)"
    }
}
