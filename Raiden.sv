// SPDX-License-Identifier: GPL-3.0-or-later
//
// Raiden (Seibu Kaihatsu, 1990) - MiSTer FPGA core
// Copyright (c) Umberto Parisi (rmonic79)
// Based on the MiSTer Template by Sorgelig.

module emu
(
	input         CLK_50M,
	input         RESET,
	inout  [48:0] HPS_BUS,
	output        CLK_VIDEO,
	output        CE_PIXEL,
	output [12:0] VIDEO_ARX,
	output [12:0] VIDEO_ARY,
	output  [7:0] VGA_R,
	output  [7:0] VGA_G,
	output  [7:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,
	output        VGA_DE,
	output        VGA_F1,
	output [1:0]  VGA_SL,
	output        VGA_SCALER,
	output        VGA_DISABLE,
	input  [11:0] HDMI_WIDTH,
	input  [11:0] HDMI_HEIGHT,
	output        HDMI_FREEZE,
	output        HDMI_BLACKOUT,
	output        HDMI_BOB_DEINT,

`ifdef MISTER_FB
	output        FB_EN,
	output  [4:0] FB_FORMAT,
	output [11:0] FB_WIDTH,
	output [11:0] FB_HEIGHT,
	output [31:0] FB_BASE,
	output [13:0] FB_STRIDE,
	input         FB_VBL,
	input         FB_LL,
	output        FB_FORCE_BLANK,
`ifdef MISTER_FB_PALETTE
	output        FB_PAL_CLK,
	output  [7:0] FB_PAL_ADDR,
	output [23:0] FB_PAL_DOUT,
	input  [23:0] FB_PAL_DIN,
	output        FB_PAL_WR,
`endif
`endif

	output        LED_USER,
	output  [1:0] LED_POWER,
	output  [1:0] LED_DISK,
	output  [1:0] BUTTONS,

	input         CLK_AUDIO,
	output [15:0] AUDIO_L,
	output [15:0] AUDIO_R,
	output        AUDIO_S,
	output  [1:0] AUDIO_MIX,

	inout   [3:0] ADC_BUS,

	output        SD_SCK,
	output        SD_MOSI,
	input         SD_MISO,
	output        SD_CS,
	input         SD_CD,

	output        DDRAM_CLK,
	input         DDRAM_BUSY,
	output  [7:0] DDRAM_BURSTCNT,
	output [28:0] DDRAM_ADDR,
	input  [63:0] DDRAM_DOUT,
	input         DDRAM_DOUT_READY,
	output        DDRAM_RD,
	output [63:0] DDRAM_DIN,
	output  [7:0] DDRAM_BE,
	output        DDRAM_WE,

	output        SDRAM_CLK,
	output        SDRAM_CKE,
	output [12:0] SDRAM_A,
	output  [1:0] SDRAM_BA,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nCS,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nWE,

`ifdef MISTER_DUAL_SDRAM
	input         SDRAM2_EN,
	output        SDRAM2_CLK,
	output [12:0] SDRAM2_A,
	output  [1:0] SDRAM2_BA,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_nCS,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nWE,
`endif

	input         UART_CTS,
	output        UART_RTS,
	input         UART_RXD,
	output        UART_TXD,
	output        UART_DTR,
	input         UART_DSR,

	input   [6:0] USER_IN,
	output  [6:0] USER_OUT,

	input         OSD_STATUS
);

///////// Unused ports /////////
assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = 0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;
// DDRAM gestito da raiden_ddram (sprite ROM in DDR3) + screen_rotate (FB writes).
// Mux a fine modulo: sprite_rom_cache=read 0x30000000, rotate_fb=write 0x24000000.
assign DDRAM_CLK = clk_sys;
assign FB_FORCE_BLANK = 0;

assign VGA_SL = 0;
assign VGA_F1 = 0;
assign VGA_DISABLE = 0;
// Pause: toggle on rising edge of joy[12] (standard MiSTer pause bit)
reg pause_toggle;
reg joy_pause_prev;
always @(posedge clk_sys) begin
	if (reset) begin
		pause_toggle <= 1'b0;
		joy_pause_prev <= 1'b0;
	end else begin
		joy_pause_prev <= joy0[12] | joy1[12];
		if ((joy0[12] | joy1[12]) && !joy_pause_prev)
			pause_toggle <= ~pause_toggle;
	end
end
wire pause = pause_toggle;  // solo pad (OSD pause rimosso, pattern Darius2)

// --- VBlank-synced pause (frame-aligned, pattern Darius2 F2) ---
// pause raw asincrono → paused_safe registrato che cambia SOLO al rising edge
// vblank. Sincronizza pause boundary su tutti i moduli (CPU cen, audio cen).
// Necessario per evitare race a metà bus cycle / scanline / DDR3 transaction.
wire ss_busy;       // savestate DMA in corso — da save_state_data.busy
wire ss_mgr_pause;  // richiesta pausa dal coordinatore (raiden_ss_manager)
// Coordinamento frame-aligned (pattern BoogieWings/F2): alla pressione il manager
// alza ss_mgr_pause SUBITO; paused_safe sale al vblank successivo (confine frame);
// solo allora il manager pulsa il DMA. ss_mgr_pause resta alto per tutto il DMA
// (fino a ss_busy basso) → paused_safe resta alto → stato COERENTE per il memory_stream.
// Il memory_stream dirotta le porte BRAM al ssbus (adaptor), no race.
reg vblank_prev_safe;
reg paused_safe_r;
always @(posedge clk_sys) begin
	if (reset) begin
		vblank_prev_safe <= 1'b0;
		paused_safe_r    <= 1'b0;
	end else begin
		vblank_prev_safe <= VBlank;
		// una volta alto per SS resta alto finché il manager tiene la pausa
		// (ss_mgr_pause), che scende solo a DMA finito → paused_safe coerente.
		if (ss_mgr_pause & paused_safe_r)
			paused_safe_r <= 1'b1;
		else if (VBlank && !vblank_prev_safe)
			paused_safe_r <= pause | ss_mgr_pause;
		// RELEASE post-LOAD: il save avviene DENTRO il vblank -> il restore
		// riporta il video timing dentro il vblank e la pausa lo tiene fermo
		// -> VBlank resta alto FISSO -> il fronte di salita non arriva mai ->
		// pausa eterna (deadlock trovato in sim scene; vale anche su HW: il
		// SAVE non lo soffre perche' non ripristina il video). Rilascio anche
		// a VBlank a LIVELLO: siamo comunque al confine frame, stessa garanzia.
		else if (paused_safe_r && VBlank && !pause && !ss_mgr_pause)
			paused_safe_r <= 1'b0;
	end
end
wire paused_safe = paused_safe_r;
assign HDMI_FREEZE = 1'b0;  // overlay pause renderizzato real-time, no freeze scaler
assign HDMI_BLACKOUT = 0;
assign HDMI_BOB_DEINT = 0;

assign AUDIO_S = 1;  // signed audio
wire signed [15:0] game_audio_l, game_audio_r;
assign AUDIO_MIX = 0;

assign LED_DISK = 0;
assign LED_POWER = 0;
assign BUTTONS = 0;

//////////////////////////////////////////////////////////////////

wire [1:0] ar = status[122:121];

// Offset layer hardcodati (= valori runtime attuali, gli OSD erano bit morti
// senza voce → status sempre 0). Tolto il termine status[...] per liberare
// ALM e accelerare il fit; comportamento HW IDENTICO.
//   BG/FG X 0, Y +16 ; SPR X 0, Y -16 ; TXT X -1, Y 0
wire signed [9:0] osd_l0_xoff  = 10'sd0;
wire signed [9:0] osd_l0_yoff  = 10'sd16;
wire signed [9:0] osd_spr_xoff = 10'sd0;
wire signed [9:0] osd_spr_yoff = -10'sd16;
wire signed [9:0] osd_fg_xoff  = 10'sd0;
wire signed [9:0] osd_fg_yoff  = 10'sd16;
wire signed [9:0] osd_txt_xoff = -10'sd1;
wire signed [9:0] osd_txt_yoff = 10'sd0;
wire signed [9:0] osd_bg_xoff  = osd_l0_xoff;
wire signed [9:0] osd_bg_yoff  = osd_l0_yoff;

wire [21:0] gamma_bus;   // OSD framework <-> gamma_fast (inout, decodifica interna)
`include "build_id.v"
localparam CONF_STR = {
	"Raiden;SS3E000000:200000;",
	"-;",
	"O[106:105],Savestate Slot,1,2,3,4;",
	"R[107],Save state (Alt-F1);",
	"R[108],Restore state (F1);",
	"-;",
	"P1,Video;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[7:5],Scale,Normal,V-Integer,HV-Integer,Narrower HV-Integer;",
	"P1O[2:1],Rotate,No,CCW (TATE),CW;",
	"P1O[3],Flip 180,Off,On;",
	"P1O[19],Refresh Rate,Original 59.4Hz,60Hz;",
	"P1O[66:62],CRT H-Size,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[104:98],CRT H-Position,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,-48,-47,-46,-45,-44,-43,-42,-41,-40,-39,-38,-37,-36,-35,-34,-33,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[97:92],Analog VGA H-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,+32,+33,+34,+35,+36,+37,+38,+39,+40,+41,+42,+43,+44,+45,+46,+47,+48,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"P1O[61:56],Analog VGA V-Shift,0,+1,+2,+3,+4,+5,+6,+7,+8,+9,+10,+11,+12,+13,+14,+15,+16,+17,+18,+19,+20,+21,+22,+23,+24,+25,+26,+27,+28,+29,+30,+31,-32,-31,-30,-29,-28,-27,-26,-25,-24,-23,-22,-21,-20,-19,-18,-17,-16,-15,-14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1;",
	"-;",
	"O[18],Clean Pause,Off,On;",
	"O[30],Player,1P,2P;",
	"-;",
	"DIP;",
	"-;",
	"P3,Audio;",
	"O[83],Audio Filter,On,Off;",
	"P3O[87:84],FM Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[91:88],ADPCM Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[70:67],OKI Ch1 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[74:71],OKI Ch2 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[78:75],OKI Ch3 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[82:79],OKI Ch4 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[11:8],FM Ch1 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[15:12],FM Ch2 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[23:20],FM Ch3 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[27:24],FM Ch4 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[39:36],FM Ch5 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[43:40],FM Ch6 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[47:44],FM Ch7 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[51:48],FM Ch8 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"P3O[55:52],FM Ch9 Volume,Default,Mute,25%,50%,75%,100%,125%,150%,200%,250%,300%,400%,500%,700%,1000%;",
	"-;",
	"T[0],Reset;",
	"R[0],Reset and close OSD;",
	"-;",
	// J1: bit 4=Fire(A), 5=Bomb(B), 6,7,8,9=unused, 10=Start, 11=Coin, 12=Pause.
	// Start 2P/Coin 2P rimossi: per giocare P2 con 1 pad usa OSD "Controls: Swap 1P/2P".
	"J1,Fire,Bomb,-,-,-,-,Start,Coin,Pause;",
	"jn,A,B,,,,,Start,R,L;",
	"V,v",`BUILD_DATE
};

wire forced_scandoubler;
wire  [1:0] buttons;
wire [127:0] status;
wire [10:0] ps2_key;
wire [15:0] joy0, joy1;
// ioctl_* come uscono da hps_io (RAW dalla MRA, encrypted dove serve)
wire        ioctl_download_raw;
wire [15:0] ioctl_index_raw;
wire        ioctl_wr_raw;
wire [26:0] ioctl_addr_raw;
wire [15:0] ioctl_dout_raw;
wire        ioctl_wait;

hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys(clk_sys),
	.HPS_BUS(HPS_BUS),
	.EXT_BUS(),
	.gamma_bus(gamma_bus),
	.forced_scandoubler(forced_scandoubler),
	.buttons(buttons),
	.status(status),
	.status_menumask(16'd0),
	.ps2_key(ps2_key),
	.joystick_0(joy0),
	.joystick_1(joy1),
	.ioctl_download(ioctl_download_raw),
	.ioctl_index(ioctl_index_raw),
	.ioctl_wr(ioctl_wr_raw),
	.ioctl_addr(ioctl_addr_raw),
	.ioctl_dout(ioctl_dout_raw),
	.ioctl_wait(ioctl_wait)
);

// === Savestate UI: trigger save/load da tasti (Alt+F1-F4 / F1-F4), gamepad, OSD ===
wire       ss_save, ss_load;
wire [1:0] ss_slot;
wire [15:0] joy_all = joy0 | joy1;
savestate_ui #(.INFO_TIMEOUT_BITS(25)) u_ss_ui (
	.clk         (clk_sys),
	.ps2_key     (ps2_key),
	.allow_ss    (1'b1),
	.joySS       (joy_all[13]),   // Select
	.joyRight    (joy_all[0]),
	.joyLeft     (joy_all[1]),
	.joyDown     (joy_all[2]),
	.joyUp       (joy_all[3]),
	.joyStart    (joy_all[12]),
	.joyRewind   (1'b0),
	.rewindEnable(1'b0),
	.status_slot (status[106:105]),
	.autoincslot (1'b0),
	.OSD_saveload(status[108:107]),  // R[107]=save, R[108]=restore
	.ss_save     (ss_save),
	.ss_load     (ss_load),
	.ss_info_req (),
	.ss_info     (),
	.statusUpdate(),
	.selected_slot(ss_slot)
);

// raiden_decrypt — applica MAME init_decryption() al volo durante download.
// Pass-through completo per ioctl_addr/wr/index/download; modifica solo ioctl_dout
// nei range crittati (Main section 2 + Sub completo).
wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [15:0] ioctl_dout;
raiden_decrypt u_decrypt (
	.ioctl_addr_in     (ioctl_addr_raw),
	.ioctl_dout_in     (ioctl_dout_raw),
	.ioctl_wr_in       (ioctl_wr_raw),
	.ioctl_index_in    (ioctl_index_raw),
	.ioctl_download_in (ioctl_download_raw),
	.ioctl_addr_out    (ioctl_addr),
	.ioctl_dout_out    (ioctl_dout),
	.ioctl_wr_out      (ioctl_wr),
	.ioctl_index_out   (ioctl_index),
	.ioctl_download_out(ioctl_download)
);

// --- Joystick to Raiden input mapping ---
// MAME P1_P2 port ($E0002): active low.
// Low byte P1 / high byte P2: bit0=U, bit1=D, bit2=L, bit3=R, bit4=Btn1, bit5=Btn2.
// MiSTer joy bits: joy[0]=R, joy[1]=L, joy[2]=D, joy[3]=U, joy[4]=A, joy[5]=B.
// Raiden P1_P2 layout MAME (raiden.cpp:589-605):
//   bit 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT, 4=BTN1(Fire), 5=BTN2(Bomb), 6=unused, 7=START1
//   bit 8-15 = P2 (stesso layout)
// MiSTer joy bits: 0=R, 1=L, 2=D, 3=U, 4=A=Fire, 5=B=Bomb, 10=Start
// Active LOW.
// Player 1P/2P (OSD O[30]): con 1 solo pad + "2P", il pad guida la nave P2.
wire        swap_pl = status[30];
wire [15:0] jp1 = swap_pl ? joy1 : joy0;
wire [15:0] jp2 = swap_pl ? joy0 : joy1;
wire [7:0] p1_input = {~jp1[10], 1'b1, ~jp1[5], ~jp1[4], ~jp1[0], ~jp1[1], ~jp1[2], ~jp1[3]};
wire [7:0] p2_input = {~jp2[10], 1'b1, ~jp2[5], ~jp2[4], ~jp2[0], ~jp2[1], ~jp2[2], ~jp2[3]};
wire [15:0] p1_p2_input = {p2_input, p1_input};

// Raiden NON ha system_input separato. P1_P2 inglobano già Start (bit 7=START1,
// bit 15=START2). Lo costruisco in p1_p2_input più sopra. Coin va via SEIBU_COIN_INPUTS
// (Z80 path → soundlatch). Service in coin_input bit (Z80 leggerà il segnale).
// system_input16 mantenuto per compat con la firma del compat wrapper.
wire [15:0] system_input16 = 16'hFFFF;

// Seibu coin input (ACTIVE_HIGH per SEIBU_COIN_INPUTS macro): bit0=COIN1, bit1=COIN2.
// Letto dal Z80 a 0x4013 → coin_r → soundlatch sub2main → main 68k legge 0xA0004.
wire [7:0] coin_input = {6'd0, jp2[11], jp1[11]};

// DIP switches — loaded from MRA via ioctl (index 254)
// Active-LOW: default "FF,FF" = all OFF = all 1s
reg [15:0] dip_sw = 16'hFFFF;
always @(posedge clk_sys)
	if (ioctl_wr && (ioctl_index == 16'd254) && !ioctl_addr[26:1])
		dip_sw <= ioctl_dout;

///////////////////////   CLOCKS   ///////////////////////////////

wire clk_sys;    // 80 MHz: monolitico
wire pll_locked;
pll pll
(
	.refclk(CLK_50M),
	.rst(0),
	.outclk_0(clk_sys),
	.locked(pll_locked)
);

// Game reset: include ioctl_download + reset_hold 22-bit (~43 ms @96 MHz).
// Pattern documentato: project_raiden_download_stretch_useful.md (commit 213d572).
// Senza hold il V30 esce da reset prima che SDRAM sia stabile → primo fetch a
// 0xFFFF0 ritorna garbage → schermo nero. Revert (8dd29a2) = regressione.
wire reset_cause = RESET | status[0] | buttons[1] | ~pll_locked | ioctl_download;
reg [21:0] reset_hold_cnt = 22'h3FFFFF;
always @(posedge clk_sys) begin
	if (reset_cause)                  reset_hold_cnt <= 22'h3FFFFF;
	else if (reset_hold_cnt != 22'd0) reset_hold_cnt <= reset_hold_cnt - 22'd1;
end
wire reset = (reset_hold_cnt != 22'd0);
`ifdef V30_SIM_PROBES
// spia sim: quale sorgente tiene il reset (RESET, OSD, ~pll_locked, download)
integer dbg_rst_c = 0;
always @(posedge clk_sys) begin
	dbg_rst_c <= dbg_rst_c + 1;
	if (dbg_rst_c % 500000 == 0)
		$display("[rst] clk%0d reset=%b RESET=%b osd=%b btn=%b pll=%b dl=%b",
		         dbg_rst_c, reset, RESET, status[0], buttons[1], pll_locked, ioctl_download);
end
`endif
// Bridge reset: ONLY pll_locked — bridge must run during download
// (revert dd86f8b: includere user reset causa mismatch sdram_ack/sdram_req,
// SDRAM controller non si resetta → bridge in reset vs SDRAM running → stuck)
wire bridge_reset = ~pll_locked;
// Video reset: ONLY pll_locked — CRT needs sync always
wire video_reset = ~pll_locked;

///////////////////////   SDRAM   ///////////////////////////////

// Genesis 4-port SDRAM controller (Sorgelig + donor bridge)
// Port 0: graphics ROM + download
// Port 1: main 68000 ROM
// Port 2: temporarily unused donor ROM path
// Port 3: audio/sample ROM path

wire [24:1] sd_addr0, sd_addr1, sd_addr2, sd_addr3;
wire [15:0] sd_din0, sd_din1, sd_din2, sd_din3;
wire        sd_wrl0, sd_wrh0, sd_wrl1, sd_wrh1, sd_wrl2, sd_wrh2, sd_wrl3, sd_wrh3;
wire        sd_req0, sd_req1, sd_req2, sd_req3;
wire        sd_ack0, sd_ack1, sd_ack2, sd_ack3;
wire [15:0] sd_dout0, sd_dout1, sd_dout2, sd_dout3;
wire        sdram_ready;

// OKI ADPCM ROM bridge ↔ jt6295 (via main_top)
wire [17:0] oki_rom_addr;
wire  [7:0] oki_rom_data;
wire        oki_rom_ok;

sdram sdram_ctrl
(
	.SDRAM_DQ(SDRAM_DQ),
	.SDRAM_A(SDRAM_A),
	.SDRAM_DQML(SDRAM_DQML),
	.SDRAM_DQMH(SDRAM_DQMH),
	.SDRAM_BA(SDRAM_BA),
	.SDRAM_nCS(SDRAM_nCS),
	.SDRAM_nWE(SDRAM_nWE),
	.SDRAM_nRAS(SDRAM_nRAS),
	.SDRAM_nCAS(SDRAM_nCAS),
	.SDRAM_CLK(SDRAM_CLK),
	.SDRAM_CKE(SDRAM_CKE),

	.init(~pll_locked),
	.clk(clk_sys),
	// CPU-first FISSO (mode 2), sicuro per costruzione: 2 V30 = max 1 txn/32clk
	// l'uno (ciclo bus 4T@10MHz) = max 50% slot SDRAM anche a miss 100%; il video
	// worst-case usa ~25-31% della scanline -> mai affamato (margine ~2x). Con
	// le CPU 0-wait anche sui miss spariscono i rallentamenti (assenti su PCB)
	// -> niente sfasamenti main<->sub -> niente leak sprite. (status[35:34] era
	// senza voce OSD = morto.)
	.prio_mode(2'd2),
	.ready(sdram_ready),

	.addr0(sd_addr0), .wrl0(sd_wrl0), .wrh0(sd_wrh0),
	.din0(sd_din0), .dout0(sd_dout0), .req0(sd_req0), .ack0(sd_ack0),

	.addr1(sd_addr1), .wrl1(sd_wrl1), .wrh1(sd_wrh1),
	.din1(sd_din1), .dout1(sd_dout1), .req1(sd_req1), .ack1(sd_ack1),

	.addr2(sd_addr2), .wrl2(sd_wrl2), .wrh2(sd_wrh2),
	.din2(sd_din2), .dout2(sd_dout2), .req2(sd_req2), .ack2(sd_ack2),

	.addr3(sd_addr3), .wrl3(sd_wrl3), .wrh3(sd_wrh3),
	.din3(sd_din3), .dout3(sd_dout3), .req3(sd_req3), .ack3(sd_ack3)
);

///////////////////////   BRIDGE   ///////////////////////////////

// Bridge between game logic (level protocol) and Genesis SDRAM (toggle protocol)
wire [23:0] game_tile_addr, game_main_addr, game_sub_addr;
wire        game_tile_req, game_main_req, game_sub_req;
wire  [2:0] game_tile_kind;     // 0=BG, 1=MG, 2=FG, 3=SPR, 4=TXT
wire [31:0] game_tile_data;
wire        game_tile_valid;
wire [15:0] game_main_data, game_sub_data;
// Audio Z80 ROM removed from SDRAM — will use BRAM when audio implemented
wire        game_main_ready, game_sub_ready;

// (Storico: il vecchio bypass "pattern WonderSwan" era di un'era pre-latch-FSM;
// oggi il ricevitore Main latcha il ready-pulse come il Sub -> cache ok.)
// MAIN ROM via rom_cache (come il Sub): senza cache il Main fetcha DIRETTO da
// SDRAM in arbitraggio contro grafica+OKI+miss Sub -> nelle scene pesanti viene
// AFFAMATO -> rallentamenti (che MAME non ha) -> sfasamento timing main<->sub
// -> leak slot sprite (i bug avvengono DURANTE i rallentamenti, verificato HW).
// Il ricevitore Main ha lo stesso latch-FSM del Sub (main_top:128-152 =
// sub_top:112-134): protocollo ready-pulse gia' provato con la cache su HW.
wire [23:0] bridge_main_addr;
wire        bridge_main_req;
wire [15:0] bridge_main_data;
wire        bridge_main_ready;
rom_cache #(.CACHE_BITS(13)) u_main_cache (
	.clk(clk_sys), .reset(bridge_reset),
	.cpu_addr(game_main_addr), .cpu_req(game_main_req),
	.cpu_data(game_main_data), .cpu_ready(game_main_ready),
	.sdram_addr(bridge_main_addr), .sdram_req(bridge_main_req),
	.sdram_data(bridge_main_data), .sdram_ready(bridge_main_ready)
);

// SUB ROM in SDRAM (porta 2), non piu' in BRAM. Il download scrive gia' tutto
// in SDRAM (Sub @ word offset SUB_BASE=0x030000); il Sub V30 legge via bridge
// come il Main (READY/Tw per la latenza SDRAM). Rimosso il duplicato BRAM
// (256KB = ~205 M10K liberati).
wire [15:0] bridge_sub_data;
wire        bridge_sub_ready;
wire [23:0] sub_cache_sdram_addr;
wire        sub_cache_sdram_req;

// rom_cache: la maggior parte dei fetch del Sub in 1 ciclo (niente SDRAM) ->
// il Sub non ruba banda alla grafica -> niente nero da contesa.
// CACHE_BITS 13 = 16KB (8192x16): miss rarissimi -> timing Sub quasi-BRAM.
// Con 9 (1KB) il thrash nelle scene pesanti rallentava/jitterava il Sub ->
// race mailbox main<->sub -> leak slot sprite (detriti fissi a schermo, stage 3).
rom_cache #(.CACHE_BITS(13)) u_sub_cache (
	.clk(clk_sys), .reset(bridge_reset),
	.cpu_addr(game_sub_addr), .cpu_req(game_sub_req),
	.cpu_data(game_sub_data), .cpu_ready(game_sub_ready),
	.sdram_addr(sub_cache_sdram_addr), .sdram_req(sub_cache_sdram_req),
	.sdram_data(bridge_sub_data), .sdram_ready(bridge_sub_ready)
);

sdram_bridge bridge
(
	.clk(clk_sys),
	.reset(bridge_reset),
	.sdram_ready(sdram_ready),

	// HPS download
	.ioctl_download(ioctl_download),
	.ioctl_wr(ioctl_wr),
	.ioctl_addr(ioctl_addr),
	.ioctl_dout(ioctl_dout),
	.ioctl_index(ioctl_index),
	.ioctl_wait(ioctl_wait),

	// Game: Tile ROM (32-bit)
	.tile_byte_addr(game_tile_addr),
	.tile_req(game_tile_req),
	.gfx_kind(game_tile_kind),
	.tile_data(game_tile_data),
	.tile_valid(game_tile_valid),

	// Game: Main CPU ROM (16-bit)
	.main_byte_addr(bridge_main_addr),
	.main_req(bridge_main_req),
	.main_data(bridge_main_data),
	.main_ready(bridge_main_ready),

	// Sub V30 ROM in SDRAM (porta 2) via rom_cache.
	.sub_byte_addr(sub_cache_sdram_addr),
	.sub_req(sub_cache_sdram_req),
	.sub_data(bridge_sub_data),
	.sub_ready(bridge_sub_ready),

	// OKI ADPCM ROM (port 3)
	.oki_byte_addr(oki_rom_addr),
	.oki_data(oki_rom_data),
	.oki_ok(oki_rom_ok),

	// SDRAM ports
	.sdram_addr0(sd_addr0), .sdram_din0(sd_din0),
	.sdram_wrl0(sd_wrl0), .sdram_wrh0(sd_wrh0),
	.sdram_req0(sd_req0), .sdram_ack0(sd_ack0), .sdram_dout0(sd_dout0),

	.sdram_addr1(sd_addr1), .sdram_din1(sd_din1),
	.sdram_wrl1(sd_wrl1), .sdram_wrh1(sd_wrh1),
	.sdram_req1(sd_req1), .sdram_ack1(sd_ack1), .sdram_dout1(sd_dout1),

	.sdram_addr2(sd_addr2), .sdram_din2(sd_din2),
	.sdram_wrl2(sd_wrl2), .sdram_wrh2(sd_wrh2),
	.sdram_req2(sd_req2), .sdram_ack2(sd_ack2), .sdram_dout2(sd_dout2),

	.sdram_addr3(sd_addr3), .sdram_din3(sd_din3),
	.sdram_wrl3(sd_wrl3), .sdram_wrh3(sd_wrh3),
	.sdram_req3(sd_req3), .sdram_ack3(sd_ack3), .sdram_dout3(sd_dout3)
);

///////////////////////   GAME   ///////////////////////////////

// render_x/y prodotti da Raiden_video_timing più sotto
wire [9:0]  render_x;
wire [8:0]  render_y;

// Palette read-side: indirizzo deciso dal pixel pipeline (priorità tra layer)
reg  [10:0] pal_b_addr;
wire [7:0]  pal_b_r, pal_b_g, pal_b_b;

// Text VRAM read wires
wire [10:0] text_vram_addr;
wire [15:0] text_vram_data;

// ── Scroll/ctrl esposti dal Main top (scroll_ram $0F000 + ctrl_w $0E006) ──
// Layout legacy ctrl_l0: [0]=BG, [1]=MG, [2]=FG, [3]=Text, [4]=Spr, [5]=flip
wire [511:0] scroll_words_flat;
// MAME raiden.cpp:391-401 — scroll_ram è u16 array, indici 0xNN sono WORD index
// (non byte address). Ogni m_scroll_ram[N] = u16 word, di cui CPU usa solo low byte.
// Formula (raiden.cpp:398): ((b_hi & 0xF0) << 4) | ((b_lo & 0x7F) << 1) | ((b_lo & 0x80) >> 7)
//   scrollregs[0] = BG_X = scroll_ram[0x09].lo<<4 | scroll_ram[0x0A].lo shifted
//   scrollregs[1] = BG_Y = scroll_ram[0x01].lo<<4 | scroll_ram[0x02].lo shifted
//   scrollregs[2] = FG_X = scroll_ram[0x19].lo<<4 | scroll_ram[0x1A].lo shifted
//   scrollregs[3] = FG_Y = scroll_ram[0x11].lo<<4 | scroll_ram[0x12].lo shifted
// scroll_words_flat[gi*16 +: 16] = scroll_ram[gi]  →  low byte = [gi*16 +: 8]
// (BUG corretto v81: prima usavamo byte indices, MAME usa word indices.)
wire [7:0] sr_b09 = scroll_words_flat[ 9*16+0 +: 8];   // scroll_ram[0x09] low
wire [7:0] sr_b0A = scroll_words_flat[10*16+0 +: 8];   // scroll_ram[0x0A] low
wire [7:0] sr_b01 = scroll_words_flat[ 1*16+0 +: 8];   // scroll_ram[0x01] low
wire [7:0] sr_b02 = scroll_words_flat[ 2*16+0 +: 8];   // scroll_ram[0x02] low
wire [7:0] sr_b19 = scroll_words_flat[25*16+0 +: 8];   // scroll_ram[0x19] low
wire [7:0] sr_b1A = scroll_words_flat[26*16+0 +: 8];   // scroll_ram[0x1A] low
wire [7:0] sr_b11 = scroll_words_flat[17*16+0 +: 8];   // scroll_ram[0x11] low
wire [7:0] sr_b12 = scroll_words_flat[18*16+0 +: 8];   // scroll_ram[0x12] low
wire [15:0] map_xscroll_l0 = {4'd0, sr_b09[7:4], sr_b0A[6:0], sr_b0A[7]};  // BG X
wire [15:0] map_yscroll_l0 = {4'd0, sr_b01[7:4], sr_b02[6:0], sr_b02[7]};  // BG Y
wire [15:0] map_xscroll_l1 = {4'd0, sr_b19[7:4], sr_b1A[6:0], sr_b1A[7]};  // FG X
wire [15:0] map_yscroll_l1 = {4'd0, sr_b11[7:4], sr_b12[6:0], sr_b12[7]};  // FG Y
wire        ctrl_bg_en, ctrl_fg_en, ctrl_tx_en, ctrl_sp_en, ctrl_flipscreen;
wire [15:0] map_ctrl_l0 = {9'd0, 1'b0, ctrl_flipscreen,
                            ctrl_sp_en, ctrl_tx_en, ctrl_fg_en, 1'b0, ctrl_bg_en};

// ── Shared RAM Main ↔ Sub: 2 BRAM speculari (no race, no stall) ──
wire [11:1] main_shared_addr;
wire        main_shared_cs;
wire  [1:0] main_shared_we;
wire [15:0] main_shared_wdata;
wire [15:0] main_shared_rdata;
wire [11:1] sub_shared_addr;
wire        sub_shared_cs;
wire  [1:0] sub_shared_we;
wire [15:0] sub_shared_wdata;
wire [15:0] sub_shared_rdata;

raiden_shared_ram #(.SS_IDX(3)) u_shared (
	.clk        (clk_sys),
	.reset      (reset),
	.main_addr  (main_shared_addr),
	.main_cs    (main_shared_cs),
	.main_din   (main_shared_wdata),
	.main_dout  (main_shared_rdata),
	.main_we    (main_shared_we),
	.sub_addr   (sub_shared_addr),
	.sub_cs     (sub_shared_cs),
	.sub_din    (sub_shared_wdata),
	.sub_dout   (sub_shared_rdata),
	.sub_we     (sub_shared_we),
	.ss_shared  (ssb[3])
);


// ── Sound comm bus: pilotato da Z80 audio reale (Raiden_audio_z80) ────────
wire        snd_cs;
wire  [3:1] snd_addr;
wire        snd_wr;
wire        snd_rd;
wire [15:0] snd_wdata;
wire [15:0] snd_rdata;

// ── Palette → RGB888 ── modalità selezionabile da OSD P1O[91:89]
//   000 xBGR       : R=[ 3:0]  G=[ 7:4]  B=[11:8]   (MAME palette_device::xBGR_444)
//   001 xRGB       : R=[11:8]  G=[ 7:4]  B=[ 3:0]   (R↔B swap)
//   010 xBRG       : R=[ 3:0]  G=[11:8]  B=[ 7:4]   (rotazione 1)
//   011 xGBR       : R=[ 7:4]  G=[ 3:0]  B=[11:8]   (rotazione 2)
//   100 xRBG       : R=[11:8]  G=[ 3:0]  B=[ 7:4]   (rotazione 3)
//   101 xGRB       : R=[ 7:4]  G=[11:8]  B=[ 3:0]   (rotazione 4)
//   110 nibblesHi  : usa nibble alti [15:12]/[11:8]/[ 7:4]  (palette in word HI)
//   111 nibblesLo  : usa nibble bassi [11:8]/[ 7:4]/[ 3:0] in ordine inverso (R↔B nibble alto)
wire [15:0] pal_word_raw;
wire [2:0]  pal_fmt = 3'b000;  // xBGR (MAME palette_device::xBGR_444)
reg  [3:0]  pal_r4, pal_g4, pal_b4;
always @(*) begin
	case (pal_fmt)
		3'b000: begin pal_r4 = pal_word_raw[ 3:0];  pal_g4 = pal_word_raw[ 7:4];  pal_b4 = pal_word_raw[11:8];  end
		3'b001: begin pal_r4 = pal_word_raw[11:8];  pal_g4 = pal_word_raw[ 7:4];  pal_b4 = pal_word_raw[ 3:0];  end
		3'b010: begin pal_r4 = pal_word_raw[ 3:0];  pal_g4 = pal_word_raw[11:8];  pal_b4 = pal_word_raw[ 7:4];  end
		3'b011: begin pal_r4 = pal_word_raw[ 7:4];  pal_g4 = pal_word_raw[ 3:0];  pal_b4 = pal_word_raw[11:8];  end
		3'b100: begin pal_r4 = pal_word_raw[11:8];  pal_g4 = pal_word_raw[ 3:0];  pal_b4 = pal_word_raw[ 7:4];  end
		3'b101: begin pal_r4 = pal_word_raw[ 7:4];  pal_g4 = pal_word_raw[11:8];  pal_b4 = pal_word_raw[ 3:0];  end
		3'b110: begin pal_r4 = pal_word_raw[ 7:4];  pal_g4 = pal_word_raw[11:8];  pal_b4 = pal_word_raw[15:12]; end
		3'b111: begin pal_r4 = pal_word_raw[15:12]; pal_g4 = pal_word_raw[11:8];  pal_b4 = pal_word_raw[ 7:4];  end
	endcase
end
assign pal_b_r = {pal_r4, pal_r4};
assign pal_b_g = {pal_g4, pal_g4};
assign pal_b_b = {pal_b4, pal_b4};


// ── Audio Seibu reale: Z80 + YM3812 (jtopl2) + OKI6295 (jt6295) ─────────────
// MRA layout audiocpu: 0x0A0000-0x0AFFFF (64KB raw byte-pack)
// OKI ROM: SDRAM @ OKI_BASE (oki_rom_addr/data/ok via SDRAM bridge)
// Coin button (joy[11]) → Z80 0x4013 → sub2main → main 0xA0004 → coin_credit
Raiden_audio_z80 #(.SS_IDX_ZRAM(10), .SS_IDX_Z80(12), .SS_IDX_YMSH(13), .SS_IDX_GLUE(14)) u_audio (
	.clk           (clk_sys),
	.reset         (reset),
	.pause         (paused_safe),
	.clk_sel       (2'd0),         // legacy, ignored
	.fm_vol_sel    (status[87:84]),
	.oki_vol_sel   (status[91:88]),
	.oki_ch_vol_sel0 (status[70:67]),
	.oki_ch_vol_sel1 (status[74:71]),
	.oki_ch_vol_sel2 (status[78:75]),
	.oki_ch_vol_sel3 (status[82:79]),
	.fm_ch_vol_sel0 (status[11:8]),
	.fm_ch_vol_sel1 (status[15:12]),
	.fm_ch_vol_sel2 (status[23:20]),
	.fm_ch_vol_sel3 (status[27:24]),
	.fm_ch_vol_sel4 (status[39:36]),
	.fm_ch_vol_sel5 (status[43:40]),
	.fm_ch_vol_sel6 (status[47:44]),
	.fm_ch_vol_sel7 (status[51:48]),
	.fm_ch_vol_sel8 (status[55:52]),
	.ioctl_download(ioctl_download),
	.ioctl_wr      (ioctl_wr),
	.ioctl_addr    (ioctl_addr),
	.ioctl_dout    (ioctl_dout),
	.snd_cs        (snd_cs),
	.snd_addr      (snd_addr),
	.snd_wr        (snd_wr),
	.snd_rd        (snd_rd),
	.snd_wdata     (snd_wdata),
	.snd_rdata     (snd_rdata),
	.snd_nmi_n     (1'b1),
	.snd_reset_in  (1'b0),
	.coin_input    (coin_input),
	.oki_rom_addr  (oki_rom_addr),
	.oki_rom_data  (oki_rom_data),
	.oki_rom_ok    (oki_rom_ok),
	.audio_l       (game_audio_l),
	.audio_r       (game_audio_r),
	.ss_zram       (ssb[10]),
	.ss_z80        (ssb[12]),
	.ss_ymsh       (ssb[13]),
	.ss_glue       (ssb[14]),
	.z80_ss_ready  (z80_ss_ready)
);

// ─── FILTRO DI USCITA (Arcade LPF 6 kHz 2nd order) ───────────────────────
// L'uscita del MiSTer e' piatta; il PCB reale ha uno stadio analogico che
// taglia gli alti. La curva scelta confrontando con le registrazioni dalla
// scheda e' "Arcade LPF 6khz 2nd.txt" dei filtri di sistema, qui CUCITA
// nell'RTL cosi' ogni utente ce l'ha di serie senza file esterni.
// Voce OSD "Audio Filter" On/Off: status[83]=0 -> On (default).
//
// Coefficienti presi dal file, invariati:
//   Sampling Frequency 7056000 ; Base gain 0.00003952949005309181
//   X0=2  X1=1  X2=0 ; Y0=-1.99244411238133389830 ; Y1=0.99247255086338648233
// (il file non ha Y2 -> 0)
//
// Il `ce` e' generato con LO STESSO accumulatore del framework
// (sys/audio_out.v: cnt += flt_rate*2, confronto con CLK_RATE), quindi la
// frequenza media e' identica e la risposta e' la stessa, non un'approssimazione.
localparam [31:0] FLT_RATE_HZ = 32'd7056000;   // Sampling Frequency del file
localparam [31:0] CLK_RATE_HZ = 32'd80000000;  // clk_sys

reg flt_ce;
always @(posedge clk_sys) begin
	reg [31:0] flt_cnt = 0;
	flt_ce  = 0;
	flt_cnt = flt_cnt + {FLT_RATE_HZ[30:0], 1'b0};
	if (flt_cnt >= CLK_RATE_HZ) begin
		flt_cnt = flt_cnt - CLK_RATE_HZ;
		flt_ce  = 1;
	end
end

// sample_ce = frequenza di uscita del filtro (48 kHz), stesso metodo
localparam [31:0] SND_RATE_HZ = 32'd48000;
reg snd_ce;
always @(posedge clk_sys) begin
	reg [31:0] snd_cnt = 0;
	snd_ce  = 0;
	snd_cnt = snd_cnt + SND_RATE_HZ;
	if (snd_cnt >= CLK_RATE_HZ) begin
		snd_cnt = snd_cnt - CLK_RATE_HZ;
		snd_ce  = 1;
	end
end

wire [15:0] flt_audio_l, flt_audio_r;
IIR_filter #(
	.use_params(1),
	.stereo    (1),
	.coeff_x   (0.00003952949005309181),
	.coeff_x0  (2),
	.coeff_x1  (1),
	.coeff_x2  (0),
	.coeff_y0  (-1.99244411238133389830),
	.coeff_y1  (0.99247255086338648233),
	.coeff_y2  (0)
) u_audio_lpf (
	.clk      (clk_sys),
	.reset    (reset),
	.ce       (flt_ce),
	.sample_ce(snd_ce),
	.cx (40'd0), .cx0(8'd0), .cx1(8'd0), .cx2(8'd0),
	.cy0(24'd0), .cy1(24'd0), .cy2(24'd0),
	.input_l  (game_audio_l),
	.input_r  (game_audio_r),
	.output_l (flt_audio_l),
	.output_r (flt_audio_r)
);

wire audio_filter_off = status[83];
assign AUDIO_L = audio_filter_off ? game_audio_l : flt_audio_l;
assign AUDIO_R = audio_filter_off ? game_audio_r : flt_audio_r;

// ── Isolatore OSD → CPU (2-FF sync, attributi preserve). ──
// Aggiungere bit OSD altrove NON destabilizza più le CPU.
wire        pause_iso;
wire  [2:0] main_clk_sel_iso, sub_clk_sel_iso;
raiden_osd_iso u_osd_iso (
	.clk              (clk_sys),
	.pause_in         (paused_safe),
	.main_clk_sel_in  (3'd0),   // CPU FISSA 10 MHz — scollegata dall'OSD (no overclock)
	.sub_clk_sel_in   (3'd0),   // CPU FISSA 10 MHz — scollegata dall'OSD (no overclock)
	.pause_out        (pause_iso),
	.main_clk_sel_out (main_clk_sel_iso),
	.sub_clk_sel_out  (sub_clk_sel_iso)
);

// ── Savestate: park delle CPU a confine d'istruzione ──────────────────────
// Bug savestate: la cattura è frame-aligned (vblank), non allineata al confine
// istruzione del V30. Se una CPU è a metà istruzione al save, reg_ip punta a
// metà istruzione e la FSM microcode (non salvata) è persa → al restore la CPU
// misdecoda → freeze (intermittente). Fix: quando arriva la pausa, la CPU
// continua a girare finché non raggiunge CPUSTAGE_IDLE (cpu_idle=1), POI si
// congela (park). La cattura SS parte solo quando ENTRAMBE sono a confine.
wire main_cpu_idle, sub_cpu_idle;
wire z80_ss_ready;
wire main_cpu_pause = pause_iso & main_cpu_idle;   // gira finché non è idle, poi park
wire sub_cpu_pause  = pause_iso & sub_cpu_idle;
wire cpus_ss_ready  = main_cpu_idle & sub_cpu_idle & z80_ss_ready; // TUTTE le CPU a confine = cattura sicura

// ── Main V30 (raiden_state::main_map) ──
Raiden_main_top #(.SS_IDX_SPR(7), .SS_IDX_CPU(8)) u_main (
	.clk              (clk_sys),
	.reset            (reset),
	.pause            (main_cpu_pause),
	.cpu_idle         (main_cpu_idle),
	.clk_sel          (main_clk_sel_iso),      // = 0 → 10 MHz fisso (no overclock OSD)
	.p1_input         (p1_input),
	.p2_input         (p2_input),
	.dsw_input        (dip_sw),
	.main_rom_rdata   (game_main_data),
	.main_rom_ready   (game_main_ready),
	.main_rom_addr    (game_main_addr),
	.main_rom_req     (game_main_req),
	.vblank_in        (VBlank),
	.ioctl_download   (ioctl_download),
	.ctrl_bg_en       (ctrl_bg_en),
	.ctrl_fg_en       (ctrl_fg_en),
	.ctrl_tx_en       (ctrl_tx_en),
	.ctrl_sp_en       (ctrl_sp_en),
	.ctrl_flipscreen  (ctrl_flipscreen),
	.scroll_words_flat(scroll_words_flat),
	.snd_cs           (snd_cs),
	.snd_addr         (snd_addr),
	.snd_wr           (snd_wr),
	.snd_rd           (snd_rd),
	.snd_wdata        (snd_wdata),
	.snd_rdata        (snd_rdata),
	.text_vram_addr   (text_vram_addr),
	.text_vram_data   (text_vram_data),
	.spr_vram_addr    (spr_vram_addr),
	.spr_vram_data    (spr_vram_data),
	.main_shared_addr (main_shared_addr),
	.main_shared_cs   (main_shared_cs),
	.main_shared_we   (main_shared_we),
	.main_shared_wdata(main_shared_wdata),
	.main_shared_rdata(main_shared_rdata),
	.dbg_irq_pending  (main_irq_pending_probe),
	.ss_workram       (ssb[0]),
	.ss_txt           (ssb[1]),
	.ss_scroll        (ssb[2]),
	.ss_spr           (ssb[7]),
	.ss_cpu           (ssb[8]),
	.ss_cpu_reload    (ss_cpu_reload)
);

// ── Sub V30 (raiden_state::sub_map) ──
Raiden_sub_top #(.SS_IDX_BG(4), .SS_IDX_FG(5), .SS_IDX_PAL(6), .SS_IDX_CPU(9), .SS_IDX_SUBRAM(11)) u_sub (
	.clk              (clk_sys),
	.reset            (reset),
	.pause            (sub_cpu_pause),
	.cpu_idle         (sub_cpu_idle),
	.clk_sel          (sub_clk_sel_iso),       // = 0 → 10 MHz fisso (no overclock OSD)
	.sub_rom_rdata    (game_sub_data),
	.sub_rom_ready    (game_sub_ready),
	.sub_rom_addr     (game_sub_addr),
	.sub_rom_req      (game_sub_req),
	.vblank_in        (VBlank),
	.sub_shared_addr  (sub_shared_addr),
	.sub_shared_cs    (sub_shared_cs),
	.sub_shared_we    (sub_shared_we),
	.sub_shared_wdata (sub_shared_wdata),
	.sub_shared_rdata (sub_shared_rdata),
	.bg_vram_addr     (bg_vram_addr),
	.bg_vram_data     (bg_vram_data),
	.fg_vram_addr     (fg_vram_addr),
	.fg_vram_data     (fg_vram_data),
	.pal_vram_addr    (pal_b_addr),
	.pal_vram_data    (pal_word_raw),
	.dbg_irq_pending  (sub_irq_pending_probe),
	// Palette overlay tap
	.dbg_cpu_addr     (sub_dbg_cpu_addr),
	.dbg_cpu_dout     (sub_dbg_cpu_dout),
	.dbg_cpu_be       (sub_dbg_cpu_be),
	.dbg_cpu_wr       (sub_dbg_cpu_wr),
	.dbg_palette_memrq(sub_dbg_palette_memrq),
	.ss_bg            (ssb[4]),
	.ss_fg            (ssb[5]),
	.ss_pal           (ssb[6]),
	.ss_cpu           (ssb[9]),
	.ss_cpu_reload    (ss_cpu_reload),
	.ss_subram        (ssb[11])
);

// Wires per palette overlay tap
wire [19:0] sub_dbg_cpu_addr;
wire [15:0] sub_dbg_cpu_dout;
wire  [1:0] sub_dbg_cpu_be;
wire        sub_dbg_cpu_wr;
wire        sub_dbg_palette_memrq;

///////////////////////   VIDEO   ///////////////////////////////

// Raiden timing single-screen 320x224 @ 59.4 Hz (original) / 60.1 Hz.
// Pixel clock 6 MHz = clk_sys/16. HTotal=384, VTotal=263 (59.4) o 260 (60Hz).
wire ce_pix;
wire HBlank, VBlank, HSync, VSync, video_de;
wire [9:0] timing_hpos;
wire [9:0] timing_vpos;

Raiden_video_timing u_video_timing (
	.clk        (clk_sys),
	.reset      (video_reset),
	.mode_60hz  (status[19]),
	.ce_pix     (ce_pix),
	.hpos       (timing_hpos),
	.vpos       (timing_vpos),
	.active_x   (render_x),
	.active_y   (render_y),
	.hblank     (HBlank),
	.vblank     (VBlank),
	.hsync      (HSync),
	.vsync      (VSync),
	.de         (video_de)
);

// ── Flip screen (CRTC reg 0x1A bit 0) ───────────────────────────────────────
// MAME: BIT(reg_1a, 0) → flip_screen. Arcade reale = monitor CRT capovolto;
// game scrive in VRAM convinto che lo schermo sia ruotato 180° → noi vediamo
// flippato finchè non invertiamo.
//
// Strategia:
//  - X flip: il read-side dei line_buffer (dentro tile_layer/text_renderer)
//    legge linebuf[hpos]. Sostituisco hpos con (319-hpos) → mostra mirror H.
//  - Y flip: la prefetch durante display vpos=N riempie il buffer per il
//    display vpos=N+1. In flip ON serve che mostri riga ROM (V_VISIBLE-1-(N+1))
//    = (V_VISIBLE-2-N). Sostituisco vpos del prefetch con (V_VISIBLE-2-vpos)
//    quando flip on, così target_y = V_VISIBLE-2-vpos + 1 = V_VISIBLE-1-vpos.
wire        flip_screen = map_ctrl_l0[5];
// Coordinate LOGICHE display (0..319, 0..223), shiftate dal timing CRTC raw.
// Il timing CRTC ora è SYNC→BP→VISIBLE→FP, quindi VISIBLE inizia a hpos=48,
// vpos=30. I renderer devono vedere coordinate "del gioco" 0..319/0..223.
localparam [9:0] H_VIS_START_TOP = 10'd48;   // = H_SYNC + H_BP (256 wide @clk_sys=80MHz)
localparam [8:0] V_VIS_START_TOP = 9'd30;    // = V_SYNC + V_BP
wire [9:0]  hpos_logic = timing_hpos - H_VIS_START_TOP;
wire [8:0]  vpos_logic = timing_vpos[8:0] - V_VIS_START_TOP;
// Tile_layer prefetch usa target_y = vpos+1 (la riga ROMda mostrare al display
// vpos=N+1). Quando flip ON serve target_y = V_VISIBLE-1-(N+1) = 222-N.
wire [8:0]  vpos_for_pf   = flip_screen ? (9'd222 - vpos_logic) : vpos_logic;
// (text ora flippa internamente su eff_y, non usa piu' vpos_for_text)
// Read path (X): tile_layer linebuf[hpos]. Per text è eff_x = hpos+scroll.
// Flip DIP service specchia le 256 colonne visibili (0..255): 255-hpos.
// Era 319 (off-by-64, residuo vecchio H_TOTAL) → BG/FG flippati shiftati di
// 64px. MAME raiden.cpp: tilemap set_flip_all su 256 colonne = 255-x.
wire [9:0]  hpos_for_read = flip_screen ? (10'd255 - hpos_logic) : hpos_logic;

// ── Text layer renderer (8x8, 4bpp, 64x32 grid) ─────────────────────────────
// Char ROM caricata via ioctl da MRA: txtiles SDRAM region 0x040000..0x05FFFF
// (TXT_BASE byte = 0x040000*2 = 0x080000 byte? NO: TXT_BASE è word address.
//  ioctl_addr è BYTE address. Quindi ioctl carica byte da 0x080000..0x0BFFFF
//  per 128KB di txtiles in word-base 0x040000).
wire        text_opaque;
wire [10:0] text_pen;

// Raiden text region MRA byte = 0x0B0000-0x0BFFFF (64KB, interleave 9+10 word).
// (Layout MRA v2: bg/fg/spr ora raw byte-pack, text shift -64KB rispetto a v1.)
// Filtro range e converto a offset relativo (17 bit, byte index).
wire        text_rom_dl_wr =
	ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
	(ioctl_addr >= 27'h0B0000) && (ioctl_addr < 27'h0C0000);
wire [16:0] text_rom_dl_offset = {1'b0, ioctl_addr[15:0]};
// Raiden Text scroll fissi: 128 X (MAME), 16 Y (centratura HW reale)
Raiden_text_renderer u_text (
	.clk          (clk_sys),
	.reset        (reset),                 // include ioctl_download (fix garbage primo boot, vedi GundamSD c7ace01/1bdcd6c)
	.ce_pix       (ce_pix),
	.decode_mode  (7'b0010000),  // HW verified 2026-05-19: Default + NIB
	.hpos         (hpos_logic),           // NORMALE: flip fatto dentro su eff_x
	.vpos         (vpos_logic),           // NORMALE: flip fatto dentro su eff_y
	.flip_screen  (flip_screen),
	.de           (video_de),
	.layer_en     (map_ctrl_l0[3]),       // bit3 = Text enable
	.scroll_x     (16'd0),
	.scroll_y     (16'd16),
	.xoff         (osd_txt_xoff),
	.yoff         (osd_txt_yoff),
	.vram_addr    (text_vram_addr),
	.vram_data    (text_vram_data),
	.rom_dl_wr    (text_rom_dl_wr),
	.rom_dl_addr  (text_rom_dl_offset),
	.rom_dl_data  (ioctl_dout),
	.opaque       (text_opaque),
	.pen_index    (text_pen)
);

// ── BG/FG layer renderer (16x16, 4bpp, 32x32) — fetch SDRAM via arbiter ──
// Raiden plain: NO MG layer (MAME raiden.cpp:493-495 sub_map ha solo bgram+fgram+palette).
wire        bg_opaque, fg_opaque;
wire [10:0] bg_pen, fg_pen;
wire [10:0] bg_vram_addr, fg_vram_addr;
wire [15:0] bg_vram_data, fg_vram_data;

// new_line pulse: hpos passa da H_TOTAL-1 a 0
reg [9:0] hpos_prev;
always @(posedge clk_sys) if (ce_pix) hpos_prev <= timing_hpos;
wire layer_new_line = ce_pix && (timing_hpos == 10'd0) && (hpos_prev != 10'd0);

// Scroll (Raiden scroll_ram[0..3]: BG x/y, FG x/y). Display 256x224 (BloodBros pattern).
wire [15:0] bg_scroll_x = map_xscroll_l0;
wire [15:0] bg_scroll_y = map_yscroll_l0;
wire [15:0] fg_scroll_x = map_xscroll_l1;
wire [15:0] fg_scroll_y = map_yscroll_l1;

// Arbiter wires (BG + FG)
wire        arb_bg_req,  arb_fg_req;
wire [23:0] arb_bg_addr, arb_fg_addr;
wire [31:0] arb_bg_data, arb_fg_data;
wire        arb_bg_valid, arb_fg_valid;

// Pipeline reg per decode_mode (taglia critical path status[]→tile_layer/sprite).
// HARDCODED HW-verified 2026-05-19 (mantenuti come reg per non cambiare il fanin
// dei moduli renderer; il sintetizzatore costantifica e via).
//   BGFG = {nibble=1, decode=BS=0001}    = 5'b10001
//   SPR  = {nibble=1, decode=BRSIBL=1110}= 5'b11110
reg [4:0] bgfg_decode_mode_r;
reg [4:0] spr_decode_mode_r;
always @(posedge clk_sys) begin
	bgfg_decode_mode_r <= 5'b10001;
	spr_decode_mode_r  <= 5'b11110;
end

// MAME gfx_raiden palette bases: bgtiles=0, fgtiles=256(0x100), sprites=512(0x200), text=768(0x300)
Raiden_tile_layer #(
	.COLOR_BASE   (11'h000),  // BG palette base 0 (raiden.cpp:698 GFXDECODE bgtiles 0,16)
	.HAS_TRANSP   (0),
	.HAS_GFX_BANK (0),
	.TILE_KIND    (3'd0)
) u_bg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.decode_mode(bgfg_decode_mode_r),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[0]),
	.new_line(layer_new_line),
	.scroll_x(bg_scroll_x), .scroll_y(bg_scroll_y),
	.xoff(osd_bg_xoff), .yoff(osd_bg_yoff),
	.gfx_bank(16'd0),
	.vram_addr(bg_vram_addr), .vram_data(bg_vram_data),
	.rom_req(arb_bg_req), .rom_addr(arb_bg_addr),
	.rom_data(arb_bg_data), .rom_valid(arb_bg_valid),
	.opaque(bg_opaque), .pen_index(bg_pen)
);

Raiden_tile_layer #(
	.COLOR_BASE   (11'h100),  // FG palette base 256 (raiden.cpp:699 GFXDECODE fgtiles 256,16)
	.HAS_TRANSP   (1),
	.HAS_GFX_BANK (0),
	.TILE_KIND    (3'd2)
) u_fg (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.decode_mode(bgfg_decode_mode_r),
	.hpos(hpos_for_read), .vpos(vpos_for_pf),
	.de(video_de), .layer_en(map_ctrl_l0[2]),
	.new_line(layer_new_line),
	.scroll_x(fg_scroll_x), .scroll_y(fg_scroll_y),
	.xoff(osd_fg_xoff), .yoff(osd_fg_yoff),
	.gfx_bank(16'd0),
	.vram_addr(fg_vram_addr), .vram_data(fg_vram_data),
	.rom_req(arb_fg_req), .rom_addr(arb_fg_addr),
	.rom_data(arb_fg_data), .rom_valid(arb_fg_valid),
	.opaque(fg_opaque), .pen_index(fg_pen)
);

// ── Sprite ROM su DDR3 (pattern Darius2WarriorBlade) ────────────────────────
// MRA byte $1C0000-$23FFFF (512KB sei440) → DDR3 offset 0.
// Elimina contesa SDRAM Port 0: SPR ora ha bus dedicato + cache 2-way.
wire is_spr_dl     = ioctl_download && ioctl_wr && (ioctl_index == 16'd0) &&
                     (ioctl_addr >= 27'h1C0000) && (ioctl_addr < 27'h240000);
wire [27:0] ddr_spr_waddr = {1'b0, ioctl_addr - 27'h1C0000};

// Toggle ioctl → DDRAM write
reg ddr_we_req = 1'b0;
reg ioctl_wr_prev_ddr = 1'b0;
always @(posedge clk_sys) begin
	ioctl_wr_prev_ddr <= ioctl_wr;
	if (ioctl_wr && !ioctl_wr_prev_ddr && is_spr_dl) ddr_we_req <= ~ddr_we_req;
end
wire ddr_we_ack;

// Latched data (mantenuto stabile fino a ack)
reg [27:0] ddr_waddr_lat;
reg [15:0] ddr_wdata_lat;
always @(posedge clk_sys) begin
	if (is_spr_dl) begin
		ddr_waddr_lat <= ddr_spr_waddr;
		ddr_wdata_lat <= ioctl_dout;
	end
end

// Cache ↔ DDR3 wires
wire [27:0] ddr_rdaddr;
wire [31:0] ddr_rdata;
wire        ddr_rd_req;
wire        ddr_rd_ack;

// DDR bus interfaces (pattern Taito F2 ddr_if + ddr_mux)
ddr_if ddr_host();    // bus reale → pin DDRAM_* (game: sprite+rotate)
ddr_if ddr_spr();     // client A: sprite ROM
ddr_if ddr_rot();     // client B: rotate framebuffer
ddr_if ddr_ss();      // savestate client (memory_stream) → gate → pin

// ── Savestate bus (ssbus) + save_state_data ─────────────────────────────
// SS_IDX_* = indice univoco di ogni blocco di stato salvato.
localparam SS_IDX_WORKRAM = 0;   // main work RAM (ram_lo/hi) — contiene lo score
// slave: 0=workram,1=txt,2=scroll; 3=shared; 4=bg,5=fg,6=pal; 7=sprite;
// 8=V30 main regs; 9=V30 sub regs; 10=z80_ram; 11=Sub work RAM.
localparam SS_NSLAVES     = 15;  // 12=regs Z80 (T80s REG/DIR), 13=shadow YM3812, 14=glue audio (ULTIMO: commit=replay)
localparam SS_MS_COUNT    = 16;  // memory_stream COUNT (>= SS_NSLAVES, pot. di 2)

// ss_busy dichiarato sopra (vicino a paused_safe che lo usa)
ssbus_if ssbus();
ssbus_if ssb[SS_NSLAVES]();

// raiden_ss_manager: coordinatore frame-aligned. ss_save/ss_load NON triggano
// il DMA direttamente (partirebbe a metà frame). Il manager mette in pausa PRIMA,
// aspetta paused_safe stabile (confine frame), POI pulsa read/write_start.
wire ss_mgr_wr, ss_mgr_rd, ss_cpu_reload;   // ss_mgr_pause dichiarato sopra (blocco paused_safe)
raiden_ss_manager u_ss_mgr (
	.clk           (clk_sys),
	.reset         (reset),
	.ss_save       (ss_save),
	.ss_load       (ss_load),
	.paused_safe   (paused_safe & cpus_ss_ready),   // cattura/load SOLO con entrambe le V30 a confine istruzione
	.ss_busy       (ss_busy),
	.ss_pause      (ss_mgr_pause),
	.write_start   (ss_mgr_wr),
	.read_start    (ss_mgr_rd),
	.ss_cpu_reload (ss_cpu_reload)
);

// save_state_data: DMA stato ↔ DDR (regione SS3E000000, slot ss_slot).
// Trigger dal manager (frame-aligned), NON da ss_save/ss_load diretti.
save_state_data #(.COUNT(SS_MS_COUNT)) u_ss_data (
	.clk         (clk_sys),
	.reset       (reset),
	.ddr         (ddr_ss),
	.read_start  (ss_mgr_rd),
	.write_start (ss_mgr_wr),
	.index       (ss_slot),
	.busy        (ss_busy),
	.ssbus       (ssbus)
);

// ssbus_mux: multiplexa gli slave ssb[] (masters del mux) verso ssbus (slave).
ssbus_mux #(.COUNT(SS_NSLAVES)) u_ssbus_mux (
	.clk    (clk_sys),
	.masters(ssb),
	.slave  (ssbus)
);

wire        ss_hold, ss_ddr_grant;   // dal ss_ddr_gate (sotto); dichiarati qui per il mux

raiden_ddr_mux u_ddr_mux (
	.clk     (clk_sys),
	.ss_hold (ss_hold),   // durante SS blocca l'emissione sprite/rotate alla sorgente
	.x       (ddr_host),
	.a       (ddr_spr),
	.b       (ddr_rot)
);

// ── Savestate DDR gating ────────────────────────────────────────────────
// Il gioco (ddr_host, sprite+rotate) e il savestate (ddr_ss) condividono i pin
// DDRAM_*. ss_ddr_gate instrada game↔ss su ss_busy, con drain per non troncare
// burst in volo. A SS idle: pin ← game (comportamento identico a prima).
// DDRAM_ADDR (29 bit) = ddr_host.addr[31:3], ESATTAMENTE come l'assegnazione
// originale (assign DDRAM_ADDR = ddr_host.addr[31:3]). Prima avevo troncato a
// [28:3] buttando i bit 31:29 → sprite ROM (0x30000000) con bit alto tagliato →
// sprite rotti. NON aggiungere zeri: addr è già l'indirizzo giusto per DDRAM_ADDR.
wire [28:0] game_DDRAM_ADDR = ddr_host.addr[31:3];
wire        ss_tx_inflight = ddr_ss.read | ddr_ss.write;

ss_ddr_gate #(.AW(29), .DRAIN_TH(3)) u_ss_ddr_gate (
	.clk             (clk_sys),
	.reset           (reset),
	.ss_busy         (ss_busy),
	.ss_tx_inflight  (ss_tx_inflight),
	// game (ddr_host): l'emissione sprite/rotate è già bloccata alla sorgente dal
	// raiden_ddr_mux (ss_hold → x.read/write=0), quindi ddr_host.read/write sono già 0
	// durante SS. Il gate conta i beat dei burst già in volo prima di concedere.
	.game_burstcnt   (ddr_host.burstcnt),
	.game_addr       (game_DDRAM_ADDR),
	.game_rd         (ddr_host.read),
	.game_din        (ddr_host.wdata),
	.game_be         (ddr_host.byteenable),
	.game_we         (ddr_host.write),
	// savestate (ddr_ss)
	.ss_burstcnt     (ddr_ss.burstcnt),
	.ss_addr         (ddr_ss.addr[31:3]),
	.ss_rd           (ddr_ss.read),
	.ss_din          (ddr_ss.wdata),
	.ss_be           (ddr_ss.byteenable),
	.ss_we           (ddr_ss.write),
	// controller
	.DDRAM_BUSY      (DDRAM_BUSY),
	.DDRAM_DOUT_READY(DDRAM_DOUT_READY),
	.DDRAM_BURSTCNT  (DDRAM_BURSTCNT),
	.DDRAM_ADDR      (DDRAM_ADDR),
	.DDRAM_RD        (DDRAM_RD),
	.DDRAM_DIN       (DDRAM_DIN),
	.DDRAM_BE        (DDRAM_BE),
	.DDRAM_WE        (DDRAM_WE),
	.ss_hold         (ss_hold),
	.ss_ddr_grant    (ss_ddr_grant)
);

// rdata/busy: entrambi i client vedono il ritorno DDR. Il gate garantisce che
// solo il client attivo (ss_ddr_grant) abbia transazioni in volo.
assign ddr_host.rdata       = DDRAM_DOUT;
// rdata_ready al gioco solo quando NON è concesso al SS (i beat SS non vanno ai client).
assign ddr_host.rdata_ready = ss_ddr_grant ? 1'b0 : DDRAM_DOUT_READY;
// busy al gioco alto se: SS ha il grant, OPPURE ss_hold (fase di drain): così i client
// sprite/rotate stallano e non emettono → il bus si drena → il gate concede al SS.
assign ddr_host.busy        = (ss_ddr_grant | ss_hold) ? 1'b1 : DDRAM_BUSY;
assign ddr_ss.rdata         = DDRAM_DOUT;
assign ddr_ss.rdata_ready   = ss_ddr_grant ? DDRAM_DOUT_READY : 1'b0;
assign ddr_ss.busy          = ss_ddr_grant ? DDRAM_BUSY : 1'b1;

raiden_sprite_ddr_client u_ddram (
	.clk     (clk_sys),
	.wraddr  (ddr_waddr_lat),
	.din     (ddr_wdata_lat),
	.we_byte (1'b0),
	.we_req  (ddr_we_req),
	.we_ack  (ddr_we_ack),
	.rdaddr  (ddr_rdaddr),
	.dout    (ddr_rdata),
	.rd_req  (ddr_rd_req),
	.rd_ack  (ddr_rd_ack),
	.ddr     (ddr_spr)
);

// ── Sprite renderer (SEI0211) ───────────────────────────────────────────────
wire        spr_opaque;
wire [10:0] spr_pen;
wire  [1:0] spr_pri;
wire [10:0] spr_vram_addr;
wire [15:0] spr_vram_data;

// Sprite ROM cache (DDR3 backend)
wire [23:0] spr_cache_addr;
wire        spr_cache_req_pulse;
wire [31:0] spr_cache_data;
wire        spr_cache_valid;

raiden_sprite_rom_cache #(
	.DDR_BASE_ADDR(28'h0000000)
) u_spr_cache (
	.clk      (clk_sys),
	.reset    (reset),
	.req_addr (spr_cache_addr),
	.req_pulse(spr_cache_req_pulse),
	.resp_data(spr_cache_data),
	.resp_valid(spr_cache_valid),
	.ddr_addr (ddr_rdaddr),
	.ddr_req  (ddr_rd_req),
	.ddr_data (ddr_rdata),
	.ddr_ack  (ddr_rd_ack)
);

// Adapter sprite renderer (level rom_req → rising-edge req_pulse)
wire        spr_renderer_rom_req;
wire [23:0] spr_renderer_rom_addr;
reg         spr_renderer_rom_req_prev;
always @(posedge clk_sys) spr_renderer_rom_req_prev <= spr_renderer_rom_req;
assign spr_cache_addr      = spr_renderer_rom_addr;
assign spr_cache_req_pulse = spr_renderer_rom_req & ~spr_renderer_rom_req_prev;

Raiden_sprite_renderer u_spr (
	.clk(clk_sys), .reset(reset), .ce_pix(ce_pix),
	.hpos(hpos_logic), .vpos(vpos_logic),          // read NORMALE: il flip lo fa il write 240-x
	.de(video_de), .layer_en(map_ctrl_l0[4]),    // bit4 = sprite enable
	.new_line(layer_new_line),
	.flip_screen(flip_screen),
	.xoff(osd_spr_xoff), .yoff(osd_spr_yoff),
	.decode_mode(spr_decode_mode_r),
	.spr_addr(spr_vram_addr), .spr_data(spr_vram_data),
	.rom_req(spr_renderer_rom_req), .rom_addr(spr_renderer_rom_addr),
	.rom_data(spr_cache_data), .rom_valid(spr_cache_valid),
	.opaque(spr_opaque), .pen_index(spr_pen), .pri_code(spr_pri)
);

// ── Tile ROM arbiter (BG/FG/Sprite; MG/text slot tied off) ──
tile_rom_arbiter u_arb (
	.clk(clk_sys), .reset(reset), .hblank(HBlank),
	.r0_req(arb_bg_req),  .r0_addr(arb_bg_addr),  .r0_data(arb_bg_data),  .r0_valid(arb_bg_valid),
	.r1_req(1'b0), .r1_addr(24'd0), .r1_data(), .r1_valid(),                         // MG removed (Raiden plain)
	.r2_req(arb_fg_req),  .r2_addr(arb_fg_addr),  .r2_data(arb_fg_data),  .r2_valid(arb_fg_valid),
	.r3_req(1'b0), .r3_addr(24'd0), .r3_data(), .r3_valid(),                          // SPR ora su DDR3 (raiden_sprite_rom_cache)
	.r4_req(1'b0), .r4_addr(24'd0), .r4_data(), .r4_valid(),
	.tile_req(game_tile_req), .tile_addr(game_tile_addr), .tile_kind(game_tile_kind),
	.tile_data(game_tile_data), .tile_valid(game_tile_valid)
);

// Pixel pipeline MAME RAIDEN (raiden.cpp:286-358 draw_sprites + 361-397 update):
//   Render order MAME: BG (priority 1) → FG (priority 2) → Text (priority 4)
//   Sprite con pri_mask:
//     pri=0 → SKIP (gestito nel renderer, opaque=0)
//     pri=1 → mask = GFX_PMASK_4 | GFX_PMASK_2 → sotto FG e Text → SOPRA SOLO BG
//     pri=2,3 → mask = GFX_PMASK_4 → sotto Text only → SOPRA BG e FG
// Nota: nessun sprite "above all" (sopra Text). Text è sempre sopra tutti gli sprite.
// PROBE: latch quando le 2 CPU accettano IRQ (falling edge irq_pending).
// In fase backdrop bypassiamo palette per mostrare stato direttamente:
//   nero    = nessuno IRQ acceptato
//   rosso   = solo Main IRQ
//   verde   = solo Sub IRQ
//   giallo  = entrambi
wire main_irq_pending_probe;
wire sub_irq_pending_probe;
reg  sub_irq_prev, main_irq_prev;
reg  sub_irq_seen, main_irq_seen;
always @(posedge clk_sys) begin
	if (reset) begin
		sub_irq_seen <= 0; main_irq_seen <= 0;
		sub_irq_prev <= 0; main_irq_prev <= 0;
	end else begin
		sub_irq_prev  <= sub_irq_pending_probe;
		main_irq_prev <= main_irq_pending_probe;
		if (sub_irq_prev  && !sub_irq_pending_probe)  sub_irq_seen  <= 1;
		if (main_irq_prev && !main_irq_pending_probe) main_irq_seen <= 1;
	end
end
wire [10:0] backdrop_pen = 11'h000;
`ifdef V30_SIM_NOSPR
// Isolamento layer (solo sim): sprite spenti -> cosa resta e' BG/FG/TXT.
wire spr_above_fg = 1'b0;
wire spr_above_bg = 1'b0;
`else
wire spr_above_fg = spr_opaque & (spr_pri >= 2'd2);   // pri=2,3 sopra FG
wire spr_above_bg = spr_opaque & (spr_pri == 2'd1);   // pri=1 sopra BG (sotto FG)
`endif

// OSD palette base override: i 2 bit [9:8] del pen_index = palette region (256 entry).
// Renderer originale produce: BG=0x0xx, FG=0x1xx, SPR=0x2xx, TXT=0x3xx.
// Selettore OSD permette di switchare runtime quale region usa ciascun layer.
//   bg_base_sel maps 0→00, 1→01, 2→10, 3→11
//   fg_base_sel default 01, swap con 00/10/11
//   spr_base_sel default 10, swap con 00/01/11
//   txt_base_sel default 11, swap con 00/01/10
// Palette base per layer hardcodate ai default (gli OSD base-swap erano bit
// morti senza voce → status 0 → ramo sel=0 = def_base). Rimossa function +
// 4 mux per liberare ALM. Comportamento HW identico.
wire [1:0] bg_base_eff  = 2'd0;   // palette base 0x000
wire [1:0] fg_base_eff  = 2'd1;   // palette base 0x100
wire [1:0] spr_base_eff = 2'd2;   // palette base 0x200
wire [1:0] txt_base_eff = 2'd3;   // palette base 0x300
wire [10:0] bg_pen_eff  = {bg_pen[10],  bg_base_eff,  bg_pen[7:0]};
wire [10:0] fg_pen_eff  = {fg_pen[10],  fg_base_eff,  fg_pen[7:0]};
wire [10:0] spr_pen_eff = {spr_pen[10], spr_base_eff, spr_pen[7:0]};
wire [10:0] text_pen_eff= {text_pen[10],txt_base_eff, text_pen[7:0]};

// Ordine: Text > sprite(pri=2,3) > FG > sprite(pri=1) > BG > backdrop
wire [10:0] composite_pen =
                    text_opaque   ? text_pen_eff :
                    spr_above_fg  ? spr_pen_eff  :
                    fg_opaque     ? fg_pen_eff   :
                    spr_above_bg  ? spr_pen_eff  :
                    bg_opaque     ? bg_pen_eff   :
                                    backdrop_pen;
// is_backdrop = nessun layer opaco. MAME: bitmap.fill(black_pen) prima di
// disegnare i layer → backdrop = NERO fisso, NON palette[0]. Usato sotto per
// forzare nero invece di leggere palette[0] (che il gioco usa per scene-color).
wire is_backdrop = !text_opaque & !spr_above_fg & !fg_opaque & !spr_above_bg & !bg_opaque;

// Timing closure: registriamo pal_b_addr per spezzare path lungo hpos→composite_pen→BRAM.
// video_de + is_backdrop ritardati di 1 clk_sys per allineamento.
wire [10:0] pal_b_addr_next = ~video_de ? backdrop_pen : composite_pen;
reg video_de_d;
reg is_backdrop_d;
always @(posedge clk_sys) begin
	pal_b_addr    <= pal_b_addr_next;
	video_de_d    <= video_de;
	is_backdrop_d <= is_backdrop;
end

// is_backdrop_d forza nero (MAME bitmap.fill(black_pen)) → no scene-color
// dell'area "vuota" tra layer (es. lampo blu durante coin insert).
wire [7:0] video_r = (video_de_d & ~is_backdrop_d) ? pal_b_r : 8'h00;
wire [7:0] video_g = (video_de_d & ~is_backdrop_d) ? pal_b_g : 8'h00;
wire [7:0] video_b = (video_de_d & ~is_backdrop_d) ? pal_b_b : 8'h00;

assign CLK_VIDEO = clk_sys;

// Pause overlay: dim video + logo 48x48 al centro durante pausa.
// OSD "Clean Pause" (status[18]): ON=video raw senza addon, OFF=overlay attivo.
// Output su bus intermedi av_* (poi H-Shift/V-Shift/H-Size → VGA_*).
wire [7:0] av_r, av_g, av_b;
pause_overlay u_pause_ovl (
	.clk         (clk_sys),
	.pause       (pause),
	.clean       (status[18]),
	.vblank      (VBlank),
	.rotate_en   (rotate_en),
	.render_x_in (render_x[8:0]),
	.render_y_in (render_y),
	.rgb_r_in    (video_r),
	.rgb_g_in    (video_g),
	.rgb_b_in    (video_b),
	.rgb_r_out   (av_r),
	.rgb_g_out   (av_g),
	.rgb_b_out   (av_b)
);

// ── Analog H-Shift / V-Shift / H-Size (pattern Blood Bros) ───────────────────
// Agiscono SOLO sull'uscita analogica: comprimono/spostano l'immagine senza
// toccare pixel clock né sync (HSync/refresh nativi intatti, zero jitter).
localparam int H_TOTAL_RD = 320;
localparam int V_TOTAL_RD = 263;

// H-Size: unico controllo bidirezionale (status[66:62], two's complement 5-bit):
//   0        = nativo (bypass)
//   +1..+15  = enlarge (immagine piu' larga)  → read piu' lento (quarti di ciclo)
//   -1..-16  = shrink  (immagine piu' stretta) → read piu' veloce
// Step = 1 quarto di ciclo = 1.56% (vedi accumulatore rd_acc sotto). Modulo
// STANDARD applicabile a tutti i core: un solo controllo enlarge↔shrink.
reg  signed [4:0] hsize_s;
always @(posedge clk_sys) if (ce_pix) hsize_s <= $signed(status[66:62]);
wire hsize_active = (hsize_s != 5'sd0);
// H-Position: sposta il contenuto (non il sync). Signed ±48: >0 a destra, <0 a
// sinistra. Non desincronizza (HSync intatto). Bitfield status[104:98] (7 bit):
// 0..48 = +0..+48 ; 79..127 = -48..-1 (encoding come H-Shift).
reg  [6:0] hsize_hoff_d;
always @(posedge clk_sys) if (ce_pix) hsize_hoff_d <= status[104:98];
wire signed [8:0] hsize_hoffset = (hsize_hoff_d <= 7'd48)
	? $signed({2'b0, hsize_hoff_d})
	: $signed({2'b0, hsize_hoff_d}) - 9'sd128;

// H-Shift: bitfield 0..48 = +0..+48 (destra), 49..63 = -15..-1 (status[97:92]).
reg [5:0] osd_vga_hshift_d;
always @(posedge clk_sys) if (ce_pix) osd_vga_hshift_d <= status[97:92];
wire [8:0] hshift_tap = (osd_vga_hshift_d <= 6'd48)
	? {3'd0, osd_vga_hshift_d}
	: (9'(H_TOTAL_RD) - (9'd64 - {3'd0, osd_vga_hshift_d}));
reg [H_TOTAL_RD-1:0] hsync_shreg;
always @(posedge clk_sys) if (ce_pix) hsync_shreg <= {hsync_shreg[H_TOTAL_RD-2:0], HSync};
reg vga_hs_reg;
always @(posedge clk_sys) if (ce_pix)
	vga_hs_reg <= (hshift_tap == 9'd0) ? HSync : hsync_shreg[hshift_tap - 9'd1];

// V-Shift: signed ±32 righe (status[61:56]). line_tick = fine linea.
wire line_tick = ce_pix && (timing_hpos == 10'(H_TOTAL_RD - 1));
reg signed [5:0] osd_vga_vshift_d;
always @(posedge clk_sys) if (line_tick) osd_vga_vshift_d <= $signed(status[61:56]);
wire [8:0] vshift_tap = osd_vga_vshift_d[5]
	? (9'(V_TOTAL_RD) + {{3{osd_vga_vshift_d[5]}}, osd_vga_vshift_d})
	: {3'd0, osd_vga_vshift_d};
reg [V_TOTAL_RD-1:0] vsync_line_shreg;
always @(posedge clk_sys) if (line_tick) vsync_line_shreg <= {vsync_line_shreg[V_TOTAL_RD-2:0], VSync};
reg vga_vs_reg;
always @(posedge clk_sys) if (line_tick)
	vga_vs_reg <= (vshift_tap == 9'd0) ? VSync : vsync_line_shreg[vshift_tap - 9'd1];

// Read rate a QUARTI di ciclo (step 1.56%): accumulatore in unita' di quarto.
// Periodo pixel = (64 + hsize) quarti = (16 + hsize/4) cicli. hsize±1 = ±0.25
// ciclo = step molto fine. Reset all'HSync → pattern DETERMINISTICO per riga
// (identico ogni frame) → niente shimmer dinamico (irregolarita' statica minima,
// 1 pixel su 4 di 1px per il livello ±1, invisibile).
reg  vga_hs_reg_d;
always @(posedge clk_sys) vga_hs_reg_d <= vga_hs_reg;
wire shifted_hs_rise = vga_hs_reg & ~vga_hs_reg_d;
wire [7:0] rd_period = 8'd64 + {{3{hsize_s[4]}}, hsize_s};  // quarti, hsize -16..+15 → 48..79
reg  [7:0] rd_acc;
wire rd_tick = (rd_acc + 8'd4) >= {1'b0, rd_period};
always @(posedge clk_sys) begin
	if (shifted_hs_rise)  rd_acc <= 8'd0;
	else if (rd_tick)     rd_acc <= rd_acc + 8'd4 - {1'b0, rd_period};  // sottrai periodo, tieni resto
	else                  rd_acc <= rd_acc + 8'd4;                      // +4 quarti = 1 ciclo
end
wire rd_ce = (hsize_s == 5'sd0) ? ce_pix : rd_tick;

wire [7:0] str_r, str_g, str_b;
wire       str_hs, str_vs, str_hb, str_vb;
analog_hsize u_analog_hsize (
	.clk      (clk_sys),
	.pxl_cen  (ce_pix),
	.pxl2_cen (rd_ce),
	.hsize    (hsize_s),
	.hoffset  (hsize_hoffset),
	.r_in     (av_r), .g_in (av_g), .b_in (av_b),
	.hs_in    (vga_hs_reg),
	.vs_in    (vga_vs_reg),
	.hb_in    (HBlank | VBlank),
	.vb_in    (VBlank),
	.r_out    (str_r), .g_out (str_g), .b_out (str_b),
	.hs_out   (str_hs), .vs_out (str_vs),
	.hb_out   (str_hb), .vb_out (str_vb)
);

// Finestra DE per l'OSD: apre all'attivo nativo (ritardato 1 riga), chiude a
// larghezza stretchata piena (pattern Blood Bros).
reg vblank_1l;
always @(posedge clk_sys) if (line_tick) vblank_1l <= VBlank;
wire native_active = ~(HBlank | vblank_1l);
reg  native_active_d;
always @(posedge clk_sys) if (ce_pix) native_active_d <= native_active;
wire native_rise = native_active & ~native_active_d;
wire str_active = ~str_hb;
reg  str_active_d;
always @(posedge clk_sys) if (rd_ce) str_active_d <= str_active;
wire str_fall = str_active_d & ~str_active;
reg de_osd;
always @(posedge clk_sys) begin
	if      (native_rise) de_osd <= 1'b1;
	else if (str_fall)    de_osd <= 1'b0;
end

// Output analogico: H-Size attivo → dal modulo (incorpora shift); bypass → shiftato.
// ─── GAMMA CORRECTION (gamma_fast) ───────────────────────────────────────
// Il core non usa video_mixer, quindi la gamma va agganciata a mano: `gamma_bus`
// era scollegata e nessun correttore era istanziato -> la voce OSD non faceva
// nulla.
// Usato gamma_fast e NON gamma_corr: prende `gamma_bus` come inout e lo
// decodifica da solo (niente spacchettamento a mano, che avevo sbagliato di un
// bit), ha tre LUT parallele lette in un colpo invece della sequenza a 3 cicli,
// e ha gia' DE in ingresso e in uscita, che e' quello che serve qui.
// Sta DOPO il mux hsize_active, cosi' i due rami (modulo CRT `str_*` e percorso
// diretto `av_*`) prendono lo stesso ritardo. RGB e sync escono ritardati
// INSIEME, quindi la posizione dell'immagine rispetto al sync non cambia.
wire [7:0] vid_r_pre  = hsize_active ? str_r  : av_r;
wire [7:0] vid_g_pre  = hsize_active ? str_g  : av_g;
wire [7:0] vid_b_pre  = hsize_active ? str_b  : av_b;
wire       vid_hs_pre = hsize_active ? str_hs : vga_hs_reg;
wire       vid_vs_pre = hsize_active ? str_vs : vga_vs_reg;
wire       vid_de_pre = hsize_active ? de_osd : ~(HBlank | VBlank);
wire       vid_ce_pix = hsize_active ? rd_ce  : ce_pix;

wire [23:0] vid_rgb_out;
wire        vid_hs_out, vid_vs_out, vid_de_out;
gamma_fast u_gamma (
	.clk_vid   (clk_sys),
	.ce_pix    (vid_ce_pix),
	.gamma_bus (gamma_bus),
	.HSync     (vid_hs_pre),
	.VSync     (vid_vs_pre),
	.HBlank    (~vid_de_pre),
	.VBlank    (1'b0),
	.DE        (vid_de_pre),
	.RGB_in    ({vid_r_pre, vid_g_pre, vid_b_pre}),
	.HSync_out (vid_hs_out),
	.VSync_out (vid_vs_out),
	.HBlank_out(),
	.VBlank_out(),
	.DE_out    (vid_de_out),
	.RGB_out   (vid_rgb_out)
);

assign VGA_R  = vid_rgb_out[23:16];
assign VGA_G  = vid_rgb_out[15:8];
assign VGA_B  = vid_rgb_out[7:0];
assign VGA_HS = vid_hs_out;
assign VGA_VS = vid_vs_out;
assign CE_PIXEL = vid_ce_pix;

// Aspect ratio: Original = 4:3 arcade display, Full Screen = 0:0.
// Quando ruota (TATE) swap ARX/ARY: la scena è già ruotata dal framebuffer
// dell'HPS scaler → da 4:3 landscape diventa 3:4 portrait.
wire [11:0] arx = (!ar) ? (rotate_en ? 12'd3 : 12'd4) : (ar - 1'd1);
wire [11:0] ary = (!ar) ? (rotate_en ? 12'd4 : 12'd3) : 12'd0;

// Integer scaling forzato: Narrower HV-Integer (default), V-Integer, HV-Integer.
// Normal scaling rimosso perché senza setup utente preciso dà sempre risultato sbagliato.
video_freak video_freak
(
	.CLK_VIDEO(clk_sys),
	.CE_PIXEL(hsize_active ? rd_ce : ce_pix),
	.VGA_VS(VSync),
	.HDMI_WIDTH(HDMI_WIDTH),
	.HDMI_HEIGHT(HDMI_HEIGHT),
	.VGA_DE(VGA_DE),
	.VIDEO_ARX(VIDEO_ARX),
	.VIDEO_ARY(VIDEO_ARY),
	.VGA_DE_IN(vid_de_out),
	.ARX(arx),
	.ARY(ary),
	.CROP_SIZE(12'd0),
	.CROP_OFF(5'd0),
	.SCALE((status[7:5] == 3'd0) ? 3'd0 :   // Normal (default)
	        (status[7:5] == 3'd1) ? 3'd1 :   // V-Integer
	        (status[7:5] == 3'd2) ? 3'd4 :   // HV-Integer
	                                3'd2)    // Narrower HV-Integer
);

// LED: blink during download
assign LED_USER = ioctl_download;

// ============================================================
// Screen rotation (TATE) - pattern Taito F2 (ddr_if + FIFO)
// ============================================================
// status[2:1]: 00=No rotate, 01=CCW (TATE), 10=CW. status[3]=Flip 180.
// screen_rotate snoopa VGA_* a CLK_VIDEO; FIFO 1024 entry assorbe i write;
// ddr_mux arbitra tra sprite (a) e rotate (b).
wire [1:0] rotate_sel = status[2:1];
wire rotate_en  = (rotate_sel != 2'd0);
wire rotate_ccw = (rotate_sel == 2'd1);
wire flip_180   = status[3];
wire video_rotated;

// VGA_SCALER deve restare 0: il CRT analogico non deve MAI cambiare routing
// quando attivi rotate. La rotazione HDMI e' gestita da screen_rotate via
// framebuffer HPS, NON tramite VGA_SCALER. Pattern gia' applicato in
// SkySmasher (SkySmasher.sv:149-153) e mai portato qui: con VGA_SCALER
// legato a video_rotated, abilitare la rotazione dirottava anche l'uscita
// analogica.
assign VGA_SCALER = 0;

wire [28:0] rot_addr;
wire [63:0] rot_data;
wire  [7:0] rot_be;
wire        rot_we;

screen_rotate u_screen_rotate
(
	.CLK_VIDEO     (clk_sys),
	.CE_PIXEL      (hsize_active ? rd_ce : ce_pix),

	.VGA_R         (VGA_R),
	.VGA_G         (VGA_G),
	.VGA_B         (VGA_B),
	.VGA_HS        (VGA_HS),
	.VGA_VS        (VGA_VS),
	.VGA_DE        (VGA_DE),

	.rotate_ccw    (rotate_ccw),
	.no_rotate     (~rotate_en),
	.flip          (flip_180),
	.video_rotated (video_rotated),

	.FB_EN         (FB_EN),
	.FB_FORMAT     (FB_FORMAT),
	.FB_WIDTH      (FB_WIDTH),
	.FB_HEIGHT     (FB_HEIGHT),
	.FB_BASE       (FB_BASE),
	.FB_STRIDE     (FB_STRIDE),
	.FB_VBL        (FB_VBL),
	.FB_LL         (FB_LL),

	.DDRAM_CLK     (),
	.DDRAM_BUSY    (1'b0),         // FIFO assorbe (pattern Taito F2)
	.DDRAM_BURSTCNT(),
	.DDRAM_ADDR    (rot_addr),
	.DDRAM_DIN     (rot_data),
	.DDRAM_BE      (rot_be),
	.DDRAM_WE      (rot_we),
	.DDRAM_RD      ()
);

raiden_rotate_fifo u_rot_fifo (
	.clk      (clk_sys),
	.rot_addr (rot_addr),
	.rot_data (rot_data),
	.rot_be   (rot_be),
	.rot_we   (rot_we),
	.ddr      (ddr_rot)
);

`ifdef V30_SIM_PROBES
// Probe MIXER: chi disegna il pixel (x,y) scelto? Stampa una riga per frame
// con lo stato di TUTTI i layer nel punto = identifica il colpevole del blocco.
integer dbg_px_n = 0;
always @(posedge clk_sys) begin
	if (ce_pix && video_de && dbg_px_n < 30 &&
	    ((hpos_for_read == 10'd220 && vpos_for_pf == 9'd80) ||
	     (hpos_for_read == 10'd180 && vpos_for_pf == 9'd140))) begin
		dbg_px_n <= dbg_px_n + 1;
		$display("[mix] x=%0d y=%0d | txt=%b(%03h) sprA=%b sprB=%b spr(op=%b pri=%0d pen=%03h) fg=%b(%03h) bg=%b(%03h)",
		         hpos_for_read, vpos_for_pf, text_opaque, text_pen,
		         spr_above_fg, spr_above_bg, spr_opaque, spr_pri, spr_pen,
		         fg_opaque, fg_pen, bg_opaque, bg_pen);
	end
end
`endif

`ifdef V30_SIM_PROBES
// Probe savestate (scene): comando OSD -> ss_ui -> manager -> DMA.
reg dbg_ssl_p, dbg_ssb_p, dbg_ssmp_p;
integer dbg_hb = 0;
integer dbg_ss_dur = 0;
integer dbg_mgr_dur = 0;
integer dbg_gate_tr = 0;
reg dbg_post_tr = 0;
always @(posedge clk_sys) begin
	dbg_ssl_p <= ss_load;
	dbg_ssb_p <= ss_busy;
	if (ss_load && !dbg_ssl_p)  $display("[ssui] ss_load EDGE (slot=%0d) status108=%b status107=%b", ss_slot, status[108], status[107]);
	if (ss_mgr_pause && !dbg_ssmp_p) $display("[ssui] ss_mgr_pause ALTO");
	dbg_ssmp_p <= ss_mgr_pause;
	if (ss_mgr_pause && (dbg_hb % 500000) == 0)
		$display("[ssui] wait: paused_safe=%b main_idle=%b sub_idle=%b z80_rdy=%b", paused_safe, main_cpu_idle, sub_cpu_idle, z80_ss_ready);
	if (ss_busy && !dbg_ssb_p) dbg_gate_tr <= 40;
	if (!ss_busy && dbg_ssb_p) dbg_post_tr <= 1;
	if (dbg_post_tr && (dbg_hb % 200000) == 0)
		$display("[sspost] mgr_pause=%b paused_safe=%b reload=%b main_idle=%b sub_idle=%b", ss_mgr_pause, paused_safe, ss_cpu_reload, main_cpu_idle, sub_cpu_idle);
	if (dbg_gate_tr > 0) begin
		dbg_gate_tr <= dbg_gate_tr - 1;
		$display("[ssgate] t-%0d hold=%b grant=%b infl=%b rd=%b addr=%h rdy=%b rdata=%h", dbg_gate_tr, ss_hold, ss_ddr_grant, ss_tx_inflight, ddr_ss.read, ddr_ss.addr, ddr_ss.rdata_ready, ddr_ss.rdata[31:0]);
	end
	dbg_hb <= dbg_hb + 1;
	if (ss_save)                $display("[ssui] ss_save alto!");
	if (ss_busy != dbg_ssb_p)   $display("[ssui] ss_busy=%b", ss_busy);
	// durata di ogni operazione SS: un save che non si chiude non stampa mai
	if (ss_busy && !dbg_ssb_p) dbg_ss_dur <= 0;
	else if (ss_busy) dbg_ss_dur <= dbg_ss_dur + 1;
	else if (!ss_busy && dbg_ssb_p) $display("[ssdur] operazione SS completata in %0d clk", dbg_ss_dur);
	// watchdog: se il manager tiene la pausa troppo a lungo = DEADLOCK
	if (ss_mgr_pause) dbg_mgr_dur <= dbg_mgr_dur + 1;
	else dbg_mgr_dur <= 0;
	if (dbg_mgr_dur == 32'd4000000)
		$display("[SSDEADLOCK] pausa SS bloccata >4M clk: paused_safe=%b main_idle=%b sub_idle=%b z80=%b busy=%b",
		         paused_safe, main_cpu_idle, sub_cpu_idle, z80_ss_ready, ss_busy);
end
`endif

`ifdef V30_SIM_PROBES
// Probe boot-gate (switch --define V30_SIM_PROBES=1): catena main ROM CPU->cache->bridge->porta1 SDRAM.
integer dbg_mr_ev = 0;
reg dbg_gmr_p, dbg_bmr_p, dbg_r1_p, dbg_a1_p;
always @(posedge clk_sys) begin
	dbg_gmr_p <= game_main_req;
	dbg_bmr_p <= bridge_main_req;
	dbg_r1_p  <= sd_req1;
	dbg_a1_p  <= sd_ack1;
	if (dbg_mr_ev < 80) begin
		if (game_main_req != dbg_gmr_p) begin
			dbg_mr_ev <= dbg_mr_ev + 1;
			$display("[mainrom] game_req=%b addr=%h (sdram_ready=%b)", game_main_req, game_main_addr, sdram_ready);
		end
		if (game_main_ready) begin
			dbg_mr_ev <= dbg_mr_ev + 1;
			$display("[mainrom] game_READY data=%h addr=%h", game_main_data, game_main_addr);
		end
		if (bridge_main_req != dbg_bmr_p) begin
			dbg_mr_ev <= dbg_mr_ev + 1;
			$display("[mainrom] bridge_req=%b addr=%h", bridge_main_req, bridge_main_addr);
		end
		if (bridge_main_ready) begin
			dbg_mr_ev <= dbg_mr_ev + 1;
			$display("[mainrom] bridge_READY data=%h", bridge_main_data);
		end
		if (sd_req1 != dbg_r1_p || sd_ack1 != dbg_a1_p) begin
			dbg_mr_ev <= dbg_mr_ev + 1;
			$display("[mainrom] port1 req=%b ack=%b addr=%h", sd_req1, sd_ack1, sd_addr1);
		end
	end
end
`endif

endmodule
