/* This file is part of JTOPL.

 
    JTOPL program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    JTOPL program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with JTOPL.  If not, see <http://www.gnu.org/licenses/>.

    Author: Jose Tejada Gomez. Twitter: @topapate
    Version: 1.0
    Date: 20-6-2020 

*/

module jtopl_acc(
    input                rst,
    input                clk,
    input                cenop,
    input         [17:0] slot,
    input                rhy_en,
    input  signed [12:0] op_result,
    input                zero,
    input                op,  // 0 for modulator operators
    input                con, // 0 for modulated connection
    input         [ 7:0] fmvol0, fmvol1, fmvol2, fmvol3, fmvol4,
    input         [ 7:0] fmvol5, fmvol6, fmvol7, fmvol8, // Q4.4 per canale (0x10=1.0x)
    output signed [15:0] snd
);

wire               sum_en;
wire signed [13:0] op2x;
wire               rhy2x;

// all rhythm channels are amplified by two
// given the data path latency, slot 16(-1) data enters at slot 6(-1) and so on
// upstream fix: rhythm ops land at slots 0-5; old mask [7:2] missed HH@slot1
// -> some percussion (explosions) played at HALF volume
assign rhy2x  = rhy_en && |slot[5:0];
assign sum_en = op | con;
assign op2x   = rhy2x ? {op_result, 1'b0} : {op_result[12],op_result};

// Gain per-canale (OSD mixer). Op N entra al slot (N+6)%18:
//   ch0=slot6/9  ch1=7/10  ch2=8/11  ch3=12/15  ch4=13/16  ch5=14/17
//   ch6=slot0/3  ch7=1/4   ch8=2/5   (in rhythm mode gli slot 0-5 = drums)
reg [7:0] chvol;
always @(*) case(1'b1)
    slot[ 6], slot[ 9]: chvol = fmvol0;
    slot[ 7], slot[10]: chvol = fmvol1;
    slot[ 8], slot[11]: chvol = fmvol2;
    slot[12], slot[15]: chvol = fmvol3;
    slot[13], slot[16]: chvol = fmvol4;
    slot[14], slot[17]: chvol = fmvol5;
    slot[ 0], slot[ 3]: chvol = fmvol6;
    slot[ 1], slot[ 4]: chvol = fmvol7;
    slot[ 2], slot[ 5]: chvol = fmvol8;
    default:            chvol = 8'h10;
endcase
// Q4.4 con clamp a 14-bit (default 0x10 = unita' -> identico a prima)
wire signed [22:0] opg     = (op2x * $signed({1'b0, chvol})) >>> 4;
wire signed [13:0] op2x_g  = (opg >  23'sd8191) ?  14'sd8191 :
                             (opg < -23'sd8192) ? -14'sd8192 : opg[13:0];

// Continuous output
jtopl_single_acc #(.INW(14),.OUTW(16))  u_acc(
    .clk        ( clk       ),
    .cenop      ( cenop     ),
    .op_result  ( op2x_g    ),
    .sum_en     ( sum_en    ),
    .zero       ( zero      ),
    .snd        ( snd       )
);

endmodule
