`timescale 1ns / 1ps


/*  This file is part of JT12 modification adding high precission audio
    mode called FM Overdrive

 
    JT12 program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    JT12 program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with JT12.  If not, see <http://www.gnu.org/licenses/>.

    Based on Sauraen VHDL version of OPN/OPN2, which is based on die shots.

    Author: Jose Tejada Gomez. Twitter: @topapate
    Version: 1.0
    Date: 27-1-2017 

*/


module jt12_op_hd(
    input           rst,
    input           clk,
    input           clk_en,
    input   [9:0]   pg_phase_VIII,
    input   [9:0]   eg_atten_IX,        // output from envelope generator
    input   [2:0]   fb_II,      // voice feedback
    input           xuse_prevprev1,
    input           xuse_prev2,
    input           xuse_internal,
    input           yuse_prev1,
    input           yuse_prev2,
    input           yuse_internal, 
    input           test_214,
    input           fmo_gain,
    input   [1:0]   fmo_sinelut,
    input           fmo_exprom,
    input           fmo_extra,
    
    input           s1_enters,
    input           s2_enters,
    input           s3_enters,
    input           s4_enters,
    input           zero,
    
    output  signed [ 8:0]   op_result,
    output  signed [13:0]   full_result
);

parameter num_ch = 6;

/*  enters  exits
    S1      S2
    S3      S4
    S2      S1
    S4      S3
*/

reg [27:0]  op_result_internal, op_XII; //[13:0] double bits
reg [23:0]  atten_internal_IX; //[11:0] double bits


assign op_result   = fmo_gain ? op_result_internal[26:18] : op_result_internal[27:19];//[13:5] const lenght of 8
assign full_result = fmo_gain ? op_result_internal[26:13] : op_result_internal[27:14]; //[direct] const length of 14

reg         signbit_IX, signbit_X, signbit_XI;
reg [23:0]  totalatten_X; //[11:0] double bits

wire [27:0] prev1, prevprev1, prev2; //[13:0] double bits

reg [27:0] prev1_din, prevprev1_din, prev2_din; //[13:0] double bits

always @(*)
    if( num_ch==3 ) begin
        prev1_din     = s1_enters ? op_result_internal : prev1;
        prevprev1_din = s3_enters ? op_result_internal : prevprev1;
        prev2_din     = s2_enters ? op_result_internal : prev2;
    end else begin // 6 channels
        prev1_din     = s2_enters ? op_result_internal : prev1;
        prevprev1_din = s2_enters ? prev1 : prevprev1;
        prev2_din     = s1_enters ? op_result_internal : prev2;
    end

jt12_sh #( .width(28), .stages(num_ch)) prev1_buffer( //width=14
//  .rst    ( rst       ),
    .clk    ( clk       ),
    .clk_en ( clk_en    ),
    .din    ( prev1_din ),
    .drop   ( prev1     )
);

jt12_sh #( .width(28), .stages(num_ch)) prevprev1_buffer( //width=14
//  .rst    ( rst           ),
    .clk    ( clk           ),
    .clk_en ( clk_en        ),
    .din    ( prevprev1_din ),
    .drop   ( prevprev1     )
);

jt12_sh #( .width(28), .stages(num_ch)) prev2_buffer( //width=14
//  .rst    ( rst       ),
    .clk    ( clk       ),
    .clk_en ( clk_en    ),
    .din    ( prev2_din ),
    .drop   ( prev2     )
);


reg [20:0]  subtresult; //[10:0] one bit more than doubled eg_atten_IX

reg [25:0]  shifter, shifter_2, shifter_3; //[12:0] double bits

// REGISTER/CYCLE 1
// Creation of phase modulation (FM) feedback signal, before shifting
reg [27:0]  x,  y; //[13:0] double bits
reg [28:0]  xs, ys, pm_preshift_II; //[14:0] double x,y bits + 1
reg         s1_II;

always @(*) begin
    casez( {xuse_prevprev1, xuse_prev2, xuse_internal })
        3'b1??: x = prevprev1;
        3'b01?: x = prev2;
        3'b001: x = op_result_internal;
        default: x = 28'd0; //14'd0 match width
    endcase
    casez( {yuse_prev1, yuse_prev2, yuse_internal })
        3'b1??: y = prev1;
        3'b01?: y = prev2;
        3'b001: y = op_result_internal;
        default: y = 28'd0; ////14'd0 match width
    endcase    
    xs = { x[27], x }; // sign-extend  //x[13] last bit of x
    ys = { y[27], y }; // sign-extend  //y[13] last bit of y
end

always @(posedge clk) if( clk_en ) begin
    pm_preshift_II <= xs + ys; // carry is discarded
    s1_II <= s1_enters;
end

/* REGISTER/CYCLE 2-7 (also YM2612 extra cycles 1-6)
   Shifting of FM feedback signal, adding phase from PG to FM phase
   In YM2203, phasemod_II is not registered at all, it is latched on the first edge 
   in add_pg_phase and the second edge is the output of add_pg_phase. In the YM2612, there
   are 6 cycles worth of registers between the generated (non-registered) phasemod_II signal
   and the input to add_pg_phase.     */

reg  [19:0]  phasemod_II; //[9:0] double bits
wire [19:0]  phasemod_VIII; //[9:0] double bits
//reg [28:0]  xs, ys, pm_preshift_II; //[14:0] double x,y bits + 1 - declaration ^^

always @(*) begin
    // Shift FM feedback signal
    if (!s1_II ) // Not S1
        phasemod_II = pm_preshift_II[24:5]; // Bit 0 of pm_preshift_II is never used //[10:1] match phasemod length and discard last bits

    else // S1
        case( fb_II )
            3'd0: phasemod_II = 20'd0; //10'd0 match length
            3'd1: phasemod_II = { {4{pm_preshift_II[28]}}, pm_preshift_II[28:13] }; //4 leave as is [14] last bit [14:9] last bit : match to length (stays the same)
            3'd2: phasemod_II = { {3{pm_preshift_II[28]}}, pm_preshift_II[28:12] }; //3 leave as is [14] last bit [14:8] last bit : match to length (stays the same)
            3'd3: phasemod_II = { {2{pm_preshift_II[28]}}, pm_preshift_II[28:11] }; //2 leave as is [14] last bit [14:7] last bit : match to length (stays the same)
            3'd4: phasemod_II = {    pm_preshift_II[28],   pm_preshift_II[28:10] }; //1 leave as is [14] last bit [14:6] last bit : match to length (stays the same)
            3'd5: phasemod_II = pm_preshift_II[28:9]; //[14:5] last bit : match to length (stays the same)
            3'd6: phasemod_II = pm_preshift_II[27:8]; //[13:4] last bit : match to length (stays the same)
            3'd7: phasemod_II = pm_preshift_II[26:7]; //[12:3] last bit : match to length (stays the same)
        endcase
end

// REGISTER/CYCLE 2-7
//generate
//    if( num_ch==6 )
        jt12_sh #( .width(20), .stages(6)) phasemod_sh( //width(10) double bits // match to length
            .clk    ( clk   ),
            .clk_en ( clk_en),
            .din    ( phasemod_II ),
            .drop   ( phasemod_VIII )
        );
//     else begin
//         assign phasemod_VIII = phasemod_II;
//     end
// endgenerate

// REGISTER/CYCLE 8
reg [ 19:0]  phase; //[9:0] double bits / match phasemod to length
// Sets the maximum number of fanouts for a register or combinational
// cell.  The Quartus II software will replicate the cell and split
// the fanouts among the duplicates until the fanout of each cell
// is below the maximum.

reg [ 15:0]  aux_VIII; //[7:0] double bits

always @(*) begin
    phase   = phasemod_VIII + {pg_phase_VIII, pg_phase_VIII}; //phasemod_VIII + pg_phase_VIII double length of pg_phase_VIII
    aux_VIII= phase[17:2] ^ {16{~phase[18]}}; //[7:0] double bits 8 match length 8 use last bit !@!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
end

always @(posedge clk) if( clk_en ) begin    
    signbit_IX <= phase[19]; //[9] use last bit
end

wire [23:0]  logsin_IX; //[11:0] double bits

jt12_logsin_hd u_logsin_hd (
    .clk          ( clk         ),
    .clk_en       ( clk_en      ),
    .addr         ( aux_VIII    ), //[7:0] use 16 bits, adress range limiting implemented in logsin module
    .fmo_sinelut  ( fmo_sinelut ), //[3:2] leave as is
    .logsin       ( logsin_IX   )

);  


// REGISTER/CYCLE 9
// Sine table    
// Main sine table body

always @(*) begin
    subtresult = {eg_atten_IX, eg_atten_IX} + (fmo_extra ? {1'b0, logsin_IX[23:5]} : logsin_IX[23:4]); //eg_atten_IX double length [11:2] match dimension of double eg_atten_IX
    atten_internal_IX = { subtresult[19:0], (fmo_extra ? logsin_IX[4:1] : logsin_IX[3:0])} | {24{subtresult[20]}}; //[9:0] drop last bit [1:0] take rest 12 match length 10 use last bit
end

wire [21:0] mantissa_X; //[9:0] shifter length minus 4 bits (exponent + mantissa == shifter)
reg  [21:0] mantissa_XI; //[9:0] shifter length minus 4 bits (exponent + mantissa == shifter)
reg  [3:0] exponent_X, exponent_XI; //[3:0] leave as is for now

jt12_exprom_hd u_exprom_hd(
    .clk    ( clk ),
    .clk_en ( clk_en ),
    .fmo_exprom ( fmo_exprom ), 
    .addr   ( atten_internal_IX ), //[7:0] leave 4 bits for exponent and use 12 bits of precission for exprom
    .exp    ( mantissa_X )
);

always @(posedge clk) if( clk_en ) begin
    exponent_X <= atten_internal_IX[23:20]; //[11:8] use last 4 bits for exponent
    signbit_X  <= signbit_IX;
end

always @(posedge clk) if( clk_en ) begin
    mantissa_XI <= mantissa_X;
    exponent_XI <= exponent_X;
    signbit_XI  <= signbit_X;     
end

// REGISTER/CYCLE 11
// Introduce test bit as MSB, 2's complement & Carry-out discarded

always @(*) begin    
    // Floating-point to integer, and incorporating sign bit
    // Two-stage shifting of mantissa_XI by exponent_XI
    shifter = { 3'b001, mantissa_XI }; //3'b000 leave as is
    case( ~exponent_XI[1:0] ) //[1:0] leave as is
        2'b00: shifter_2 = { 1'b0, shifter[25:1] }; // LSB discarded //1'b0 leave as is [12:1] use last bit : drop one bit
        2'b01: shifter_2 = shifter; //leave as is
        2'b10: shifter_2 = { shifter[24:0], 1'b0 }; //[11:0] use 2nd last bit 1'b0 leave as is
        2'b11: shifter_2 = { shifter[23:0], 2'b0 }; //[10:0] use 3nd last bit 2'b0 leave as is
    endcase
    case( ~exponent_XI[3:2] ) //[3:2] leave as is
        2'b00: shifter_3 = {12'b0, shifter_2[25:12]  };//12'b0 we want drop of four so leave 12 as is :12] extend to the rest of the bits
        2'b01: shifter_3 = { 8'b0, shifter_2[25:8] };//8'b0 we want drop of four so leave as is [12:8] extend to the rest of the bits so leave as is
        2'b10: shifter_3 = { 4'b0, shifter_2[25:4] };//4'b0 we want drop of four so leave as is [12:4] extend to the rest of the bits so leave as is
        2'b11: shifter_3 = shifter_2; //leave as is
    endcase
end

always @(posedge clk) if( clk_en ) begin
    // REGISTER CYCLE 11
    op_XII <= ({ test_214, shifter_3, shifter_3[25] } ^ {28{signbit_XI}}) + {13'd0, {11{signbit_XI}}};
    op_result_internal <= ~fmo_gain ? {op_XII[26:0], op_XII[26]} : op_XII;
end

endmodule
