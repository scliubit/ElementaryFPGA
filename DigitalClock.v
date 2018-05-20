`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05/20/2018 11:07:04 PM
// Design Name: DigitalClock
// Module Name: led_run
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module led_run(input clk,rst,
output wire [6:0]SEQ,
output wire [3:0]CS,
output signal,R);

wire clk1;
wire clk2;

wire [1:0]count4;

wire [3:0]DATA;
wire [3:0]DATA0;
wire [3:0]DATA1;
wire [3:0]DATA2;
wire [3:0]DATA3;

//initial
//begin
//  rst=1;
//end

//always #10 begin
//rst=0;
//end

assign signal=clk2;
assign R=rst;

fre_div #(10000,17)fre_div1
(.clk(clk),
.rst(rst),
.clk_out(clk1));

fre_div #(100,10)fre_div2
(.clk(clk1),
.rst(rst),
.clk_out(clk2));

counter #(.WIDTH(2),.M(4)) M4counter
(.clk(clk1),
.rst(rst),
.counter(count4));

//decoder2to4 decoder2to4(
//.in(count4),
//.out(CS));
assign CS[0]=!count4[0]&!count4[1];
assign CS[1]=count4[0]&!count4[1];
assign CS[2]=!count4[0]&count4[1];
assign CS[3]=count4[0]&count4[1];

digital_clock digital_clock(
.clk(clk2),
.rst(rst),
.DATA0(DATA0),
.DATA1(DATA1),
.DATA2(DATA2),
.DATA3(DATA3));

//mux4to1_bit4 mux(
//.DATA0(DATA0),
//.DATA1(DATA1),
//.DATA2(DATA2),
//.DATA3(DATA3),
//.CS(count4),
//.clk(clk1),
//.DATA(DATA));
assign DATA[0]=CS[0]&DATA0[0]|CS[1]&DATA1[0]|CS[2]&DATA2[0]|CS[3]&DATA3[0];
assign DATA[1]=CS[0]&DATA0[1]|CS[1]&DATA1[1]|CS[2]&DATA2[1]|CS[3]&DATA3[1];
assign DATA[2]=CS[0]&DATA0[2]|CS[1]&DATA1[2]|CS[2]&DATA2[2]|CS[3]&DATA3[2];
assign DATA[3]=CS[0]&DATA0[3]|CS[1]&DATA1[3]|CS[2]&DATA2[3]|CS[3]&DATA3[3];

seq seq(
.DATA(DATA),
.clk(clk),
.SEQ(SEQ));

endmodule
module digital_clock(
input clk,rst,
output [3:0]DATA0,
output [3:0]DATA1,
output [3:0]DATA2,
output [3:0]DATA3);

wire RST;

assign RST=rst|(DATA3[1]&DATA2[2]);

counter #(.WIDTH(4),.M(10))CNT_ML(
.clk(clk),
.rst(RST),
.counter(DATA0));

counter #(.WIDTH(4),.M(6))CNT_MH(
.clk(~DATA0[3]),
.rst(RST),
.counter(DATA1));

counter #(.WIDTH(4),.M(10))CNT_HL(
.clk(~DATA1[2]),
.rst(RST),
.counter(DATA2));

counter #(.WIDTH(4),.M(3))CNT_HH(
.clk(~DATA2[3]),
.rst(RST),
.counter(DATA3));

endmodule

module mux4to1_bit4(
input [3:0]DATA0,
input [3:0]DATA1,
input [3:0]DATA2,
input [3:0]DATA3,
input [1:0]CS,
input clk,
output reg [3:0]DATA);

/*
assign DATA[0]=!CS[0]&!CS[1]&DATA0[0]+
CS[0]&!CS[1]&DATA1[0]+
!CS[0]&CS[1]&DATA2[0]+
CS[0]&CS[1]&DATA3[0];
assign DATA[1]=!CS[0]&!CS[1]&DATA0[1]+
CS[0]&!CS[1]&DATA1[1]+
!CS[0]&CS[1]&DATA2[1]+
CS[0]&CS[1]&DATA3[1];
assign DATA[2]=!CS[0]&!CS[1]&DATA0[2]+
CS[0]&!CS[1]&DATA1[2]+
!CS[0]&CS[1]&DATA2[2]+
CS[0]&CS[1]&DATA3[2];
assign DATA[3]=!CS[0]&!CS[1]&DATA0[3]+
CS[0]&!CS[1]&DATA1[3]+
!CS[0]&CS[1]&DATA2[3]+
CS[0]&CS[1]&DATA3[3];
*/

always@(posedge clk)
begin
  case(CS)
    2'b00:DATA<=DATA0;
    2'b01:DATA<=DATA1;
    2'b10:DATA<=DATA2;
    2'b11:DATA<=DATA3;
endcase
end
endmodule
module seq
  (input [3:0]DATA,
  input clk,
  output reg [6:0]SEQ);
  
  always @(posedge clk)
  begin
  case(DATA)
    4'h0: SEQ=7'h7e;
    4'h1: SEQ=7'h30;
    4'h2: SEQ=7'h6d;
    4'h3: SEQ=7'h79;
    4'h4: SEQ=7'h33;
    4'h5: SEQ=7'h5b;
    4'h6: SEQ=7'h5f;
    4'h7: SEQ=7'h70;
    4'h8: SEQ=7'h7f;
    4'h9: SEQ=7'h7b;
    4'ha: SEQ=7'h77;
    4'hb: SEQ=7'h1f;
    4'hc: SEQ=7'h0d;
    4'hd: SEQ=7'h3d;
    4'he: SEQ=7'h4f;
    4'hf: SEQ=7'h47;
endcase
end

endmodule
module counter
#(parameter WIDTH=17,
parameter M=100000)
(input clk,rst,
output reg [WIDTH-1:0]counter);

always@(posedge clk or posedge rst)
begin
        if(rst)
        counter <= 0;
        else if (counter == M-1) begin  
        counter <= 0;  
        end 
        else begin  
        counter <= counter + 1;  
        end  
end
endmodule 
module fre_div #(
parameter N=8,
WIDTH=3)
(input clk,rst,
output reg clk_out);

reg [WIDTH-1:0]counter;

always@(posedge clk or posedge rst)
begin
        if(rst)
        counter<=0;
        else if (counter == N-1) begin  
        counter <= 0;  
        end 
        else begin  
        counter <= counter + 1;  
        end  
end

always @(posedge clk or posedge rst) begin  
    if (rst) begin   
        clk_out <= 0;  
    end  
    else if (counter == N-1) begin  
        clk_out <= ~clk_out;  
    end  
end

endmodule  
module decoder2to4(
input [1:0]in,
output wire [3:0]out);

assign out[0]=!in[0]&!in[1];
assign out[1]=in[0]&!in[1];
assign out[2]=!in[0]&in[1];
assign out[3]=in[0]&in[1];

endmodule
