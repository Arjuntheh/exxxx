1. bcd.v
module syncbcd (clk, rst, q);  
input clk, rst;  
output [3:0]q;  
reg [3:0]q;  
initial q = 4'b0000;  
always @ (posedge clk)  
begin  
if (rst == 1'b1|q==4'b1001)  
q = 4'b0000;  
else   
q=q+1;  
end  
endmodule  

module syncbcd_tb;
reg clk, rst;
wire [3:0]q;
syncbcd utt(clk,rst,q);
initial begin  
clk=1'b0;
forever #1 clk=~clk;
end 
initial begin 
clk=1'b0;rst=1'b0;#100
$finish;
end
endmodule


2. cla.v
module CarryLookAheadAdder(
input [3:0]A, B, 
input Cin,
output [3:0] S,
output Cout
);
wire [3:0] Ci; 

assign Ci[0] = Cin;
assign Ci[1] = (A[0] & B[0]) | ((A[0]^B[0]) & Ci[0]);
 
assign Ci[2] = (A[1] & B[1]) | ((A[1]^B[1]) & ((A[0] & B[0]) | ((A[0]^B[0]) & Ci[0])));
 
assign Ci[3] = (A[2] & B[2]) | ((A[2]^B[2]) & ((A[1] & B[1]) | ((A[1]^B[1]) & ((A[0] & B[0]) | ((A[0]^B[0]) & Ci[0])))));

assign Cout  = (A[3] & B[3]) | ((A[3]^B[3]) & ((A[2] & B[2]) | ((A[2]^B[2]) & ((A[1] & B[1]) | ((A[1]^B[1]) & ((A[0] & B[0]) | ((A[0]^B[0]) & Ci[0])))))));

assign S = A^B^Ci;
endmodule

module TB;
reg [3:0]A, B; 
reg Cin;
wire [3:0] S;
wire Cout;

CarryLookAheadAdder utt(A, B, Cin, S, Cout);

initial begin

A = 1; B = 0; Cin = 0; #3;
A = 2; B = 4; Cin = 1; #3;
A = 4'hb; B = 4'h6; Cin = 0; #3;
A = 5; B = 3; Cin = 1;
end
endmodule


3. csa.v
module csa(a,b,cin,s,cout);
input [3:0]a,b;
input cin;
output [3:0]s;
output cout;
wire c1,c2,c3,c4;
wire [3:0]p;
wire sel;
FA fa1(a[0],b[0],cin,s[0],c1);
FA fa2(a[1],b[1],c1,s[1],c2);
FA fa3(a[2],b[2],c2,s[2],c3);
FA fa4(a[3],b[3],c3,s[3],c4);

assign p[0]=a[0]^b[0];
assign p[1]=a[1]^b[1];
assign p[2]=a[2]^b[2];
assign p[3]=a[3]^b[3];

assign sel=p[0]&p[1]&p[2]&p[3];
assign cout=sel? cin:c4;

endmodule

module FA(a,b,cin,s,cout);
input a,b,cin;
output s,cout;
assign s=a^b^cin;
assign cout = (a&b)|(b&cin)|(cin&a);
endmodule

module csa_tb;
reg [3:0]a,b;
reg cin;
wire[3:0]s;
wire cout;
csa csa1(.a(a),.b(b),.cin(cin),.s(s),.cout(cout));
initial begin
a=4'b0000;b=4'b0101;cin=1'b0;#10
a=4'b0101;b=4'b1010;cin=1'b1;#10
a=4'b1111;b=4'b0000;cin=1'b0;#10
$finish;
end
endmodule


4. fifo
module sync_fifo(input clk,
            input rst_n,
            input wr_en_i,
            input [7:0]data_i,
            output full_o,
            
            input rd_en_i,
            output reg [7:0] data_o,
            output empty_o);
    
    parameter DEPTH = 8; 
reg [7:0] mem[0:DEPTH-1];

reg[2:0] wr_ptr;
reg[2:0] rd_ptr;
reg[3:0] count;

assign full_o = (count == DEPTH);
assign empty_o = (count == 0);

////write process///
always @(posedge clk or negedge rst_n)
begin
if(!rst_n)
begin
wr_ptr <= 3'd0;
end else begin
if(wr_en_i == 1) begin
mem[wr_ptr] <= data_i;
wr_ptr <= wr_ptr + 1;
end
end
end

/////read process/////
always @(posedge clk or negedge rst_n)
begin
if(!rst_n)
begin
rd_ptr <= 3'd0;
end else begin
if(rd_en_i == 1)
begin
data_o = mem[rd_ptr];
rd_ptr <= rd_ptr + 1;
end
end
end
 

//////count//////
always @(posedge clk or negedge rst_n)
begin
if(!rst_n) begin
count <= 4'd0; 
 end else begin
case({wr_en_i,rd_en_i})
	2'b10: count <= count + 1;
	2'b01: count <= count - 1;
	2'b11: count <= count;
	2'b00: count <= count;
default: count <= count;
endcase
end
end
endmodule


`define clk_period 10

module sync_fifo_tb();

reg clk, rst_n;
reg wr_en_i, rd_en_i;
reg [7:0]data_i;

wire [7:0]data_o;
wire full_o, empty_o;
integer i;
sync_fifo SYNC_FIFO(.clk(clk),
            .rst_n(rst_n),
            .wr_en_i(wr_en_i),
            .data_i(data_i),
            .full_o(full_o),
            
            .rd_en_i(rd_en_i),
            .data_o(data_o),
            .empty_o(empty_o)
);

  initial clk = 1'b1;
always #(`clk_period/2) clk = ~clk;


initial begin
rst_n = 1'b1;
wr_en_i = 1'b0;
rd_en_i = 1'b0;
data_i = 8'b0;
#(`clk_period);
rst_n = 1'b0;
#(`clk_period);
rst_n = 1'b1;

  // write data//
wr_en_i = 1'b1;
rd_en_i = 1'b0;
 for(i=0;i<8;i=i+1)
begin
data_i = i;
#(`clk_period);
end

//read//
wr_en_i = 1'b0;
rd_en_i = 1'b1;
 for(i=0;i<8;i=i+1)
begin

#(`clk_period);
end

 // write data//
wr_en_i = 1'b1;
rd_en_i = 1'b0;
 for(i=0;i<8;i=i+1)
begin
data_i = i;
#(`clk_period);
end

#(`clk_period);
#(`clk_period);
#(`clk_period);

$finish;
end
  endmodule        



5.ms.v
module ms(clk,rst,j,k,q,qb);
input clk,rst,j,k;
output reg q,qb;
assign qb=~q;
reg mq;
always @(posedge(clk))
begin 
if(rst)
mq<=1'b0;
else 
begin 
case({j,k})
2'b00:mq<=mq;
2'b00:mq<=1'b0;
2'b00:mq<=1'b1;
2'b00:mq<=~mq;
endcase 
end 
end 
always @(negedge(clk))
begin 
if(rst)
q<=1'b0;
else 
q<=mq;
end 
endmodule 

module ms_tb; 
reg clk,rst,j,k; 
wire q,qb; 
ms utt1(clk,rst,j,k,q,qb); 
initial begin 
clk = 1'b0; 
forever #1 clk = ~clk; 
end 
initial begin 
rst = 1'b1; j = 1'b0; k = 1'b0 ; #10 
rst = 1'b0; j = 1'b0; k = 1'b1 ; #10 
rst = 1'b0; j = 1'b1; k = 1'b0 ; #10 

$finish; 
end 
endmodule 




6.melay
 
module  state_machine_mealy (clk,  reset,  in,  out);  
input  clk,  reset,  in;   
output  out; 
reg  out, state, next_state;  
parameter zero=0, one=1; 
//Implement  the  state  register  
always@(posedge clk, posedge reset) begin 
if (reset)  
state <= zero;  
else  
state <= next_state;  
End 
always  @(in or state)  
case  (state) 
zero:  begin 
//  last  input  was  a  zero 
out=0; 
if  (in)   
next_state  =  one; 
else  
next_state  =  zero;   
end 
one: begin //seen one 
if (in) begin 
next_state  =  one; 
out=1; 
end else begin 
next_state = zero; 
out=0; 
end 
end 
endcase 
endmodule 


Test bench 
`timescale 1ns/1ps 
`include "state_machine_mealy.v" 
module  state_machine_mealy_tb ;  
reg clk, reset, in; 
wire out;  
// instantiate state machine  
state_machine_mealy DUT (clk,  reset,  in,  out); 
initial  
forever #5 clk = ~clk;  
initial begin  
reset = 1'b1; 
clk = 1'b0; 
in = 0;  
#6; 
reset = 1'b0;  
for (integer i=0; i< 10; i=i+1) begin  
@(negedge clk); #1; 
in = $random; 
if (out == 1'b1) 
$display ("PASS : Sequence 11 detected i=%d\n“, i); 
end  
#50; 
$finish; 
end 
