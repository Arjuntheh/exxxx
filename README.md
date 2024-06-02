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
