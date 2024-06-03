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
module CSA(cout,S,A,B,cin);
    output [3:0]S;
    output cout;
    input [3:0]A,B;
    input cin;
    wire c1,c2,c3,c4,P0,P1,P2,P3;
    full_adder F1(S[0],c1,A[0],B[0],CIN);
    full_adder F2(S[1],c2,A[1],B[1],c1);
    full_adder F3(S[2],c3,A[2],B[2],c2);
    full_adder F4(S[3],Cout,A[3],B[3],c3);
    and g0(p0,A[0],B[0]);
    and g1(p1,A[1],B[1]);
    and g2(p2,A[2],B[2]);
    and g3(p3,A[3],B[3]);
 mux2x1 m1(cout1,c3,cout,se1);
 endmodule;
module full_adder(co,S,A,B,ci); 
input A,B,ci; 
output S,co; 
assign S = A^B^ci; 
assign co = (A&B)|(B&ci)|(ci&A); 
endmodule


module CSA_tb;
reg [3:0]A1,B1; 
reg cin1; 
wire [3:0]S1; 
wire cout1;
CSA utt1(cout1,S1,A1,B1,cin1); 
initial begin 
A1=4'b0001; B1=4'b0010; cin1=1'b1; #2 
A1=4'b1111; B1=4'b1111; cin1=1'b1; #2 
A1=4'b1001; B1=4'b1010; cin1=1'b1; #2 
A1=4'b0001; B1=4'b0010; cin1=1'b0; #2 
$finish; 
end 
endmodule 
                                                          				(or)

module csa(a,b,cin,s,cout); 
input [3:0]a,b; 
input cin; 
output [3:0]s; 
output cout; 
wire [3:1]c; 
wire [3:0]p;
wire sel;


FAA FA0 (a[0],b[0],cin,s[0],c[1]); 
FAA FA1 (a[1],b[1],c[1],s[1],c[2]); 
FAA FA2 (a[2],b[2],c[2],s[2],c[3]); 
FAA FA3 (a[3],b[3],c[3],s[3],cout); 

assign p[0]=a[0]^b[0];
assign p[1]=a[1]^b[1];
assign p[2]=a[2]^b[2];
assign p[3]=a[3]^b[3];

always @(*)
case(sel)
1:cout=cin;
0:cout=c[3];
endcase 
endmodule 

module FAA (a,b,cin,s,cout); 
input a,b,ci; 
output s,co; 
wire vdd, gnd; 
assign s = a^b^ci; 
assign co = (a&b)|(b&ci)|(ci&a); 
endmodule

module csa_tb;
reg [3:0]a,b; 
reg cin; 
wire [3:0]s; 
wire cout; 
csa PA1 (a,b,cin,s,cout); 
initial begin 
a=4'b0001; b=4'b0010; cin=1'b1; #2 
a=4'b1111; b=4'b1111; cin=1'b1; #2 
a=4'b1001; b=4'b1010; cin=1'b1; #2 
a=4'b0001; b=4'b0010; cin=1'b0; #2 
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
           
