1. bcd.v
module BCD_Counter(input clk, rst, output [3:0] Q);
  
  jkfflop first(1'b1 , 1'b1, clk ,rst, Q[0]);
  
  jkfflop second(~Q[3],~Q[3],Q[0],rst, Q[1]);
  
  jkfflop third(1'b1 , 1'b1 , Q[1] ,rst,Q[2]);

  jkfflop fourth(Q[1]&Q[2] , Q[3] , Q[0] ,rst, Q[3]);

endmodule

module jkfflop(input  J, K , clk ,rst, output reg Q);
  always @(negedge clk) begin
    if(rst)
      Q <= 1'b0;
    else begin
    case({J,K})
      2'b00 : Q <= Q   ;
      2'b01 : Q <= 1'b0;
      2'b10 : Q <= 1'b1;
      2'b11 : Q <= ~Q  ;
    endcase
  end
  end
endmodule

bcd_tb.v
module BCD_Counter_TB;
  reg CLK = 0, rst = 1;
  wire[3:0] Q;
  BCD_Counter UUT(CLK ,rst,Q);
initial begin
CLK=1'b0;
forever #1 CLK=~CLK;
end
initial begin
rst=1;#10
rst=0;
$finish;
end
endmodule

2. cla 
