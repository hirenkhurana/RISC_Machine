`define MREAD 2'b10
`define MWRITE 2'b01
`define MNONE 2'b00

module RISCMACHINE(KEY,SW,LEDR,HEX0,HEX1,HEX2,HEX3,HEX4,HEX5,CLOCK_50);
    input [3:0] KEY;
    input CLOCK_50;
    input [9:0] SW;
    output [9:0] LEDR;
    output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;

    wire N,V,Z,w,switch_reader, switch_writer;
    reg [7:0] read_address, write_address ;
    reg write;
    reg [15:0] read_data;
    wire [15:0] din, dout, write_data;
    wire msel,reader,writer;
    reg triInp1,triInp2, load_led;
    wire [1:0] mem_cmd; 
    wire [8:0] mem_addr;

    //initialising cpu module
    cpu CPU(.clk(CLOCK_50),
            .reset(~KEY[1]),
            .s(~KEY[2]),
            .in(read_data),
            .out(write_data),
            .N(N),
            .V(V),
            .Z(Z),
            .w(w),
            .mem_cmd(mem_cmd),
            .mem_addr(mem_addr),
            .halter(LEDR[8])
            );

    
    //initialising RAM module
    RAM MEM(CLOCK_50,mem_addr[7:0],mem_addr[7:0],write,write_data,dout);

    // initalising load regesters.
    Reg #(8) L(load_led, CLOCK_50, write_data[7:0], LEDR[7:0]);

    //initialising Equality comparators 
    EqComp #(1) EQ1(mem_addr[8],1'b0,msel);
    EqComp #(2) EQ2(mem_cmd,`MREAD,reader);
    EqComp #(2) EQ3(mem_cmd,`MWRITE,writer);
    EqComp #(9) EQ4(mem_addr,9'h140,switch_reader);
    EqComp #(9) EQ5(mem_addr,9'h100,switch_writer);

    always @* begin //wiring for I/O and RAM

      triInp1 = msel & reader; //tristate buffer input
      triInp2 = reader & switch_reader;
      read_address = mem_addr[7:0];
      write_address = mem_addr[7:0];
      write = writer & msel;
      if(mem_addr !== 9'h140)
        read_data = triInp1 ? dout : 16'bz; 
      else begin
        read_data[7:0] = triInp2 ? SW[7:0] : 8'bz;
        read_data[15:8] = triInp2 ? 8'b0 : 8'bz;
      end 
      load_led = writer & switch_writer;
    end
endmodule

//RAM which stores the input values
module RAM(clk,read_address,write_address,write,din,dout);
  parameter data_width = 16; 
  parameter addr_width = 8;
  parameter filename = "data.txt";

  input clk;
  input [addr_width-1:0] read_address, write_address;
  input write;
  input [data_width-1:0] din;
  output [data_width-1:0] dout;
  reg [data_width-1:0] dout;

  reg [data_width-1:0] mem [2**addr_width-1:0];

  initial $readmemb(filename, mem);

  always @ (posedge clk) begin
    if (write)
      mem[write_address] <= din;
    dout <= mem[read_address]; // dout doesn't get din in this clock cycle 
                               // (this is due to Verilog non-blocking assignment "<=")
  end 
endmodule

//compares if the inputs are equal
module EqComp(a, b, eq) ;
  parameter k=8;
  input  [k-1:0] a,b;
  output eq;
  wire   eq;

  assign eq = (a==b)? 1:0 ;
endmodule

module Reg (load, clk, in, out); 
  parameter n = 8;
  input load, clk;
  input [n-1:0] in;
  output [n-1:0] out;

  reg [n-1:0] out;

  always @(posedge clk) begin // register to save the current value of output LED
    out = load ? in : out;
  end

endmodule