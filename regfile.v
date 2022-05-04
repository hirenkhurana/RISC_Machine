// decodes binary input into one hot code output.
module dec (a, b);
    
    input [2:0] a;
    output [7:0] b;

    reg [7:0] b;

    always @(a)
        b = 1 << a;

endmodule

// mux8 - 8 input multiplexer 
module mux8 (R7, R6, R5, R4, R3, R2, R1, R0, readnum, data_out);

    input [15:0] R0, R1, R2, R3, R4, R5, R6, R7;
    input [2:0] readnum;
    output [15:0] data_out;
    
    wire [7:0] readnum_oh;
    reg [15:0] data_out;

    // deconging readnum 
    dec readnum_boh (readnum, readnum_oh);

    always @* begin
        // assigns a value to data_out depending on the one hot readnum and the value stored in Rs.
        data_out = ({16{readnum_oh[0]}} & R0) | 
                        ({16{readnum_oh[1]}} & R1) |
                        ({16{readnum_oh[2]}} & R2) |
                        ({16{readnum_oh[3]}} & R3) |
                        ({16{readnum_oh[4]}} & R4) | 
                        ({16{readnum_oh[5]}} & R5) |
                        ({16{readnum_oh[6]}} & R6) |
                        ({16{readnum_oh[7]}} & R7) ; 
    end

endmodule


module regfile(data_in,writenum,write,readnum,clk,data_out);
    
    input [15:0] data_in;
    input [2:0] writenum, readnum;
    input wire write, clk;
    output [15:0] data_out;

    wire [15:0] data_in, data_out;
    wire [7:0] writenum_oh;
    wire [2:0] writenum, readnum;
    wire load7, load6, load5, load4, load3, load2, load1, load0;
    reg [15:0] R0, R1, R2, R3, R4, R5, R6, R7;
    wire [15:0] next_out7, next_out6, next_out5, next_out4, next_out3, next_out2, next_out1, next_out0;
    
    //instantiating decoder
    dec writenum_boh (writenum, writenum_oh);

    //instantiating 8 input multiplexer
    mux8 DUT (R7, R6, R5, R4, R3, R2, R1, R0, readnum, data_out);

    initial begin
        R7 = 16'b0;
        R6 = 16'b0;
        R5 = 16'b0;
        R4 = 16'b0;
        R3 = 16'b0;
        R2 = 16'b0;
        R1 = 16'b0;
        R0 = 16'b0;
    end
    
    
    // assigning load1-load7 value by ANDing write and the 
    // corresponding bit of the writenum onehot code.
    assign load7 = write & writenum_oh[7];
    assign load6 = write & writenum_oh[6];
    assign load5 = write & writenum_oh[5];
    assign load4 = write & writenum_oh[4];
    assign load3 = write & writenum_oh[3];
    assign load2 = write & writenum_oh[2];
    assign load1 = write & writenum_oh[1];
    assign load0 = write & writenum_oh[0];

    //selecting nextoutput output between storage R0-R7          
    assign next_out7 = load7 ? data_in : R7;
    assign next_out6 = load6 ? data_in : R6;
    assign next_out5 = load5 ? data_in : R5;
    assign next_out4 = load4 ? data_in : R4;
    assign next_out3 = load3 ? data_in : R3;
    assign next_out2 = load2 ? data_in : R2;
    assign next_out1 = load1 ? data_in : R1;
    assign next_out0 = load0 ? data_in : R0;

    //Output at rising edge of clock
    always @(posedge clk) begin 
        R7 = next_out7;
        R6 = next_out6;
        R5 = next_out5;
        R4 = next_out4;
        R3 = next_out3;
        R2 = next_out2;
        R1 = next_out1;
        R0 = next_out0;
    end

endmodule