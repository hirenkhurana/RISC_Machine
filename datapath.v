module datapath(PC, sximm8, sximm5, mdata, vsel, clk, write, writenum, readnum,
                loada, asel, loadb, shift, bsel, ALUop, loadc, loads, Z_out, C);

    input wire clk, loada, loadb, loadc, loads, asel, bsel, write;
    input wire [8:0] PC;
    input wire [1:0] shift, ALUop;
    input wire [2:0] writenum, readnum;
    input wire [3:0] vsel;
    
    output wire [15:0] C;
    output wire [2:0] Z_out;
       
    wire [15:0] data_in, data_out, Ain, Bin, Aout, in, sout, out;
    wire [2:0] Z;

    input [15:0] sximm5, sximm8, mdata;

    // instantiating regfile module
    regfile REGFILE(data_in,
                writenum,
                write,
                readnum,
                clk,
                data_out);

    // instantiating ALU module
    ALU alu(Ain,
            Bin,
            ALUop,
            out,
            Z);

    // instantiating three regesters A,B,C
    pipeReg A_reg (loada, clk, data_out, Aout);
    pipeReg B_reg (loadb, clk, data_out, in);
    pipeReg C_reg (loadc, clk, out, C);

    // instantiating shifter module
    shifter U1 (in,
                shift,
                sout);

    // instantiating status module
    status state(loads, clk, Z, Z_out);

    // instantiating three 2 input multiplexers.
    mux4 Vselect (C, mdata, sximm8, {7'b0, PC}, data_in, vsel);
    mux2 Aselect (Aout , {16'b0}, Ain, asel);
    mux2 Bselect (sout , sximm5, Bin, bsel);
    
endmodule

// register module
module pipeReg(load,clk,in,out);
    input wire load, clk;
    input wire [15:0] in;
    output reg [15:0] out;

    initial begin
        out = 16'b0;
    end

    always @(posedge clk) begin
        // assigning out value depending on the status of load.
        // if load is 1, out is assigned in, else remains out.
        out = load ? in : out; 
    end
endmodule

module status(loads,clk,Z,Z_out);
    input wire loads, clk;
    input wire [2:0] Z;
    output reg [2:0] Z_out;

    initial begin
        Z_out = 3'b0;
    end

    always @(posedge clk) begin
        // assigning Z_out value depending on the status of loads.
        // if loads is 1, Z_out is assigned Z, else remains Z_out.
        Z_out = loads ? Z : Z_out; 
    end
endmodule

module mux4 (C, mdata, sximm8, PC, out, vsel);
    input [15:0] C, mdata, PC, sximm8;
    input [3:0] vsel;
    output [15:0] out;

    reg [15:0] out;

    always @* begin
        out = ({16{vsel[3]}} & mdata) | 
                ({16{vsel[2]}} & sximm8) |
                ({16{vsel[1]}} & PC) |
                ({16{vsel[0]}} & C) ; 
    end

endmodule

module mux2 (in_0, in_1, out, select);

    input [15:0] in_0, in_1;
    output [15:0] out;
    input select;
    
    // assigning out a value in_1 or in_0 depending on select.
    // if select is 1 out is assigned in_1, else in_0.
    assign out = select ? in_1 : in_0;
    
endmodule

