`define Reset 7'b0000001
`define IF1 7'b0000011
`define IF2 7'b0000111
`define UpdatePC 7'b0001111
`define HALT 7'b0011111
`define Decoder 7'b0000010
`define WriteImm 7'b0000100
`define GetA 7'b0001000
`define GetB 7'b0010000
`define ALUopr 7'b0100000
`define Loader 7'b0110000
`define RWM_LDR 7'b0111000
`define RWM_STR 7'b0111100
`define RWM_STR_VAL 7'b0111110
`define RWM_STR_VAL_S2 7'b0111111
`define WriteReg 7'b1000000
`define Branch 7'b1111111
`define Branch2 7'b1111110
`define Branch2_2 7'b1111100
`define Branch2_3 7'b1111000
`define Branch2_4 7'b1110000
`define Branch2_5 7'b1100000
`define MNONE 2'b00
`define MWRITE 2'b01
`define MREAD 2'b10


// declaration of module cpu 
module cpu(clk,reset,s,in,out,N,V,Z,w,mem_cmd,mem_addr,halter);
    input clk, reset,s;
    input [15:0] in;
    output [15:0] out;
    output N, V, Z, w, halter;

    wire  N, V, Z, s;

    reg loada, loadb, loadc, loads, asel, bsel, write, load_ir, load_pc, load_addr, reset_pc, addr_sel, w, halter;
    output [1:0] mem_cmd;
    output [8:0] mem_addr;
    reg [3:0] vsel;
    wire [1:0] shift, ALUop;
    wire [1:0] op;
    wire [2:0] writenum, readnum, Z_out;
    wire [2:0] opcode, Rn, Rd, Rm, cond;
    wire [15:0] in_ID;
    wire [15:0] C;
    wire [15:0] sximm8;
    wire [15:0] sximm5;
    wire [15:0] mdata;

    wire [8:0] mem_addr, next_pc;
    reg [1:0] mem_cmd;
    reg [8:0] PC;   
    wire [8:0] addr;   
    reg [2:0] nsel;
    reg [6:0] present_state, tmp;
    
    // 
    assign mdata = in;

    //instantiating module instReg
    instReg IR(clk, load_ir, in, in_ID);

    //instantiating module InstDecoder
    instDecoder ID(in_ID, nsel, opcode, op, ALUop, shift, sximm8, sximm5, readnum, writenum, cond);

    //instantiating module datapth
    datapath DP(PC, sximm8, sximm5, mdata, vsel, clk, write, writenum, readnum,
                loada, asel, loadb, shift, bsel, ALUop, loadc, loads, Z_out, C);

    wire [8:0] newPC;
    assign newPC = PC + 9'b000000001;  

    Reg #(9) DAD(load_addr, clk, C[8:0], addr);          

    mux2_9 RESETPC (newPC, 9'b0, next_pc, reset_pc);

    mux2_9 SELPC(addr, PC, mem_addr, addr_sel);
    
    //the FSM 
    always@(posedge clk) begin 

        PC = load_pc ? next_pc : PC;

        // resets the FSM
        if (reset) begin
            halter = 0;
            reset_pc = 1;
            load_pc = 1;
            present_state = `Reset;
            w = 1;
        end
        else begin
            //using casex satements to determine the present state
            casex({present_state,opcode,op,s})
                {`Reset,3'bxxx,2'bxx,1'b0}: present_state = `IF1;
                {`Reset,3'bxxx,2'bxx,1'b1}: present_state = `Reset;
                {`IF1,3'bxxx,2'bxx,1'bx}: present_state = `IF2;
                {`IF2,6'bxxxxxx}: present_state = `UpdatePC;
                {`UpdatePC,6'bxxxxxx}: present_state = `Decoder;
                {`Decoder,3'b110,2'b10,1'bx}: present_state = `WriteImm;
                {`Decoder,3'b110,2'bxx,1'bx}: present_state = `GetB;
                {`Decoder,3'b101,2'bxx,1'bx}: present_state = `GetA;
                {`Decoder,3'b011,2'bxx,1'bx}: present_state = `GetA;
                {`Decoder,3'b100,2'bxx,1'bx}: present_state = `GetA;
                {`Decoder,3'b111,2'bxx,1'bx}: present_state = `HALT;
                {`Decoder,3'b001,2'bxx,1'bx}: present_state = `Branch;
                {`Decoder,3'b010,2'bxx,1'bx}: present_state = `Branch2;
                {`Decoder,3'bxxx,2'bxx,1'bx}: present_state = `WriteReg;
                {`HALT,3'bxxx,2'bxx,1'bx}: present_state = `HALT;
                {`WriteImm,3'bxxx,2'bxx,1'bx}: present_state = `IF1;
                {`Reset,3'bxxx,2'bxx,1'bx}: present_state = `IF1;
                {`GetA,3'bxxx,2'bxx,1'bx}: present_state = `GetB;
                {`GetB,3'bxxx,2'bxx,1'bx}: present_state = `ALUopr;
                {`ALUopr,5'b01100,1'bx}: present_state = `Loader;
                {`ALUopr,5'b10000,1'bx}: present_state = `Loader;
                {`ALUopr,6'bxxxxxx}: present_state = `WriteReg;
                {`Loader,5'b01100,1'bx}: present_state = `RWM_LDR;
                {`Loader,5'b10000,1'bx}: present_state = `RWM_STR;
                {`RWM_LDR,6'bxxxxxx}: present_state = `WriteReg;
                {`RWM_STR,6'bxxxxxx}: present_state = `RWM_STR_VAL;
                {`RWM_STR_VAL,6'bxxxxxx}: present_state = `RWM_STR_VAL_S2;
                {`RWM_STR_VAL_S2,6'bxxxxxx}: present_state = `IF1;
                {`WriteReg,6'bxxxxxx}: present_state = `IF1;
                {`Branch,6'bxxxxxx}: present_state = `IF1;
                {`Branch2,6'bxxxxxx}: present_state = `Branch2_2;
                {`Branch2_2,3'bxxx,2'b11,1'bx}: present_state = `IF1;
                {`Branch2_2,6'bxxxxxx}: present_state = `Branch2_3;
                {`Branch2_3,6'bxxxxxx}: present_state = `Branch2_4;
                {`Branch2_4,3'bxxx,2'b00,1'bx}: present_state = `IF1;
                {`Branch2_4,6'bxxxxxx}: present_state = `Branch2_5;
                {`Branch2_5,6'bxxxxxx}: present_state = `IF1;
                default: present_state = 7'bx;
            endcase

            // changing output according to present state
            case (present_state)
                `HALT: halter = 1;

                `Reset: begin
                          halter = 0;
                          w=1;
                          load_addr = 0;
                          addr_sel = 0;
                          load_ir = 0;
                          reset_pc = 1;
                          load_pc = 1;
                          loada = 1'b0;
                          loadb = 1'b0;
                          loadc = 1'b0;
                          asel = 2'b0;
                          bsel = 2'b0;
                          nsel = 3'b0;
                          vsel = 4'b1000;
                          write = 1'b0;
                          mem_cmd = `MNONE;
                       end

                `IF1: begin 
                          w=1;
                          load_addr = 0;
                          addr_sel = 1;
                          load_ir = 0;
                          load_pc = 0;
                          loada = 1'b0;
                          loadb = 1'b0;
                          loadc = 1'b0;
                          asel = 2'b0;
                          bsel = 2'b0;
                          nsel = 3'b0;
                          vsel = 4'b1000;
                          write = 1'b0;
                          reset_pc = 0;
                          mem_cmd = `MREAD;
                      end
                    
                `IF2: begin
                    load_ir = 1;
                    addr_sel = 1;
                end

                `UpdatePC:  begin 
                    load_ir = 0;
                    load_pc = 1; 
                    mem_cmd = `MNONE; 
                end
                
                `Decoder: begin 
                    load_pc = 0;
                    write = 0;
                    w = 0; 
                end

                `WriteImm:  begin
                                nsel = 3'b100;
                                vsel =  4'b0100;
                                write = 1;
                            end
                
                `GetA: begin
                        write = 0;
                        loada = 1'b1;
                        nsel = 3'b100;
                       end
            
                `GetB: begin
                        nsel = 3'b001;
                        loadb = 1'b1;
                        loada = 1'b0;              
                end
                
                `ALUopr: begin 
                        if ({opcode, op} == 5'b11000) begin
                            loadb = 1'b0;
                            asel = 1'b1;
                            bsel = 1'b0;
                            loadc = 1'b1;
                            loads = 1'b1;
                        end
                        else if ({opcode,op} == 5'b01100) begin
                            loadb = 1'b0;
                            asel = 1'b0; 
                            bsel = 1'b1;  
                            loadc = 1'b1;
                            loads = 1'b1;  
                        end
                        else if ({opcode,op} == 5'b10000) begin 
                            asel = 1'b0; 
                            bsel = 1'b1; 
                            loadb = 1'b0;
                            loadc = 1'b1;
                            loads = 1'b1;
                        end

                        else begin
                            loadb = 1'b0;
                            asel = 1'b0;
                            bsel = 1'b0;
                            if (op == 01) begin
                                loadc = 1'b0;
                                loads = 1'b1;
                            end
                            else begin
                                loadc = 1'b1;
                                loads = 1'b0;
                            end
                        end    
                    end

                `Loader: begin
                    if ({opcode, op} == 5'b01100) begin
                            load_addr = 1'b1;
                            addr_sel = 1'b0;
                            loadc = 1'b0;
                            
                        end
                    else begin
                            load_addr = 1'b1;
                            addr_sel = 1'b0;                  
                            loadc = 1'b0;
                        end
                end

                `RWM_LDR: begin
                    mem_cmd = `MREAD;
                    load_addr = 1'b0;
                end

                `RWM_STR: begin
                    nsel = 3'b010;
                    loadb = 1;
                    load_addr = 0;
                end

                `RWM_STR_VAL: begin
                    loadb = 0;
                    asel = 1;
                    bsel = 0;
                    loadc = 1;
                end

                `RWM_STR_VAL_S2: begin
                    loadc = 0;
                    mem_cmd = `MWRITE;
                end
                
                `WriteReg: begin  
                              load_addr = 1'b0;
                              loadc = 1'b0;
                              loads = 1'b0;                           
                              
                              nsel = 3'b010;
                              if({opcode, op} == 5'b01100)
                                vsel = 4'b1000;
                              else 
                                vsel = 4'b0001;
                              if (op == 01) 
                                write = 0;
                              else
                                write = 1;
                                
                end

                `Branch: begin
                    if(cond == 3'b0) begin
                        PC = PC + sximm8;
                    end
                    else if(cond == 3'b001) begin
                        if(Z == 1'b1)
                            PC = PC + sximm8[8:0];
                        else
                            PC = PC;
                    end
                    else if(cond == 3'b010) begin
                        if(Z == 1'b0)
                            PC = PC + sximm8[8:0];
                        else
                            PC = PC;
                    end
                    else if(cond == 3'b011) begin
                        if(N !== V)
                            PC = PC + sximm8[8:0];
                        else
                            PC = PC;
                    end
                    else if(cond == 3'b100) begin
                        if(N !== V || Z == 1'b1)
                            PC = PC + sximm8[8:0];
                        else
                            PC = PC;
                    end
                    else 
                        PC =  9'bxxxxxxxxx;
                end      

                `Branch2: begin
                    if (op == 2'b11 | op == 2'b10) begin
                        vsel = 4'b0010;
                        nsel = 3'b100;
                        write = 1;
                    end 
                    else begin
                        nsel = 3'b010;
                        loadb = 1;
                        loada = 0;  
                    end  
                end     

                `Branch2_2: begin
                    if (op == 2'b11) begin
                        write = 0;
                        PC = PC + sximm8[8:0];
                    end 
                    else if (op == 2'b00) begin
                        asel = 1;
                        bsel = 0;
                        loadc = 1;
                    end
                    else begin
                        write = 0;
                        nsel = 3'b010;
                        loadb = 1;
                        loada = 0;
                    end
                end

                `Branch2_3: begin
                    if(op == 2'b10) begin
                        asel = 1;
                        bsel = 0;
                        loadc = 1;
                    end
                    else begin
                        loadc = 0;
                    end
                end

                `Branch2_4: begin
                    if (op == 2'b00) begin
                        PC = C[8:0];
                    end
                    else begin
                        loadc = 0;
                    end
                end

                `Branch2_5: begin
                    PC = C[8:0];
                end

                default: begin
                            nsel = 3'bxxx;
                            loada = 1'bx;
                            loadb = 1'bx;
                            loadc = 1'bx;
                            asel = 1'bx;
                            bsel = 1'bx;
                            write = 1'bx;
                            vsel = 4'bxxxx;
                            w= 1'bx;
                         end
            endcase
        end    
    
    end

    // assigning out, N, V, Z their respective values.
    assign out = C;
    assign N = Z_out[2];
    assign V = Z_out[1];
    assign Z = Z_out[0];

endmodule
                

module instReg (clk,load,in,out); //passes the new instruction value when load is set to 1
    input wire load, clk;
    input wire [15:0] in;
    output reg [15:0] out;

    initial begin
        out = 16'b0;
    end

    always @(posedge clk) begin
        out = load ? in : out; 
    end
endmodule

module instDecoder (in, nsel, opcode, op, ALUop, shift, sximm8, sximm5, readnum, writenum, cond); 
    // decodes value and assigns the particular values to their according bit sizes.

    input [15:0] in;
    input [2:0] nsel;
    output [2:0] opcode;
    output [1:0] op;
    output [1:0] ALUop;
    output [1:0] shift;
    output [15:0] sximm8;
    output [15:0] sximm5;
    output [2:0] readnum; 
    output [2:0] writenum;
    output [2:0] cond;

    reg [2:0] readnum;
    reg [2:0] writenum;

    wire [2:0] Rn;
    wire [2:0] Rd;
    wire [2:0] Rm;
    wire [4:0] imm5;
    wire [7:0] imm8;

    assign opcode = in[15:13];
    assign op = in[12:11];
    assign ALUop = {in[15:13],in[12:11]} == 5'b01010 ? 0 : in[12:11];
    assign Rn = in[10:8];
    assign Rd = in[7:5];
    assign Rm = in[2:0];
    assign shift = (in[15:13] == 3'b110 | in[15:13] == 3'b101) ? in[4:3] : 0;
    assign imm5 = in[4:0];
    assign imm8 = in[7:0];
    assign sximm8 = {{8{imm8[7]}} ,imm8};
    assign sximm5 = {{11{imm5[4]}},in[4:0]};
    assign cond = in[10:8];

    always @* begin //nsel selects between Rn, Rm and Rd
        case (nsel)
            3'b001: begin
                        readnum = Rm;
                        writenum = Rm;
                    end         
            3'b010: begin
                        readnum = Rd;
                        writenum = Rd;
                    end
            3'b100: begin
                        readnum = Rn;
                        writenum = Rn;
                    end
            default: begin
                readnum = 3'bxxx;
                writenum = 3'bxxx;
            end
        endcase
    end
    
endmodule

module mux2_9 (in_0, in_1, out, select);

    input [8:0] in_0, in_1;
    output [8:0] out;
    input select;
    
    // assigning out a value in_1 or in_0 depending on select.
    // if select is 1 out is assigned in_1, else in_0.
    assign out = select ? in_1 : in_0;
    
endmodule