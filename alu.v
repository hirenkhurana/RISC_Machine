module ALU(Ain,Bin,ALUop,out,Z);
    input [15:0] Ain, Bin;
    input [1:0] ALUop;
    output [15:0] out;
    output [2:0] Z;
    
    reg [15:0] out,out1;
    // reg [2:0] Z;

    always @* begin
        case (ALUop)
            2'b00 : begin 
                out = Ain + Bin;//Addition
                
                out1 = 16'b0;
                // Z = Z;
            end
            
            2'b01 : begin
                out = Ain - Bin;
                out1 = Ain + Bin;
                
            end //Subtraction
            
            2'b10 : begin
                
                out1 = 16'b0;
                out = Ain & Bin;//ANDing
                // Z = Z;
            end
        
            2'b11 : begin 
                
                out1 = 16'b0;
                out = ~Bin; //Not Bin
                // Z = Z;
            end
            
            default: begin 
                
                out1 = 16'bx;
                out = 16'bxxxxxxxxxxxxxxxx;
                
            end
        endcase
    end
    
    assign Z[0] = (out == 16'b0)? 1 : 0;//Z checks whether output is 16'b0
    assign Z[1] = (~(Ain[15]^Bin[15]) & (out1[15]^Ain[15]));//check overflow
    assign Z[2] = out[15];//checks negative

endmodule
