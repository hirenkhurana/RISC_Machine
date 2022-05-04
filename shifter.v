module shifter(in,shift,sout);
    input [15:0] in;
    input [1:0] shift;
    output [15:0] sout;
    
    reg [15:0] sout;

    always @* begin
        case(shift)
            2'b00 : sout = in;//no shift in input
            2'b01 : sout = in << 1;//input is shifted to the left by 1 bit. LSB is 0.
            2'b10 : sout = in >> 1;//input is shifted to the right by 1 bit. MSB is 0.
            2'b11 : begin //input is shifted to the right by 1 bit. MSB is copy of previous.
                sout = in >> 1;
                sout[15] = sout[14];
            end
            default : sout = 16'bxxxxxxxxxxxxxxxx;
        endcase
    end
endmodule