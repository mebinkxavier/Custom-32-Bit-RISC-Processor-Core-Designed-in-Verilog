module Testbench;
    reg clk;
    reg [31:0] A, B;
    reg [3:0] ALU_control;
    wire [31:0] Result;
    wire Zero;

    ALU alu_inst (
        .A(A),
        .B(B),
        .ALU_control(ALU_control),
        .Result(Result),
        .Zero(Zero)
    );

    initial begin
      	$dumpfile("dump.vcd");      
		$dumpvars(0, Testbench);    

        $display("Time\t A\t\t B\t\t Ctrl Result Zero");
        $monitor("%0d\t %h\t %h\t %b\t %h\t %b", $time, A, B, ALU_control, Result, Zero);

        // Test ADD
        A = 32'd10; B = 32'd5; ALU_control = 4'b0010; #10;

        // Test SUB
        A = 32'd15; B = 32'd15; ALU_control = 4'b0110; #10;

        // Test AND
        A = 32'hF0F0F0F0; B = 32'h0F0F0F0F; ALU_control = 4'b0000; #10;

        // Test OR
        A = 32'hF0000000; B = 32'h0000000F; ALU_control = 4'b0001; #10;

        // Test SLT (signed comparison)
        A = -5; B = 3; ALU_control = 4'b0111; #10;

        A = 10; B = 20; ALU_control = 4'b0111; #10;

        A = 20; B = 10; ALU_control = 4'b0111; #10;

        $finish;
    end
endmodule
