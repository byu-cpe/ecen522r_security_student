module mul(clk_o, rst, out_reduced);
	input clk_o, rst;
	output [7:0] out_reduced;
	reg [4:0] cnt;
	wire clk;
	wire [63:0] z, n, x, zz;
	wire [7:0] e;
	wire e_round;
	
	assign n=64'hbe3a20ff7a7d7fca;
	assign x=64'hf01f2e724ac0ab35;
	assign e=8'b11110000;
	
	divi Ud(.clk(clk_o), .clkout(clk));
	
	always @(negedge clk) begin
		if(rst==1'b1) cnt=5'b00000;
		else cnt=cnt+5'b00001;
	end

	assign e_round=e[7-cnt[2:0]];

	sam_o U1 (.clk(clk), .z( (cnt[2:0]==3'b000)? 64'h0000000000000001:zz), .n(n), .x(x), .e(e_round), .zz(zz));

	assign out_reduced = {^zz[63:56], ^zz[55:48], ^zz[47:40], ^zz[39:32], ^zz[31:24], ^zz[23:16], ^zz[15:8], ^zz[7:0]};

endmodule
