module divi(clk, clkout);
	input clk;
	output reg clkout;
	reg [25:0] cnt;
	
	always @(posedge clk)
		begin
			cnt=cnt+1;
			if (cnt==50)
				begin clkout=~clkout; cnt=26'b0; end
		end
		
endmodule