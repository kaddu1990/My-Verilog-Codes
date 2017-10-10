`timescale 1ns / 1ps

module regfile (clk,
				rst,
				r0addr,
				r1addr,
				waddr,
				wdata,
				wena,
				r0data,
				r1data
			   );

input 			clk;
input 			rst;
input [4:0] 	r0addr;
input [4:0] 	r1addr;
input [4:0] 	waddr;
input [63:0] 	wdata;
input 			wena;
output [63:0]	r0data;
output [63:0]	r1data;

reg [63:0] reg_file [31:0];
reg [4:0] addr0,addr1,addr2;
reg [63:0] r0data,r1data;

always @ *
begin
addr0 = r0addr;
addr1 = r1addr;
addr2 = waddr;
end


always @ (posedge clk, posedge rst)
begin
	if (rst)
	begin
		reg_file[0] <= 'h0;
		reg_file[1] <= 'h0;
		reg_file[2] <= 'h0;
		reg_file[3] <= 'h0;
		reg_file[4] <= 'h0;
		reg_file[5] <= 'h0;
		reg_file[6] <= 'h0;
		reg_file[7] <= 'h0;
		reg_file[8] <= 'h0;
		reg_file[9] <= 'h0;
		reg_file[10] <= 'h0;
		reg_file[11] <= 'h0;
		reg_file[12] <= 'h0;
		reg_file[13] <= 'h0;
		reg_file[14] <= 'h0;
		reg_file[15] <= 'h0;
		reg_file[16] <= 'h0;
		reg_file[17] <= 'h0;
		reg_file[18] <= 'h0;
		reg_file[19] <= 'h0;
		reg_file[20] <= 'h0;
		reg_file[21] <= 'h0;
		reg_file[22] <= 'h0;
		reg_file[23] <= 'h0;
		reg_file[24] <= 'h0;
		reg_file[25] <= 'h0;
		reg_file[26] <= 'h0;
		reg_file[27] <= 'h0;
		reg_file[28] <= 'h0;
		reg_file[29] <= 'h0;
		reg_file[20] <= 'h0;
		reg_file[31] <= 'h0;
		
	end

	else
	begin
		r0data <= reg_file [addr0];
		r1data <= reg_file [addr1];
		
		if (wena)
			reg_file[addr2] <= wdata;
	end
end


endmodule

