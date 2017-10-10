//Amanpreet March 11,2017


`timescale 1ns / 100ps

module fifomem #(
				parameter ADR_WIDTH = 3,									//max fifo depth
				parameter DATA_WIDTH = 64,										//width of data stored in fifo
				parameter CTRL_WIDTH = DATA_WIDTH/8	,							//width of control data
				parameter WIDTH = 72,
				parameter PROG_FULL_THRESHOLD = 2**ADR_WIDTH - 1
				)
						
				(//FIFO control signals 
				input fiforead,
				input fifowrite,
				
				//output control signals
				output full,
				output nearly_full,
				output nearly_empty,
				output empty,
				
				//FIFO mode data to and from UDP
				input [WIDTH-1:0] in_fifo,
				output reg [WIDTH-1:0] out_fifo,
				
				//misc
				input clk,
				input rst);

				
parameter MAX_DEPTH = 2 ** ADR_WIDTH;
reg [ADR_WIDTH-1:0] WA;
reg [ADR_WIDTH-1:0] RA;

reg [WIDTH-1:0] queue [MAX_DEPTH-1:0];
reg [ADR_WIDTH:0] depth;

//assign dout = queue[rd_ptr];
assign full = depth == MAX_DEPTH;
assign nearly_full = depth >= MAX_DEPTH-1;
assign empty = depth == 'h0;
assign nearly_empty = depth == 'h1;

assign in_rdy = (!full | !nearly_full) & fifowrite;
	
	
always @(posedge clk, posedge rst)
begin
	if (rst)
		begin
			WA <= 'h0;
			RA <= 'h0;
			depth <= 'h0;
	end
	else begin
		if (fifowrite & ~full) WA <= WA + 'h1;
		if (fiforead & ~(empty)) RA <= RA + 'h1;
		if ((fifowrite & ~full) & ~(fiforead & ~empty)) depth <= 
					depth + 'h1;
		else if (~(fifowrite & ~full) & (fiforead & ~empty)) depth <=
					depth - 'h1;


	end
end

always @(posedge clk)
begin

	out_fifo <= queue[RA];
	if (fifowrite & ~full)	queue[WA] <= in_fifo;
end

endmodule





`timescale 1ns / 100ps
module fifomem_tb(); 

//misc
reg clk, rst;
integer count;

//fifo control signals
reg fiforead, fifowrite;
reg [71:0] in_fifo;
wire [71:0] out_fifo;

wire full;
wire nearly_full;
wire nearly_empty;
wire empty;

fifomem UUT (//FIFO control signals 
				.fiforead(fiforead),
				.fifowrite(fifowrite),
//				output reg valid_data,
				
				//output control signals
				.full(full),
				.nearly_full(nearly_full),
				.nearly_empty(nearly_empty),
				.empty(empty),
				
				//FIFO mode data to and from UDP
				.in_fifo(in_fifo),
				.out_fifo(out_fifo),
				
				//misc
				.clk(clk),
				.rst(rst)
				);
initial begin
fiforead = 'h0;
fifowrite = 'h0;
in_fifo = 'h0;
count = 0;
clk = 0;
rst = 1;
#10
rst = 0;

forever #10 clk = ~clk; 
end

always @(posedge clk)
begin
		if (count < 1)				//1st word
			begin
				fiforead = 'h1;
				fifowrite = 'h1;
				in_fifo <= 'hff_9999_1010_0101_1010;
			end
		else if (count < 2)			//inject header
			begin
				in_fifo <= 'h00_0101_1010_0101_1010;			
			end
		else if (count < 4)			//inject data body
			begin
				fiforead <= 'h0;
				in_fifo <= 'h00_ABCD_AAAA_CAFE_0000;	
			end
		else if (count < 6)			//last word
			begin
				in_fifo <= 'hff_0101_1010_0101_9999;			
			end
		else if (count < 11)			//1st word
			begin
				fiforead <= 'h1;
				fifowrite <= 'h0;
				in_fifo <= 'hff_9999_1010_0101_1010;
			end
		else if (count < 12)			//inject header
			begin
				in_fifo <= 'h00_0101_1010_0101_1010;			
			end
		else if (count < 13)		//inject data body
			in_fifo <= 'h00_ABCD_AAAA_CAFE_0000;				
		else if (count < 14)		//last word
			begin
				in_fifo <= 'hff_0101_1010_0101_9999;
			end
		else if (count < 15)		//last word
			begin
				fiforead <= 'h0;
				fifowrite <= 'h1;
				in_fifo <= 'hff_0101_1010_0101_9999;
			end
		
count <= count + 1;
end

endmodule
