//Written by: Nishant Mathur
//Date: 16 Apr 2017

//This FIFO is similar to convert FIFO except that it can read out the same packet multiple times. Also there are no input muxes for the data entering the FIFO ie. there is only 1 data source.

//Notes on this modules:
// 	1. The wr_state machine is able to recognize 1st and last words of any packet received. The address locations of these elements are stored in a register which can be accessed by outside elements(eg pipeline).
//	2. Since, this module is integrated along with mem_mod_mast in UDP, we are receiving 72 bits of data (8 bits control + 64 bit data).
//	3. fifo read and fifo write signals shall be enabled from mem_mod_master based on the conditions placed there.
//	4. Interface with UDP: fifo interacts with the UDP through mem_mod mast. Data coming from UDP passes through several input and output queues. The control signals of interest are rdy and wr. If the next module is ready to receive inputs, then enable its write and drain data. Similarly, receive data from upstream module if the FIFO is ready to receive data.
//	5. in_rdy is provided as a status to UDP to signal that fifo is ready to receive data from outside. in_rdy is set if fifo is not full and fifo_wr_en is set.
//	6. Empty, Nearly_empty, Nearly_full and full have been provided to mem_mod_mast for monitoring purposes only.
//	7. fifo_read and fifo_write are permission bits that signify that the FIFO is allowed to carry out those operations. Its not necessary that FIFO actually reads and writes when these signals are 1.
//	8. out_wr is used to signify the validity of data read from fifo. As soon as reading starts ie. ra of 1st bit to be read is loaded, out_wr goes to 1. It remains 1 till the last data from the last address is read from fifo.
//	9. pckt_rd_wr provides status to mem_mod_mast about the current read/write status of the packet
//	10. We need a state machine which can control read and writes to the spring FIFO. At a time only 1 operation can be carried out (either read or write). 
//	11. We shall have rd_req, wr_req to request for read and write operations. Thus, fifo_rd_en and fifo_wr_en shall be controlled from inhouse.
//	12. Once we receive rd_req, we will read out a packet multiple times reloading the FW into read address whenever a packet is read out. Thus, only when rd_count reduces to 0 ie. a pcket is read out for the last time, should pckt_rd_wr turn to 1. Once, the read operation is completed, rd_done signal shall be issued.
//	13. To reload the FW address into fifo_ra, a change in read count can be checked by comparing read_count against read_count_prev.
//	13. Similary, when we receive wr_req, we will write a packet into the FIFO. Completion of write operation can be checked based on pckt_rd_wr_next. Once, the write operation is completed, wr_done signal shall be issued.
//	14. The design assumes that we can multicast to 4 subscribers currently.



module spring_fifo #(	parameter MAX_DEPTH = 256,										//max fifo depth
						parameter ADDR_WIDTH = 8,										//addr width of fifo
						parameter DATA_WIDTH = 64,										//width of data stored in fifo
						parameter CTRL_WIDTH = DATA_WIDTH/8								//width of control data	
						)
						
						(//control signals
						input rd_req,
						input wr_req,
						input out_rdy, 
						input in_wr,
						input [1:0] ip_rd_count,				//number of times same packet needs to be read out. We can handle maximum 4 packets multicast.
						output reg out_wr,
						output reg pckt_rd_wr,
						output reg rd_ack,
						output reg wr_ack,
						output reg rd_done,
						output reg wr_done,
						
						//fifo status regs
						output in_rdy,					//signal to decide if the fifo is ready to receive data or not
						output full, 						//only status... not necessary to be used
						output empty, 						//only status... not necessary to be used
						output nearly_full, 				//only status... not necessary to be used
						output nearly_empty,				//only status... not necessary to be used
						output reg [1:0] packet_count,
						output reg [ADDR_WIDTH-1:0] first_word,
						output reg [ADDR_WIDTH-1:0] last_word,
						output fifo_busy,
						
						// data 
						//FIFO mode data
						input [DATA_WIDTH-1:0] fifo_data_in,
						output [DATA_WIDTH-1:0] fifo_data_out,
						input [CTRL_WIDTH-1:0] fifo_ctrl_in,
						output [CTRL_WIDTH-1:0] fifo_ctrl_out,
						output reg [ADDR_WIDTH:0] DEPTH,
						output reg [1:0] wr_state, 					//added for testing
						output reg [1:0] state,	
						
						//for testing
						output reg [ADDR_WIDTH-1:0] fifo_wa,
						output reg [ADDR_WIDTH-1:0] fifo_ra,
						output reg fifo_rd_en,
						output reg fifo_wr_en,
						
						//misc
						input clk,
						input rst
						);
						
//state m/c signals

//reg fifo_rd_en; 
//reg fifo_wr_en;
reg [1:0] rd_count, rd_count_next, rd_count_prev;

//wr_state m/c signals
reg [1:0] /* wr_state,  */wr_state_next;						//commneted for testing
reg [ADDR_WIDTH-1:0] first_word_next, last_word_next;
reg [1:0] packet_count_next;
reg [ADDR_WIDTH-1:0] packet_size, packet_size_next;

//FIFO mode signals and addr
reg [ADDR_WIDTH-1:0] /*fifo_wa,*/ fifo_wa_next;
reg [ADDR_WIDTH-1:0] /*fifo_ra,*/ fifo_ra_next, fifo_ra_prev;
wire fifo_read, fifo_write;
reg [ADDR_WIDTH:0] DEPTH_next;
reg [ADDR_WIDTH:0] DEPTH_prev, DEPTH_prev_next;


//internal signals
reg out_wr_next;
reg pckt_rd_wr_next;

spring_fifo_mem mem2(
				.clka(clk),
				.dina({fifo_ctrl_in,fifo_data_in}),
				.addra(fifo_wa),
				.wea(fifo_write),
				.clkb(clk),
				.addrb(fifo_ra),
				.doutb({fifo_ctrl_out,fifo_data_out})
				);

assign empty = (DEPTH == 'h0) ? 1'b1 : 1'b0;								//empty when depth = 0
assign nearly_empty = (DEPTH == 'h1) ? 1'b1 : 1'b0;							//nearly empty when depth = 1 ie. in the next cycle FIFO may empty out
assign full = (DEPTH == MAX_DEPTH) ? 1'b1 : 1'b0;							//full when depth = MAX_DEPTH
assign nearly_full = (DEPTH == MAX_DEPTH - 1) ? 1'b1 : 1'b0;				//nearly full when depth = MAX_DEPTH - 1 ie. in the next cycle FIFO may become full 
assign in_rdy = (!(nearly_full | full)) & fifo_wr_en;						//signal ready to receive input if the fifo is not full and mem_mod_mast has enabled fifo_wr_en to 1
assign fifo_read = (fifo_rd_en & out_rdy & !empty);							//enable fifo read when fifo read mode enabled , output module ready for inputs & fifo is not empty
assign fifo_write = (fifo_wr_en & in_wr & !full);							//enable fifo write when fifo write mode enabled, input module sets we to 1 & fifo is not full
assign fifo_busy = (fifo_read | fifo_write);								//check if fifo is busy performing read and write operation

always @ *
begin

//fifo read & write conditions 
	fifo_ra_next = fifo_ra;				//initialization to avoid latches
	fifo_wa_next = fifo_wa;
	DEPTH_next = DEPTH;
	DEPTH_prev_next = DEPTH_prev;
	first_word_next = first_word;
	last_word_next = last_word;
	packet_size_next = packet_size;
	pckt_rd_wr_next = pckt_rd_wr;
	packet_count_next = packet_count;
	wr_state_next = wr_state;
	out_wr_next = out_wr;
	rd_count_next = rd_count;
	
	
	if (fifo_read)						//if FIFO read mode is enabled, next module on output line is ready to receive inputs & fifo is not empty, set write en of that module to 1 and start draining FIFO
		begin
			if (rd_count != rd_count_prev)					//bring back read pointer to first word whenever rd_count decrements and fifo_read is enabled.
				begin
					fifo_ra_next = first_word;
					DEPTH_next = DEPTH_prev;
				end
			else if (fifo_ra != last_word)					//condition to ensure reading operation from last word to last word
				begin
					if (fifo_ra == first_word)				//condition to ensure that out write is enabled only when first word reading starts
					begin
						out_wr_next = 1'b1;					//signifies that the data coming out from FIFO is valid
						DEPTH_prev_next = DEPTH;			//store the depth value at first word
					end
					fifo_ra_next = fifo_ra + 1;
					DEPTH_next = DEPTH_next -1;
				end
			else if ((fifo_ctrl_out != 0) & out_wr)	
				begin
					packet_count_next = packet_count - 1;	//decrease packet count by 	1 once read operation is completed
					out_wr_next = 1'b0; 					//ensure that out write goes to 0 only after last word is read from fifo
					rd_count_next = rd_count - 1;			//decrease number of packets to be read by 1
					if (rd_count_next == 0)
						pckt_rd_wr_next = 1'b1;				//set the packet read status to 1 if all packets are read out
				end
			
		end
	else out_wr_next = 1'b0;
	
	if ((fifo_ra == fifo_ra_prev) & (out_wr))		// don't read same word twice.
		begin
		out_wr_next = 1'b0;
		end

	if (fifo_write)						//if FIFO write mode is enabled & previous module in UDP sets the FIFO write enable to 1, start filling FIFO
		begin
			
			//fifo wr_state m/c to detect packet size, 1st word and last word
			case (wr_state)
				2'h0: begin	//Start
						if(fifo_ctrl_in != 'h0)			//1st & last word of packet in UDP has ctrl !=0
							begin
								wr_state_next = 2'h1;
								first_word_next = fifo_wa;	
								packet_size_next = 'h1;
								fifo_wa_next = fifo_wa + 1;
								DEPTH_next = DEPTH_next +1;
							end
					  end
					  
				2'h1: begin	//Header			
						if(fifo_ctrl_in == 'h0)			//check to see that its indeed 2nd word of the packet
						begin
							packet_size_next = packet_size + 1;
							fifo_wa_next = fifo_wa + 1;
							DEPTH_next = DEPTH_next +1;
							if (packet_size == 'h5)	//remain in header wr_state for header part of packet (5 words: 1 in start and 4 in header)
								wr_state_next = 2'h2;
						end		
					  end
					  
				2'h2: begin	//Data	
						fifo_wa_next = fifo_wa + 1;
						DEPTH_next = DEPTH_next +1;
						packet_size_next = packet_size + 1;
						if(fifo_ctrl_in != 'h0)			//remain in data mode till last word
						begin
								wr_state_next = 2'h0;
								last_word_next = fifo_wa;
								pckt_rd_wr_next = 1'b0;	//set the packet read status to 0
								packet_count_next = packet_count + 1; //increase packet count when write operation is complete
						end						
					  end
					  
				default: begin
							wr_state_next = wr_state;
							packet_count_next = packet_count;
							pckt_rd_wr_next = pckt_rd_wr;
							packet_size_next = packet_size;
							first_word_next = first_word;
							last_word_next = last_word;
						end
			endcase
		end

end

always @ (posedge clk, posedge rst)
begin
	if (rst)							//reset
		begin
			fifo_wa <= 'h0;
			fifo_ra <= 'h0;
			DEPTH <= 'h0;
			DEPTH_prev <= 'h0;
			state <= 'h0;
			wr_state <= 'h0;
			packet_size <= 'h0;
			packet_count <= 'h0;
			first_word <= 'h0;
			last_word <= 'h0;
			out_wr <= 'h0;
			pckt_rd_wr <= 'h1;
			fifo_rd_en <= 'h0;
			fifo_wr_en <= 'h0;
			rd_count <= 'h0;
			rd_ack <= 0;
			wr_ack <= 0;
			rd_done <= 1;			//initialized to 1 to signify that FIFO is ready to accept inputs ie. last packet is read out
			wr_done <= 0;
		end
	else
		begin
			fifo_wa <= fifo_wa_next;
			fifo_ra_prev <= fifo_ra;
			fifo_ra <= fifo_ra_next;
			DEPTH <= DEPTH_next;
			DEPTH_prev <= DEPTH_prev_next;
			wr_state <= wr_state_next;
			packet_size <= packet_size_next;
			packet_count <= packet_count_next;
			first_word <= first_word_next;
			last_word <= last_word_next;
			out_wr <= out_wr_next;
			pckt_rd_wr <= pckt_rd_wr_next;
			rd_count <= rd_count_next;
			rd_count_prev <= rd_count;
		

	//state machine to control FIFO read and write operations	
	case (state)
		'h0:begin	//Check for read req & enable fifo read
				rd_done <= 'h0;
				state <= 'h1;
				if(rd_req)
					begin
						fifo_rd_en <= 'b1;
						rd_ack <= 'b1;
						state <= 'h2;
						rd_count <= ip_rd_count;
						rd_count_prev <= ip_rd_count;
					end
			end
			
		'h1:begin	//Check for write req & enable fifo write
				wr_done <= 'h0;
				state <= 'h0;
				if(wr_req)
					begin
						fifo_wr_en <= 'b1;
						wr_ack <= 'b1;
						state <= 'h3;
					end
				end
			
		'h2:begin	//Wait for Read to complete
				rd_ack <= 'b0;
				if(pckt_rd_wr_next)
					begin
						fifo_rd_en <= 'b0;
						state <= 'h0;
						rd_done <= 'h1;
					end
			end
	
		'h3:begin	//Wait for Write to complete
				wr_ack <= 'b0;
				if(!pckt_rd_wr_next)
					begin
						fifo_wr_en <= 'b0;
						state <= 'h0;
						wr_done <= 'h1;
					end			
			end
		default: begin
					fifo_wr_en <= 0;
					fifo_rd_en <= 0;
					rd_ack <= 0;
					wr_ack <= 0;
					rd_done <= 0;
					wr_done <= 0;
					state <= 'h0;
				end
		endcase
	end
end

endmodule







//spring fifo test bench
`timescale 1ns / 100ps
module spring_fifo_tb ();

//misc
reg clk, rst;
integer count;

//test bench input
reg [71:0] ip_reg [255:0];
reg [7:0] reg_ra;

//test bench op reg file outputs
reg[4:0] reg_wa;

//fifo control signals
reg rd_req,wr_req;
reg [1:0] ip_rd_count;
reg out_rdy, in_wr;
wire out_wr;
wire rd_ack, wr_ack;
wire rd_done, wr_done;

//fifo status reg
wire [1:0] packet_count;
wire in_rdy;
wire [7:0] first_word, last_word;

//fifo data
//input
wire [63:0] fifo_data_in;
wire [7:0] fifo_ctrl_in;

//output
wire [63:0] fifo_data_out;
wire [7:0] fifo_ctrl_out;

spring_fifo	UUT(//control signals
						.rd_req(rd_req),
						.wr_req(wr_req),
						.out_rdy(out_rdy), 
						.in_wr(in_wr),
						.ip_rd_count(ip_rd_count),				//number of times same packet needs to be read out. We can handle maximum 4 packets multicast.
						.out_wr(out_wr),
						.pckt_rd_wr(),
						.rd_ack(rd_ack),
						.wr_ack(wr_ack),
						.rd_done(rd_done),
						.wr_done(wr_done),
						
						//fifo status regs
						.in_rdy(in_rdy),					//signal to decide if the fifo is ready to receive data or not
						.full(), 						//only status... not necessary to be used
						.empty(), 						//only status... not necessary to be used
						.nearly_full(), 				//only status... not necessary to be used
						.nearly_empty(),				//only status... not necessary to be used
						.packet_count(packet_count),
						.first_word(first_word),
						.last_word(last_word),
						.fifo_busy(),
						
						// data 
						//FIFO mode data from & to UDP
						.fifo_data_in(fifo_data_in),
						.fifo_data_out(fifo_data_out),
						.fifo_ctrl_in(fifo_ctrl_in),
						.fifo_ctrl_out(fifo_ctrl_out),
						.DEPTH(),
						.wr_state(), 					//added for testing
						.state(),	
						
						//misc
						.clk(clk),
						.rst(rst)
						);

assign {fifo_ctrl_in,fifo_data_in} = ip_reg[reg_ra];						
						
regfile op_reg(	
				.clk(clk),
				.rst(rst),
				.r0addr(),
				.r1addr(),
				.waddr(reg_wa),
				.wdata(fifo_data_out),
				.wena(out_wr),
				.r0data(),
				.r1data()
			   );

						
initial begin		//reset
rst = 1;
#30 rst = 0;
end

initial begin		//clock gen
#100
clk = 0;
forever #10 clk = ~clk;
end

initial begin
// fifo_data_in = 'h0;
// fifo_ctrl_in = 'h0;
count = 0;
rd_req = 0;
wr_req = 1;
ip_rd_count = 4 ;
out_rdy = 1;		//outputs are ready to be received
in_wr = 0;			//inputs provided shall be written to FIFO
reg_wa = 0;
reg_ra = 0;
$readmemh ("ip_reg_init.txt",ip_reg);
end

always @ (posedge clk)
begin
//if (in_rdy)
//	begin
		// in_wr <= 1'b1;
		// if (count < 1)			//1st word
			// begin
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h9999_1010_0101_1010;
			// end
		// else if (count < 5)		//inject header
			// begin
				// wr_req <= 0;
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'h0101_1010_0101_1010;			
			// end
		// else if (count < 10)	//inject data body
				// fifo_data_in <= 'hABCD_AAAA_CAFE_0000;				
		// else if (count < 11)	//last word
			// begin
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h0101_1010_0101_9999;			
			// end
		// else if (count < 15)
			// begin
				// out_rdy <= 1;
			// end
		// else if (count < 16)
			// begin
				// rd_req <= 0;
			// end
		// else if (count < 21)			//1st word
			// begin
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h9999_1010_0101_1010;
			// end
		// else if (count < 25)		//inject header
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'h0101_1010_0101_1010;			
			// end
		// else if (count < 30)	//inject data body
			// fifo_data_in <= 'hABCD_AAAA_CAFE_0000;				
		// else if (count < 31)		//last word
			// begin
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h0101_1010_0101_9999;
			// end
		// else if (count == 40)
			// begin
				// count <= 0;
				// out_rdy <= 0;
			// end
//	end
		if (count < 2)	wr_req <= 1;
		else if (wr_ack)	wr_req <= 0;

		if (count > 5 )
		begin
			if (in_rdy && fifo_ctrl_in != 'hab)
				begin
					in_wr <= 1;
					if (in_wr)
						reg_ra <= reg_ra + 1;
				end
			else 
				begin
					reg_ra <= reg_ra;
					in_wr <= 0;
				end
		end
		
		if (wr_done)	rd_req <= 1;
		if (rd_ack)		rd_req <= 0;
		
		if(out_wr)
			reg_wa <= reg_wa +1;
		else 
			reg_wa <= reg_wa;
			


count <= count + 1;
end


endmodule


