//Master Controller module for multicore-multithreaded smart PUB/SUB router
//Created By: Nishant Mathur & Amanpreet Kaur
//Date: 14 Apr 2017

//The required functionality for the module is:
//1. The module will receive PL done signal from ip PL1 and issue packets to dest PL2.
//2. Master controller will acess scubscriber details in memory which will tell it which TOPIC and PACKET FIFO combination to execute
//3. 2 types of FIFOs are used: a). TOPIC FIFOs store the publisher data b). PACKET FIFOs store empty packet containers
//4. Master controller shall access topic SPRING FIFOs directly. TOPIC FIFOs are used in 1 packet write multiple read configuration. PACKET FIFOs are used in 1 write 1 read configuration
//5. Dest PL2 will signal rdy status when its ready to accept packets. 
//6. If packets are routed through FIFO 0, they are just let through wihout any modifications.

module master_ctrl	#(
				parameter ADDR_WIDTH = 8,										//addr width of fifo
				parameter DATA_WIDTH = 64,										//width of data stored in fifo
				parameter CTRL_WIDTH = DATA_WIDTH/8,							//width of control data
				parameter Number_of_FIFO = 5							//number of FIFO's we have after pipeline 1
			)
			(	
				//interface signals from ip PL
				input 			[log2(Number_of_FIFO)-1:0]	ip_fifo_sel,
				input 										ip_PL_done,
				
				//interface with dest PL
				
				//Queue signals 						
				output 	reg									in_rdy,
				input 										in_wr,				//checking for in-wr condition
				input 										out_rdy,
				output 	reg									out_wr,
				
				//master controller status
				output reg 		[log2(Number_of_FIFO)-1:0]	pckt_count,			//Its assumed that every FIFO can store 1 packet	
				output reg 		[1:0] 						state,
				output reg 									full,
				output reg 									nearly_full,
				output reg 									nearly_empty,
				output reg 									empty,
				output reg 									pckt_rd_wr,
				
				//data signals
				input 			[DATA_WIDTH-1:0] 			fifo_data_in,
				output reg 		[DATA_WIDTH-1:0] 			fifo_data_out,
				input 			[CTRL_WIDTH-1:0] 			fifo_ctrl_in,
				output reg 		[CTRL_WIDTH-1:0] 			fifo_ctrl_out,
				
				//misc
				input 		clk,
				input 		rst
			);			
// Spring FIFO
// control signals			
reg [Number_of_FIFO-1:0] sf_rd_req;			
reg [Number_of_FIFO-1:0] sf_wr_req;			
wire [Number_of_FIFO-1:0] sf_rd_ack;			
wire [Number_of_FIFO-1:0] sf_wr_ack;			
wire [Number_of_FIFO-1:0] sf_rd_done;			
wire [Number_of_FIFO-1:0] sf_wr_done;			

wire [Number_of_FIFO-1:0] sf_out_wr;			
wire [Number_of_FIFO-1:0] sf_in_rdy;			
reg [Number_of_FIFO-1:0] sf_in_wr;			
reg [Number_of_FIFO-1:0] sf_out_rdy;

//status
wire [Number_of_FIFO-1:0] sf_full;
wire [Number_of_FIFO-1:0] sf_nearly_full;
wire [Number_of_FIFO-1:0] sf_nearly_empty;
wire [Number_of_FIFO-1:0] sf_empty;
wire [Number_of_FIFO-1:0] sf_pckt_rd_wr;

//reg sf_rd_count [Number_of_FIFO-1:0];	

//Data signals
wire [DATA_WIDTH-1:0] sf_data_out[Number_of_FIFO-1:0];
wire [CTRL_WIDTH-1:0] sf_ctrl_out[Number_of_FIFO-1:0];

// State m/c signals
reg [log2(Number_of_FIFO)-1:0]	fifo_sel;
reg [log2(Number_of_FIFO)-1:0] op_fifo_sel;
reg rd_req;
reg wr_req;
reg rd_ack;
reg wr_ack;
reg rd_done;
reg wr_done;
reg [15:0] counter,counter_next;				//watchdog counter	

//misc control signals
reg [log2(Number_of_FIFO)-1:0]	prev_pckt_count, prev_pckt_count_next;	

//  `LOG2_FUNC
  function integer log2;
      input integer number;
      begin
         log2=0;
         while(2**log2<number) begin
            log2=log2+1;
         end
      end
   endfunction // log2		

genvar i;
generate
	for (i=0 ; i<Number_of_FIFO ; i = i+1)
	begin:SF
	spring_fifo	fifo(//control signals
					.rd_req(sf_rd_req[i]),
					.wr_req(sf_wr_req[i]),
					.out_rdy(sf_out_rdy[i]), 
					.in_wr(sf_in_wr[i]),
					.ip_rd_count(2'b1),				
					.out_wr(sf_out_wr[i]),
					.pckt_rd_wr(sf_pckt_rd_wr[i]),
					.rd_ack(sf_rd_ack[i]),
					.wr_ack(sf_wr_ack[i]),
					.rd_done(sf_rd_done[i]),
					.wr_done(sf_wr_done[i]),
					
					//fifo status regs
					.in_rdy(sf_in_rdy[i]),					//signal to decide if the fifo is ready to receive data or not
					.full(sf_full[i]), 						//only status... not necessary to be used
					.empty(sf_empty[i]), 						//only status... not necessary to be used
					.nearly_full(sf_nearly_full[i]), 				//only status... not necessary to be used
					.nearly_empty(sf_nearly_empty[i]),				//only status... not necessary to be used
					.packet_count(),
					.first_word(),
					.last_word(),
					.fifo_busy(),
					
					// data 
					//FIFO mode data from & to UDP
					.fifo_data_in(fifo_data_in),
					.fifo_data_out(sf_data_out[i]),
					.fifo_ctrl_in(fifo_ctrl_in),
					.fifo_ctrl_out(sf_ctrl_out[i]),
					.DEPTH(),
					.wr_state(), 					//added for testing
					.state(),	
					
					//for testing
					.fifo_wa(),
					.fifo_ra(),
					.fifo_rd_en(),
					.fifo_wr_en(),
					
					//misc
					.clk(clk),
					.rst(rst)
					);
	end
endgenerate



always @ (*)
begin

	sf_rd_req [0] = 'h0;
	sf_wr_req [0] = 'h0;	
	sf_rd_req [1] = 'h0;
	sf_wr_req [1] = 'h0;	
	sf_rd_req [2] = 'h0;
	sf_wr_req [2] = 'h0;	
	sf_rd_req [3] = 'h0;
	sf_wr_req [3] = 'h0;	
	sf_rd_req [4] = 'h0;
	sf_wr_req [4] = 'h0;	
	sf_in_wr [0] = 'h0; 		
	sf_out_rdy [0] = 'h0; 
	sf_in_wr [1] = 'h0; 		
	sf_out_rdy [1] = 'h0; 
	sf_in_wr [2] = 'h0; 		
	sf_out_rdy [2] = 'h0; 
	sf_in_wr [3] = 'h0; 		
	sf_out_rdy [3] = 'h0; 
	sf_in_wr [4] = 'h0; 		
	sf_out_rdy [4] = 'h0; 
 		
	case (fifo_sel)						//Mux to route relevant control signals to and from relevant SF
	'h0:begin
		rd_ack = sf_rd_ack [0];
		wr_ack = sf_wr_ack [0];
		rd_done	= sf_rd_done [0];
		wr_done	= sf_wr_done [0];		
		sf_rd_req [0] = rd_req;
		sf_wr_req [0] = wr_req;		

		in_rdy = sf_in_rdy [0];
		out_wr = sf_out_wr [0];					
		sf_in_wr [0]	= in_wr; 		
		sf_out_rdy [0] = out_rdy;
		
		full = sf_full [0]; 
		nearly_full = sf_nearly_full [0]; 
		nearly_empty = sf_nearly_empty [0]; 
		empty = sf_empty [0]; 
		pckt_rd_wr = sf_pckt_rd_wr[0];
		
		// fifo_data_out = sf_data_out[0];
		// fifo_ctrl_out = sf_ctrl_out[0];
	end

	'h1:begin
		rd_ack = sf_rd_ack [1];
		wr_ack = sf_wr_ack [1];
		rd_done	= sf_rd_done [1];
		wr_done	= sf_wr_done [1];		
		sf_rd_req [1] = rd_req;
		sf_wr_req [1] = wr_req;		

		in_rdy = sf_in_rdy [1];
		out_wr = sf_out_wr [1];					
		sf_in_wr [1]	= in_wr; 		
		sf_out_rdy [1] = out_rdy; 
		
		full = sf_full [1]; 
		nearly_full = sf_nearly_full [1]; 
		nearly_empty = sf_nearly_empty [1]; 
		empty = sf_empty [1]; 
		pckt_rd_wr = sf_pckt_rd_wr[1];
		
		// fifo_data_out = sf_data_out[1];
		// fifo_ctrl_out = sf_ctrl_out[1];
	end

	'h2:begin
		rd_ack = sf_rd_ack [2];
		wr_ack = sf_wr_ack [2];
		rd_done	= sf_rd_done [2];
		wr_done	= sf_wr_done [2];		
		sf_rd_req [2] = rd_req;
		sf_wr_req [2] = wr_req;		

		in_rdy = sf_in_rdy [2];
		out_wr = sf_out_wr [2];					
		sf_in_wr [2]	= in_wr; 		
		sf_out_rdy [2] = out_rdy; 
		
		full = sf_full [2]; 
		nearly_full = sf_nearly_full [2]; 
		nearly_empty = sf_nearly_empty [2]; 
		empty = sf_empty [2]; 
		pckt_rd_wr = sf_pckt_rd_wr[2];
	
		// fifo_data_out = sf_data_out[2];
		// fifo_ctrl_out = sf_ctrl_out[2];
	end

	'h3:begin
		rd_ack = sf_rd_ack [3];
		wr_ack = sf_wr_ack [3];
		rd_done	= sf_rd_done [3];
		wr_done	= sf_wr_done [3];		
		sf_rd_req [3] = rd_req;
		sf_wr_req [3] = wr_req;		

		in_rdy = sf_in_rdy [3];
		out_wr = sf_out_wr [3];					
		sf_in_wr [3]	= in_wr; 		
		sf_out_rdy [3] = out_rdy; 
		
		full = sf_full [3]; 
		nearly_full = sf_nearly_full [3]; 
		nearly_empty = sf_nearly_empty [3]; 
		empty = sf_empty [3]; 
		pckt_rd_wr = sf_pckt_rd_wr[3];
		
		// fifo_data_out = sf_data_out[3];
		// fifo_ctrl_out = sf_ctrl_out[3];
	end

	'h4:begin
		rd_ack = sf_rd_ack [4];
		wr_ack = sf_wr_ack [4];
		rd_done	= sf_rd_done [4];
		wr_done	= sf_wr_done [4];		
		sf_rd_req [4] = rd_req;
		sf_wr_req [4] = wr_req;		

		in_rdy = sf_in_rdy [4];
		out_wr = sf_out_wr [4];					
		sf_in_wr [4]	= in_wr; 		
		sf_out_rdy [4] = out_rdy; 
		
		full = sf_full [4]; 
		nearly_full = sf_nearly_full [4]; 
		nearly_empty = sf_nearly_empty [4]; 
		empty = sf_empty [4]; 
		pckt_rd_wr = sf_pckt_rd_wr[4];
			
		// fifo_data_out = sf_data_out[4];
		// fifo_ctrl_out = sf_ctrl_out[4];
		end
		
	default: begin
		rd_ack = sf_rd_ack [0];
		wr_ack = sf_wr_ack [0];
		rd_done	= sf_rd_done [0];
		wr_done	= sf_wr_done [0];		
		sf_rd_req [0] = rd_req;
		sf_wr_req [0] = wr_req;		

		in_rdy = sf_in_rdy [0];
		out_wr = sf_out_wr [0];					
		sf_in_wr [0]	= in_wr; 		
		sf_out_rdy [0] = out_rdy; 
		
		full = sf_full [0]; 
		nearly_full = sf_nearly_full [0]; 
		nearly_empty = sf_nearly_empty [0]; 
		empty = sf_empty [0]; 
		pckt_rd_wr = sf_pckt_rd_wr[0];
	
		// fifo_data_out = sf_data_out[0];
		// fifo_ctrl_out = sf_ctrl_out[0];
		end
	endcase
 

end

always @ (fifo_sel, sf_data_out[0],sf_data_out[1],sf_data_out[2],sf_data_out[3],sf_data_out[4],sf_ctrl_out[0],sf_ctrl_out[1],sf_ctrl_out[2],sf_ctrl_out[3],sf_ctrl_out[4])	//separate always block due to xilinx synthesis issues. 
begin
 		
	case (fifo_sel)						//Mux to route relevant data to and from SF
	'h0:begin
		fifo_data_out = sf_data_out[0];
		fifo_ctrl_out = sf_ctrl_out[0];
	end

	'h1:begin
		
		fifo_data_out = sf_data_out[1];
		fifo_ctrl_out = sf_ctrl_out[1];
	end

	'h2:begin
		fifo_data_out = sf_data_out[2];
		fifo_ctrl_out = sf_ctrl_out[2];
	end

	'h3:begin
		fifo_data_out = sf_data_out[3];
		fifo_ctrl_out = sf_ctrl_out[3];
	end

	'h4:begin
		fifo_data_out = sf_data_out[4];
		fifo_ctrl_out = sf_ctrl_out[4];
		end
		
	default: begin
		fifo_data_out = sf_data_out[0];
		fifo_ctrl_out = sf_ctrl_out[0];
		end
	endcase
 

end



always @ *
begin
//store prev packt count to detect change completion of a read and write operation
prev_pckt_count_next = pckt_count;
counter_next = counter + 1;

end


always @ (posedge clk, posedge rst)
begin
	if (rst)
		begin
			state <= 'h0;
			rd_req <= 'h0;
			wr_req <= 'h0;
			fifo_sel <= 'h0;
			op_fifo_sel <= 'h0;
			pckt_count <= 'h0;
			prev_pckt_count <= 'h0;
			counter <= 'h0;
		end

	else
	begin
	
		//logic to select op_fifo	
	prev_pckt_count <= prev_pckt_count_next;
	//fifo_sel <= ip_fifo_sel;	
	counter <= counter_next;
	if (fifo_sel == 'h0)		//if current FIFO selected for ip is bypass FIFO, set the op to be selected from the same FIFO
		op_fifo_sel <= fifo_sel;
	else if (pckt_count == 'h4)
		op_fifo_sel <='h1;
	else if (pckt_count == 'h3)	
		op_fifo_sel <='h2;
	else if (pckt_count == 'h2)	
		op_fifo_sel <='h3;
	else if (pckt_count == 'h1)	
		op_fifo_sel <='h4;
		
	// else if (prev_pckt_count != pckt_count)		//change op FIFO once a packet is read out
		// begin
			// op_fifo_sel <= op_fifo_sel + 1;		// this selection can later be changed to get the selection from a subscriber memory and op_fifo_sel can be replaced with mem address
		//end	

		//state machine to read and write data to convert FIFOs
		if (counter < 'h10000)
		begin
			case (state)	//state m/c to control write and read operations to SF
			'h0: begin		//wait for select signal from ip PL and enable write operation on FIFO
					if (ip_PL_done)
						begin
							fifo_sel <= ip_fifo_sel;
							if (pckt_rd_wr)	
									wr_req <= 1;
							
							if (wr_ack)
								begin
									wr_req <= 0;
									state <= 'h1;
								end
						end
				end
			
			'h1: begin		//wait for write operation to complete
					if (!pckt_rd_wr)
						begin
							if (prev_pckt_count != pckt_count)
							begin
							
								if ((pckt_count == 'h4) | (fifo_sel == 'h0))	//proceed to read state if all packets received or current FIFO is bypass FIFO.
									state <= 'h2;
							
								else
									state <= 'h0;
							end
							else
								pckt_count <= pckt_count + 1;
						end
				end
			
			
			'h2: begin		//check and assign next FIFO to read from 
					fifo_sel <= op_fifo_sel;
					rd_req <= 1;
					
					if (rd_ack)
						begin
							rd_req <= 0;
							state <= 'h3;
						end
				end
			
			'h3: begin		//transfer required packet to dest PL and wait for transfer to complete
					if (pckt_rd_wr)
						begin
							if (prev_pckt_count != pckt_count)
								begin
								if ((pckt_count == 'h0) | (op_fifo_sel == 'h0))		//revert to write state if all packets read out or current op FIFO is bypass FIFO
								begin	
									state <= 'h0;
									counter_next <= 'h0;
								end
								else
									state <= 'h2;
								end
							else
							pckt_count <= pckt_count - 1;
						end
				end
			
			default:begin
						state <= 'h0;
						rd_req <= 'h0;
						wr_req <= 'h0;
						fifo_sel <= 'h0;
						op_fifo_sel <= 'h0;
						pckt_count <= 'h0;
					end
			endcase
		end
		else
		begin
			if(!sf_pckt_rd_wr[1])
					begin
						fifo_sel <= 'h1;
						wr_req <= 'h0;
						rd_req <= 'h1;
					end
				else if(!sf_pckt_rd_wr[2])
					begin
						fifo_sel <= 'h2;
						wr_req <= 'h0;
						rd_req <= 'h1;
					end	
				else if(!sf_pckt_rd_wr[3])
					begin
						fifo_sel <= 'h3;
						wr_req <= 'h0;
						rd_req <= 'h1;	
					end	
				else if(!sf_pckt_rd_wr[4])
					begin
						fifo_sel <= 'h4;
						wr_req <= 'h0;
						rd_req <= 'h1;	
						counter_next <= 'h0;
					end	
		end
		
		
	end
end
endmodule