//Pipeline control and datapath
//Created by: Amanpreet Kaur, Nishant Mathur and Reetinder Kaur
//Date: 15-Apr-17

//This is modularization of multithreaded pipeline created in EE 533 uptil Lab 10. The pipeline modularized here-in was ealier part of Mem_mod_master module. All the comments and functionality are copied from mem_mod_mast.

//CPCI_NF2_DATA_WIDTH is the width of data that would be entered into geenric regs
//UDP_REG_SRC_WIDTH is the width of source address which is assigned when sending out requests

//R_W : 1 for read 0 for write
//I_D : o for IstMem and 1 for DataMem
//HW_RDY: 0 for ready to receive input from user and 1 for busy executing writing/reading memeory
//PL_EN: 1 for normal pipeline functioning and 0 for pipline disabled
//SW_CTRL: 1 for DM/FIFO read/write control under SW and 0 for HW. Active when PL_EN = 0. Must be used in tandem with fifo_rd_en and fifo_wr_en for full effect.
//state[1] bit is used to identify 	the current state of mod master. When the state m/c is in state 0, & when all the inputs from SW are passed, SW sets HW_reg to 1. The machine then changes state and proceeds to state 2. This is to signify that HW is not ready to accept any further inputs. When the m/c reaches State 2, then mem_mod_mast will wait for the SW to set HW reg to 0. M/c will then revert to state 0 and be ready to accept inputs from SW.

//Modifications for Lab 9:	1. IM and DM continuously give out the relevant PL or SW defined read and write addresses
//							2. SW_CTRL bit added to read or write data from SW to FIFO/DM. Active only when pipeline is diabled.

//The Data RAM in pipeline in Lab 8 now shall now be relaced with a convertible FIFO.
//The state machine would now have the following functionality:
//	1. The FIFO will be by default in receive mode
//	2. When the FIFO is done receiving data, mem_mod_mast state machine will enable pipeline and disable FIFO.
//	3. Since, eventually, data memory will be replaced by FIFO, read and write operations to DM mememory need to be revisited. Most probabably, those are not required anymore.
//	4. Thus, we have 2 state machines. 1 as part of FIFO module that detects 1st word and last word, sets bits in_rdy and out_rdy that signals whether its ready to receive data from outside or not. The other should be a part of mem_mod_master that controls when the pipeline & FIFO should be enabled. 
//	5. When the FIFO write is enabled, it will receive data and signal to mem_mod_mast that new data set is available for pipeline to process. Based, on certain preset criterion, the pipeline shall be enabled. Once the pipeline processing is complete, mem_mod_mast will be notified about the same. The FIFO read & write would then be enabled to drain the data from and receive new data into the FIFO. 
//	6. Eventually, this perpertual cycle will continue. We need to check, what will happen to incoming data, during the time pipeline processes the data stored in FIFO.
//	7. Variable packet sizes also needs to be dealt with. That can be done in the pipeline by creating an instruction that recognizes the first word location as the offset and shifts the address of all the load/store operations accordingly.
//	8. State machine in mem_mod_mast shall have the following functionality:
//		a). 2 modes are envisaged - FIFO mode & PL mode
//		b). FIFO mode - fifo_rd_en & fifo_wr_en shall be 1. Thus, FIFO shall continue to write and read data from UDP.
//		c). PL mode - fifo_wr_en shall be 1 & fifo_rd_en shall be 0. PL_EN shall also be 1. Thus, the PL shall carryout operations on the packet received. Once, the processing is complete, the PL shall be disabled and packet will be drained from FIFO by enabling fifo_rd_en. 
//	9. The modes can be checked based on PL_EN from user/SW. 
//	10. ......missing comments regarding pipeline and fifo MODs
//	11. ......missing comments regarding pipeline and fifo book keeping MODs
//	12. Checks added on changing state of the control machine to avoid state change in the middle of pipeline and FIFO operation.

//Important Notes: - Minimum gap of 2 cycles is required between packets for successful operation of convert FIFO

module pipeline #(
				parameter MAX_DEPTH = 256,										//max convertible fifo depth
				parameter ADDR_WIDTH = 8,										//addr width of convertible fifo
				parameter DATA_WIDTH = 64,										//width of data stored in fifo
				parameter CTRL_WIDTH = DATA_WIDTH/8,							//width of control data
				parameter Number_of_FIFO = 5									//width of control data
				)
				(
				//data inputs & outputs from & to UDP
				input 	[DATA_WIDTH-1:0]			in_data,
				input 	[CTRL_WIDTH-1:0]			in_ctrl,
				output 	[DATA_WIDTH-1:0]			out_data,
				output 	[CTRL_WIDTH-1:0]			out_ctrl,

				//control signals to and from UDP
				input 								out_rdy,
				input 								in_wr,
				output 								in_rdy,
				output 								out_wr,
				
				//control signals from SW reg
				input								IorD,
				input	[8:0]						Addr_in,
				input [DATA_WIDTH-1:0]				Data_In,
				
				//outputs to HW reg
				output	[8:0]						Addr_DM, 	//Read & Write addresses from Data Mem and Instruction mem
				output	[8:0]						Addr_IM,		
				output	[31:0]						Data_out_IM,	//Data out from Data Mem and Instruction mem
				output	[DATA_WIDTH-1:0]			Data_out_DM,
				output 								full,			//convertible fifo status
				output 								empty,			
				output								fifo_busy,			
				output 	[1:0]						packet_count,			
				output 	[ADDR_WIDTH-1:0]			FIFO_RD_addr,
				output 	[ADDR_WIDTH-1:0]			FIFO_WR_addr,
				output	[1:0] 						fifo_state,
				output	[ADDR_WIDTH:0]				DEPTH,
				output	reg							PL_EN,
				output	reg							SW_CTRL,
				output	reg							fifo_rd_en,
				output	reg							fifo_wr_en,
				
				output	[log2(Number_of_FIFO)-1:0]	op_fifo_sel,
				output 	reg							PL_done,
				
				//misc
				input clk,
				input rst
				);

				
//PL signals, addr and data				
//Inputs to pipeline

//Internal controls
reg PL_EN_next;
reg SW_CTRL_next;
reg R_W_EN,R_W_EN_next;

//outputs from pipeline
wire DM_wea;
wire [DATA_WIDTH-1:0] PL_Data_mem_dina, PL_MEM_dout;
wire [ADDR_WIDTH-1:0] PL_Addr_DM;		
reg PL_done_next;
wire PL_done_raw;
//reg PL_done;

//ALU signals & data
//Inputs to ALU from PL
wire [63:0] EX_ALU_in1,EX_ALU_in2;

//Output from ALU to PL
wire [63:0] EX_ALU_out;


//Control Module control signals
wire [3:0] ALUop,EX_ALUop;
wire RegWrite,MemWrite, MemtoReg, Jump, ble, PL_done_ctrl, FW_ld;


//FIFO/DM signals, addr and data
//Control
reg fifo_rd_en_next;
reg fifo_wr_en_next;

//status
wire [ADDR_WIDTH-1:0] first_word; 
	
//outputs
wire pckt_rd_wr_next;
reg pckt_rd_wr;

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

   
datapath dp		( 	//Misc signals
				 .clk(clk),
				 .rst(rst),

				//Pipeline control commands & status
				 .pipeline_en(PL_EN),
				 .sw_ctrl(SW_CTRL),
				 .IorD(IorD),			// 1 for Data Memory
				 .WorR(R_W_EN),				// 1 for Read
				 .MEM_PL_done(PL_done_raw),		//PL done output from datapath
				 
				 //Read and Write address and data from user
				 .Addr_in(Addr_in),
				 .Data_in(Data_In),
				 .IM_Addr_out(Addr_IM),
				 .DM_Addr_out(Addr_DM),
				 .IM_Data_out(Data_out_IM),
				 .DM_Data_out(Data_out_DM),
				 
				 //Signals to be integrated with PL
				 .Data_mem_wea (DM_wea),
				 .Data_mem_dina (PL_Data_mem_dina),
				 .Data_mem_addr (PL_Addr_DM), 
				 .MEM_Dout(PL_MEM_dout),
				 .first_word(first_word),
//				 .last_word(last_word),
				 
				 //Data to and from ALU
				 .EX_ALU_in1(EX_ALU_in1),
				 .EX_ALU_in2(EX_ALU_in2),
				 .EX_ALU_out(EX_ALU_out),
				 .EX_ALUop(EX_ALUop),
				 
				 //Control signals from control mod
				 .RegWrite(RegWrite),
				 .ALUSrc(ALUSrc),
				 .MemWrite(MemWrite),
				 .MemtoReg(MemtoReg),	
				 .Jump(Jump),	
				 .ble(ble),	
				 .imme(imme),
				 .ALUop(ALUop),
				 .PL_done(PL_done_ctrl),
				 .FW_ld (FW_ld)
				);

ALU_ctrl EX_ALU (
				.A(EX_ALU_in1),
				.B(EX_ALU_in2),
				.cmd(EX_ALUop),					
				.psw(),					//psw not used since behavioural coding is used for ALU now which is void of comparitors.
				.Data_out(EX_ALU_out)
				);
				
control ctrl	(
				.instruction(Data_out_IM),
				.RegWrite(RegWrite),
				.ALUSrc(ALUSrc),
				.MemWrite(MemWrite),
				.MemtoReg(MemtoReg),	
				.Jump(Jump),	
				.ble(ble),	
				.imme(imme), 
				.PL_done(PL_done_ctrl),
				.FW_ld (FW_ld),
				.ALUop(ALUop),
				.Master_reg(op_fifo_sel),
				.mem_sel()
				);				
				
convert_fifo fifo_dm 	(//control signals
				.fifo_rd_en(fifo_rd_en), 
				.fifo_wr_en(fifo_wr_en),
				.out_rdy(out_rdy), 
				.in_wr(in_wr),
				.out_wr(out_wr),
				.pckt_rd_wr(pckt_rd_wr_next),
				
				//fifo status regs
				.in_rdy(in_rdy),				//signal to decide if the fifo is ready to receive data or not
				.full(full),						//full, empty, nearly full, nearly empty provided just for status... not necessary to be used
				.empty(empty), 
				.nearly_full(), 
				.nearly_empty(),
				.packet_count(packet_count),
				.first_word(first_word),
				.last_word(),
				.fifo_busy(fifo_busy),
				
				// data 
				//FIFO mode data from & to UDP
				.fifo_data_in(in_data),
				.fifo_data_out(out_data),
				.fifo_ctrl_in(in_ctrl),
				.fifo_ctrl_out(out_ctrl),
				
				//DM mode data and addr from and to PL
				.PL_A_DM(PL_Addr_DM),
				.PL_Data_mem_dina(PL_Data_mem_dina),
				.DM_wea(DM_wea),
				.PL_MEM_dout(PL_MEM_dout),
				
				//status outputs
				.addrb(FIFO_RD_addr),
				.addra(FIFO_WR_addr),
				.state (fifo_state),
				.DEPTH (DEPTH),
								
				//misc
				.clk(clk),
				.rst(rst)
				);

				
always @ * 
	begin

		PL_EN_next = 'b0;					//Initialize to avoid latches
		SW_CTRL_next = 'b0;
		R_W_EN_next = 'b1;
		fifo_rd_en_next = 'b0;	
		fifo_wr_en_next = 'b0;
		PL_done_next = PL_done;
									
		if (PL_done_raw & !pckt_rd_wr_next)
			PL_done_next = 1;
		if (PL_done_raw & !pckt_rd_wr_next)
			PL_done_next = 0;
			
		if (PL_done & pckt_rd_wr)			//condition to check if the current slot is empty for writing
			begin
				fifo_wr_en_next = 'b1;
				PL_done_next = pckt_rd_wr_next;
			end

		if (!PL_done & !pckt_rd_wr)		//condition to check if the current slot is available for PL processing
			begin
				PL_EN_next = 'b1;				
				PL_done_next = PL_done_raw;
			end
			
		if (PL_done & !pckt_rd_wr)		//condition to check if the current slot is has processed packet for reading
			begin
				fifo_rd_en_next = 'b1;							
			end
	end

always @ (posedge clk, posedge rst)
	begin
		if (rst)
			begin
				PL_EN <= 0;
				SW_CTRL <= 0;
				R_W_EN <= 1;
				fifo_rd_en <= 0;
				fifo_wr_en <= 0;
				PL_done <= 'h1;
				pckt_rd_wr <= 'h1;
			end

		else
			begin
				PL_EN <= PL_EN_next;
				SW_CTRL <= SW_CTRL_next;
				R_W_EN <= R_W_EN_next;
				fifo_rd_en <= fifo_rd_en_next;
				fifo_wr_en <= fifo_wr_en_next;
				PL_done <= PL_done_next;
				pckt_rd_wr <= pckt_rd_wr_next;
			end
	end

endmodule


`timescale 1ns/100ps
module pipeline_tb();

	//misc
reg clk, rst;


//FIFO/DM signals
	//input
wire [63:0] fifo_data_in;
wire [7:0] fifo_ctrl_in;

	//outputs
wire [63:0] fifo_data_out;
wire [7:0] fifo_ctrl_out;

	//control
reg out_rdy, in_wr;
wire in_rdy, out_wr;
	
//controls
integer count,fq;
integer i, counter_restart;

//FIFO ip reg file and signals
reg [71:0] ip_reg [255:0];
reg [7:0] reg_ra;

//FIFO op reg file signals
reg[4:0] reg_wa;


pipeline PL 	(
				//data inputs & outputs from & to UDP
				.in_data(fifo_data_in),
				.in_ctrl(fifo_ctrl_in),
				.out_data(fifo_data_out),
				.out_ctrl(fifo_ctrl_out),

				//control signals to and from UDP
				.out_rdy(out_rdy),
				.in_wr(in_wr),
				.in_rdy(in_rdy),
				.out_wr(out_wr),
				
				//control signals from SW reg
				.IorD(),
				.Addr_in(),
				.Data_In(),
				
				//outputs to HW reg
				.Addr_DM(), 	//Read & Write addresses from Data Mem and Instruction mem
				.Addr_IM(),		
				.Data_out_IM(),	//Data out from Data Mem and Instruction mem
				.Data_out_DM(),
				.full(),			//convertible fifo status
				.empty(),			
				.fifo_busy(),			
				.packet_count(),			
				.FIFO_RD_addr(),
				.FIFO_WR_addr(),
				.fifo_state(),
				.DEPTH(),
				.PL_EN(),
				.SW_CTRL(),
				.fifo_rd_en(),
				.fifo_wr_en(),
				//misc
				.clk(clk),
				.rst(rst)
				);
				
			
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
			   
assign {fifo_ctrl_in,fifo_data_in} = ip_reg[reg_ra];

initial begin		//clk generator
	
	clk = 0;
	forever	#10	clk = ~clk;
end
	
initial begin		//reset 
	rst = 1;
	#30	rst = 0;

end
	
initial begin		//Initialize all the input registers to mem_mod_mast & setup outputs recording

	count = 0;
	out_rdy = 1;		//outputs are ready to be received
	in_wr = 0;			//inputs provided shall be written to FIFO
	counter_restart = 25;
	reg_wa = 0;
	reg_ra = 0;
	
	$readmemh ("ip_reg_init_var_pckt.txt", ip_reg);
end				

always @ (posedge clk)
	begin
		count <= count + 1;
		
		//take inputs from dummy reg file into fifo
		if (count > 2)
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
		
		//enable PL, disable output read operation
			
		// if (in_wr)
				// reg_ra <= reg_ra + 1;
		
		// if (in_rdy && (counter_restart == 0))
			// begin
				// in_wr <= 1;
				// counter_restart = 50;
			// end
		
		// if (fifo_ctrl_in == 'hab)
			// begin
				// in_wr <= 0;
			// end
		// counter_restart <= counter_restart - 1;
		
		else if (count == 200 )
			begin
				out_rdy <= 0;
				in_wr <= 0;
			end
		
		//take outputs from fifo into dummy reg file
		if(out_wr)
			reg_wa <= reg_wa +1;
		else 
			reg_wa <= reg_wa;
	end

endmodule