//Memory module tester to be run on UDP
//Created by: Nishant Mohan Mathur & Amanpreet Kaur
//Date: 04-Mar-17
//File modified for testing Lab 8
//Further Modifications for Lab 9

//This module serves as the master control of a 5 stage pipeline with an ALU and convertible FIFO. The master acts as an interface between the user and the HW implemented here-in. The module is intended to be integrated with the UDP module as part of NetFPGA infrastructure.

//Comments regarding generic registers:
//MEM_MOD_MAST_REG_ADDR_WIDTH refers to the address width of generic regs assigned to this module
//MEM_MOD_MAST_TAG refers to the tag of the generic registers assigned to this module
//<-----------------------UDP---------------------------->
//<-----------------UDP_REG_ADDR_WIDTH------------------->
//<---MEM_MOD_MAST_TAG--><--MEM_MOD_MAST_REG_ADDR_WIDTH-->
//<----------TAG--------><--------MEM_MOD_MAST----------->

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



 `define UDP_REG_ADDR_WIDTH 4	//for testing
 `define CPCI_NF2_DATA_WIDTH 32	//for testing
 `define MEM_BLOCK_ADDR 0		//for testing
 `define MEM_REG_ADDR_WIDTH 3	//for testing
 `define UDP_REG_SRC_WIDTH 2	//for testing

 `define Number_of_FIFO 5		//for master controller
 
//Memory_module_master
module mem_mod_mast
			#( 	parameter UDP_REG_SRC_WIDTH = 2,
				parameter MAX_DEPTH = 256,										//max convertible fifo depth
				parameter ADDR_WIDTH = 8,										//addr width of convertible fifo
				parameter DATA_WIDTH = 64,										//width of data stored in fifo
				parameter CTRL_WIDTH = DATA_WIDTH/8								//width of control data
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
				
				//generic regs inputs
				input                               reg_req_in,
				input                               reg_ack_in,
				input                               reg_rd_wr_L_in,
				input  [`UDP_REG_ADDR_WIDTH-1:0]    reg_addr_in,
				input  [`CPCI_NF2_DATA_WIDTH-1:0]   reg_data_in,
				input  [UDP_REG_SRC_WIDTH-1:0]     	reg_src_in,

				output                              reg_req_out,
				output                              reg_ack_out,
				output                              reg_rd_wr_L_out,
				output  [`UDP_REG_ADDR_WIDTH-1:0]   reg_addr_out,
				output  [`CPCI_NF2_DATA_WIDTH-1:0]  reg_data_out,
				output  [UDP_REG_SRC_WIDTH-1:0]    	reg_src_out,
					
				//misc inputs
				input clk,
				input rst				
				);
					
					
//UDP Data and control signals
wire [31:0] cmd;
//wire [15:0] DUMMY;
//cmd = 	{MOD_1(1),HW_RDY(1),I_D(1),R_W(1),MOD_0(1),
//			DUMMY(18),
//			Addr_in(9)
//			}
//reg [3:0] dummy;
//reg [20:0] DUMMY; 


//Pipeline signals					
wire IorD;
wire [8:0] Addr_in;
wire [DATA_WIDTH-1:0] Data_In;
wire [DATA_WIDTH-1:0] Data_out_DM;
wire full,empty;
wire nearly_full,nearly_empty;
//wire [2:0] pckt_count;
wire [7:0] FIFO_RD_addr;
wire [7:0] FIFO_WR_addr;
wire [1:0] fifo_state;
wire [8:0] DEPTH;
wire PL_EN, SW_CTRL;
wire [1:0] fifo_rd_en, fifo_wr_en;
wire [1:0] packet_count;
wire PL1_done;
wire [2:0] ip_fifo_sel;	


//Interleaving Data and signals
wire [DATA_WIDTH-1:0] in_data_MC, in_data_PLDCF;
wire [CTRL_WIDTH-1:0] in_ctrl_MC, in_ctrl_PLDCF;
wire in_rdy_MC, in_rdy_PLDCF;
wire in_wr_MC, in_wr_PLDCF;


//State m/c controls & status
/* wire[1:0] MOD;
reg PL_done;
wire pckt_rd_wr;
wire pckt_rd_wr_next;
reg [1:0] state, state_next; */

//Internal Controls for state_00 m/c
//reg RE,RE_next;
//reg WE, WE_next;
// reg R_W_EN,R_W_EN_next;
// reg HW_RDY, HW_RDY_next;
// reg[1:0] state_00,state_00_next;
//wire[1:0] state_11;
// reg[5:0] counter,counter_next;

//FW LW
// wire [ADDR_WIDTH-1:0] first_word;
// wire [ADDR_WIDTH-1:0] last_word;

//Spring FIFO control signals
// reg rd_req, wr_req;
// wire rd_ack, wr_ack;
// wire rd_done, wr_done;
// reg [1:0] ip_rd_count;

//Master controller control signals
//reg [2:0] ip_fifo_sel;

//wire [`UDP_REG_ADDR_WIDTH-1:0] reg_addr_out;

 generic_regs
   #( 
      .UDP_REG_SRC_WIDTH   (UDP_REG_SRC_WIDTH),
      .TAG                 (`MEM_BLOCK_ADDR),          // Tag -- eg. MODULE_TAG. This is the tag assigned for mem_mod_mast generic regs.
      .REG_ADDR_WIDTH      (`MEM_REG_ADDR_WIDTH),     // Width of block addresses -- eg. MODULE_REG_ADDR_WIDTH. This is the width of generic reg addresses.
      .NUM_COUNTERS        (0),                 // Number of counters
      .NUM_SOFTWARE_REGS   (3),                 // Number of sw regs
      .NUM_HARDWARE_REGS   (4)                  // Number of hw regs
   ) module_regs (
      .reg_req_in       (reg_req_in),
      .reg_ack_in       (reg_ack_in),
      .reg_rd_wr_L_in   (reg_rd_wr_L_in),		//1 for read 0 for write. Read operation is bot for SW and HW regs. Write operation only for SW regs since HW regs will be written by HW.
      .reg_addr_in      (reg_addr_in),
      .reg_data_in      (reg_data_in), 			//This data is written to sw reg if reg_addr_in matches the addressses assigned to sw reg
      .reg_src_in       (reg_src_in),

      .reg_req_out      (reg_req_out),
      .reg_ack_out      (reg_ack_out),
      .reg_rd_wr_L_out  (reg_rd_wr_L_out),
      .reg_addr_out     (reg_addr_out),
      .reg_data_out     (reg_data_out),			//This data is read from hw reg if reg_addr_in matches the addressses assigned to hw reg
      .reg_src_out      (reg_src_out),

      // --- counters interface
      .counter_updates  (),
      .counter_decrement(),

      // --- SW regs interface
      .software_regs    ({cmd,Data_In}), 			//Continuous output from SW regs. This op is assigned from sw reg file

       // --- HW regs interface
      .hardware_regs    ({26'h0,packet_count,PL_EN,SW_CTRL,fifo_state,fifo_rd_en,fifo_wr_en,FIFO_WR_addr,FIFO_RD_addr, full, empty, out_rdy, DEPTH, Data_out_DM /*114'h0, out_wr, out_rdy, in_wr, in_rdy, pckt_rd_wr, full, nearly_full, nearly_empty, empty, fifo_state, pckt_count*/}), 			//Continuous input to HW regs. This ip is assigned to hw reg file
//Bit width					4			2		1		1		2			1		1			1		1	  1	    8	   1	  8		21	1	1		1	  8 64
      .clk              (clk),
      .reset            (rst)
    );


//cmd = 	{MOD_1(1),HW_RDY(1),I_D(1),R_W(1),MOD_0(1),
//			DUMMY(18),
//			Addr_in(9)
//			}

pipeline 	 	#(
				.ADDR_WIDTH(ADDR_WIDTH),										//addr width of convertible fifo
				.DATA_WIDTH(DATA_WIDTH),										//width of data stored in fifo
				.CTRL_WIDTH(CTRL_WIDTH),										//width of control data
				.Number_of_FIFO(`Number_of_FIFO) 									//width of control data
				)
				PL1
				(
				//data inputs & outputs from & to UDP
				.in_data(in_data),
				.in_ctrl(in_ctrl),
				.out_data(in_data_MC),
				.out_ctrl(in_ctrl_MC),

				//control signals to and from UDP
				.out_rdy(in_rdy_MC),
				.in_wr(in_wr),
				.in_rdy(in_rdy),
				.out_wr(in_wr_MC),
				
				//control signals from SW reg
				.IorD(),
				.Addr_in(),
				.Data_In(),
				
				//outputs to HW reg
				.Addr_DM(), 			//Read & Write addresses from Data Mem and Instruction mem
				.Addr_IM(),		
				.Data_out_IM(),			//Data out from Data Mem and Instruction mem
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
				.op_fifo_sel(ip_fifo_sel),
				.PL_done(PL1_done),
				
				//misc
				.clk(clk),
				.rst(rst)
				);
 			
master_ctrl	#(
				.ADDR_WIDTH(ADDR_WIDTH),										//addr width of fifo
				.DATA_WIDTH(DATA_WIDTH),										//width of data stored in fifo
				.CTRL_WIDTH(CTRL_WIDTH),								//width of control data
				.Number_of_FIFO(`Number_of_FIFO)							//number of FIFO's we have after pipeline 1
			)
			UUT
			(	
				//interface signals from ip PL
				.ip_fifo_sel(ip_fifo_sel),
				.ip_PL_done(PL1_done),
				
				//interface with dest PL
				
				//Queue signals 						
				.in_rdy(in_rdy_MC),
				.in_wr(in_wr_MC),											//checking for in-wr condition
				.out_rdy(in_rdy_PLDCF),
				.out_wr(in_wr_PLDCF),
				
				//master controller status
				.pckt_count(),			//Its assumed that every FIFO can store 1 packet	
				.state(),
				.full(),
				.nearly_full(),
				.nearly_empty(),
				.empty(),
				.pckt_rd_wr(),
				
				//data signals
				.fifo_data_in(in_data_MC),
				.fifo_data_out(in_data_PLDCF),
				.fifo_ctrl_in(in_ctrl_MC),
				.fifo_ctrl_out(in_ctrl_PLDCF),
				
				//misc
				.clk(clk),
				.rst(rst)

			);			
			

pipeline_dualCF #(
				.MAX_DEPTH(MAX_DEPTH),										//max convertible fifo depth
				.ADDR_WIDTH(ADDR_WIDTH),										//addr width of convertible fifo
				.DATA_WIDTH(DATA_WIDTH),										//width of data stored in fifo
				.CTRL_WIDTH(CTRL_WIDTH),							//width of control data
				.Number_of_FIFO()
				)
				PL2
				(
				//data inputs & outputs from & to UDP
				.in_data(in_data_PLDCF),
				.in_ctrl(in_ctrl_PLDCF),
				.out_data(out_data),
				.out_ctrl(out_ctrl),

				//control signals to and from UDP
				.out_rdy(out_rdy),
				.in_wr(in_wr_PLDCF),
				.in_rdy(in_rdy_PLDCF),
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
				.nearly_full(),			//convertible fifo status
				.nearly_empty(),			//convertible fifo status
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
				
				//Interface with master_control
				.Master_reg(),		//Chose which FIFO in master reg to read data out to
				
				//misc
				.clk(clk),
				.rst(rst)
				);
 
 
assign MOD = {cmd[31],cmd[27]};
assign IorD = cmd[29];
assign Addr_in = cmd[8:0];
//assign state_11 = {PL_done,pckt_rd_wr};

/* always @ (posedge clk, posedge rst)
begin
	if (rst)
	begin
		ip_fifo_sel <= 'h0;
	end
	else
	begin
//testing for sping FIFO
	// ip_rd_count <= 'h1;	
	// if (rd_done)	wr_req <= 1;
	// if (wr_ack)		wr_req <= 0;
	// if (wr_done)	rd_req <= 1;
	// if (rd_ack)		rd_req <= 0;
	
//testing for master controller
	ip_fifo_sel <= ip_fifo_sel + 1;
	end
 end*/

endmodule











//Memory_Module_master Test Bench

`timescale 1ns/100ps
module mem_mod_mast_tb();

//mem_mod_mast signals
	//Inputs to SW regs
reg [31:0] Data_In_hi,Data_In_lo;
//reg [7:0] PLD_RA, PLD_WA;
reg [8:0] LD_A;
reg [17:0] DUMMY;
reg [31:0] reg_addr_data_out;
	//Control bits
reg HW_RDY_in, I_D, R_W;
reg [1:0] MOD;

	//Outputs from HW reg
reg [31:0] Data_out_hi_DM, Data_out_lo_DM, Data_out_IM;
//reg [7:0] WA,RA;	//WA is cirrently unused
wire [31:0] reg_addr_data_in;
reg [12:0] dummy;
reg HW_RDY_out;
reg [8:0] A_IM, A_DM;	   

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


//generic reg signals
	//Inputs
reg reg_req_out;		//out from tb
reg reg_ack_out;	//out from tb
reg reg_rd_wr_L_out; //1 for read 0 for write.Read operation is both for SW and HW regs. Write operation only for SW regs since HW regs will be written by HW.
reg [`UDP_REG_SRC_WIDTH-1:0] reg_src_out;	//put out the source of request: here tb
reg[`UDP_REG_ADDR_WIDTH-1:0] addr_out;	//address to query from / write to

	//Outputs
	
wire reg_req_in;		//in to tb
wire reg_ack_in;		//in to tb
wire reg_rd_wr_L_in;    //in to tb. 1 for read 0 for write.Read operation is both SW and HW regs. Write operation only for SW regs since HW regs will be written by HW.
wire[`UDP_REG_SRC_WIDTH-1:0] reg_src_in;	    //source of request
wire[`UDP_REG_ADDR_WIDTH-1:0] addr_in;		
	
	
mem_mod_mast UUT ( //data inputs & outputs from & to UDP
					.in_data(fifo_data_in),
					.in_ctrl(fifo_ctrl_in),
					.out_data(fifo_data_out),
					.out_ctrl(fifo_ctrl_out),

				//control signals to and from UDP
					.out_rdy(out_rdy),
					.in_wr(in_wr),
					.in_rdy(in_rdy),
					.out_wr(out_wr),

					//Generic reg inputs

//cmd = 	{MOD_1(1),HW_RDY(1),I_D(1),R_W(1),MOD_0(1),
//			DUMMY(18),
//			LD_A(9)
//			}

					.reg_req_in(reg_req_out),
					.reg_ack_in(reg_ack_out),
					.reg_rd_wr_L_in(reg_rd_wr_L_out),
					.reg_addr_in(addr_out),
//					.reg_data_in({PLD_RA,PLD_WA,Data_In}),
					.reg_data_in(reg_addr_data_out),
					.reg_src_in(reg_src_out),

					.reg_req_out(reg_req_in),
					.reg_ack_out(reg_ack_in),
					.reg_rd_wr_L_out(reg_rd_wr_L_in),
					.reg_addr_out(addr_in),
//					.reg_data_out({DUMMY,RA,Data_out}),
					.reg_data_out(reg_addr_data_in),						 
					.reg_src_out(reg_src_in),

					//misc inputs
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
			   
initial begin		//clk generator
	
	clk = 0;
	forever	#10	clk = ~clk;
end
	
initial begin		//reset 
	rst = 1;
	#30	rst = 0;

end
	
initial begin		//Initialize all the input registers to mem_mod_mast & setup outputs recording

	LD_A = 'h0001;
//	PLD_RA = 'h 0001;
//	PLD_WA = 'h 0001;
	Data_In_hi = 'h BAAB;
	Data_In_lo = 'h ABBA;
	reg_addr_data_out = 'h BEEF;
	DUMMY = 'h 0;
	dummy = 'h 0; 
	count = 0;
//	PL_EN = 0;			// Pipepline disabled
	MOD = 'h0;
	HW_RDY_in = 1;		//when this is written to SW reg, master knows that the last bit has arrived and ready to execute read/write
	I_D = 1;			//Write to Instruction mem
	R_W = 0;			//Write operation
	// fifo_data_in = 'h0;
	// fifo_ctrl_in = 'h0;
	out_rdy = 1;		//outputs are ready to be received
	in_wr = 0;			//inputs provided shall be written to FIFO
	reg_wa = 0;
	reg_ra = 0;
	counter_restart = 25;
	
	//$readmemh ("ip_reg_init.txt",ip_reg);
	$readmemh ("ip_reg_init_var_pckt.txt",ip_reg);
	
	fq = $fopen ("mem_mod_mast_tb_op.txt", "w");
	if (!fq) $display ("Failed genrating output file");
end				

always @ (posedge clk)
	begin
		reg_rd_wr_L_out <= 1; 	//Default to read
		reg_req_out <= 0;		//Request operation on generic regs
		count <= count + 1;
	
		if (count < 2)	//enable write to SW reg
			begin
				reg_rd_wr_L_out <= 0; 	//Enable write to SW reg
				reg_req_out <= 1;		//Request operation on generic regs
				reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				reg_src_out <= 2'b00;
				addr_out <= 'h0	;
				reg_addr_data_out <= Data_In_lo;
			end	

		else if (count < 3)	//enable write to SW reg
			begin
				reg_rd_wr_L_out <= 0; 	//Enable write to SW reg
				reg_req_out <= 1;		//Request operation on generic regs
				reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				reg_src_out <= 2'b00;
				addr_out <= 'h1;
				reg_addr_data_out <= Data_In_hi;
			end	
	
		else if (count < 4)	//enable write to SW reg
			begin
				reg_rd_wr_L_out <= 0; 	//Enable write to SW reg
				reg_req_out <= 1;		//Request operation on generic regs
				reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				reg_src_out <= 2'b00;
				addr_out <= 'h2;
				reg_addr_data_out <= {MOD[1],HW_RDY_in,I_D,R_W,MOD[0],DUMMY,LD_A};
			end
		
		else if (count < 8)	//enable read from HW reg
			begin
				reg_rd_wr_L_out <= 1; 	//Enable read from HW reg
				reg_req_out <= 1;		//Request operation on generic regs
				reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				reg_src_out <= 2'b00;
				addr_out <= 'h6;
			end
		else if (count < 9)	//enable read from HW reg
			begin
				reg_rd_wr_L_out <= 1; 	//Enable read from HW reg
				reg_req_out <= 1;		//Request operation on generic regs
				reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				reg_src_out <= 2'b00;
				addr_out <= 'h5;
			end
		else if (count < 10)	//enable read from HW reg
			begin
				reg_rd_wr_L_out <= 1; 	//Enable read from HW reg
				reg_req_out <= 1;		//Request operation on generic regs
				reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				reg_src_out <= 2'b00;
				addr_out <= 'h4;
			end
		else if (count < 11)	//enable read from HW reg
			begin
				reg_rd_wr_L_out <= 1; 	//Enable read from HW reg
				reg_req_out <= 1;		//Request operation on generic regs
				reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				reg_src_out <= 2'b00;
				addr_out <= 'h3;
			end

	// enable PL/fifo mod
		// else if (count < 12)
			// MOD <= 'h3;
		// else if (count < 13)	//enable write to SW reg
			// begin
				// reg_rd_wr_L_out <= 0; 	//Enable write to SW reg
				// reg_req_out <= 1;		//Request operation on generic regs
				// reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				// reg_src_out <= 2'b00;
				// addr_out <= 'h2;
				// reg_addr_data_out <= {MOD[1],HW_RDY_in,I_D,R_W,MOD[0],DUMMY,LD_A};
			// end
		//enable fifo mode
		// else if (count == 51)
			// MOD <= 'h0;
		// else if (count == 52)	//enable write to SW reg
			// begin
				// reg_rd_wr_L_out <= 0; 	//Enable write to SW reg
				// reg_req_out <= 1;		//Request operation on generic regs
				// reg_ack_out <= 0;		//Make ack out 0 so that generic reg can make it 1
				// reg_src_out <= 2'b00;
				// addr_out <= 'h2;
				// reg_addr_data_out <= {MOD[1],HW_RDY_in,I_D,R_W,MOD[0],DUMMY,LD_A};
			// end
	
		//testing FIFO
		// else if (count < 13)			//1st word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h9999_1010_0101_1010;
			// end
		// else if (count < 17)		//inject header = 4 words
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'h0101_1010_0101_1010;			
			// end
		// else if (count < 18)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_0000;		
		// else if (count < 19)	//inject data body = 6 words
						// fifo_data_in <= 'hABCD_AAAA_CAFE_1000;		
		// else if (count < 20)	//inject data body = 6 words
						// fifo_data_in <= 'hABCD_AAAA_CAFE_2000;		
		// else if (count < 21)	//inject data body = 6 words
						// fifo_data_in <= 'hABCD_AAAA_CAFE_3000;		
		// else if (count < 22)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_4000;						
		// else if (count < 23)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_5000;				
		// else if (count < 24)	//last word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h0101_1010_0101_9999;			
			// end
		// else if (count < 25)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_EEEE_EEEE_EEEE;
			// end
		// else if (count < 26)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_FFFF_EEEE_EEEE;
			// end
		
				//next word
		
		// else if (count < 33)			//1st word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h9999_1010_0101_1010;
			// end
		// else if (count < 37)		//inject header = 4 words
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'h0101_1010_0101_1010;			
			// end
		// else if (count < 43)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_0000;				
		// else if (count < 44)	//last word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h0101_1010_0101_9999;			
			// end
		// else if (count < 45)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_EEEE_EEEE_EEEE;
			// end
		// else if (count < 46)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_FFFF_EEEE_EEEE;
			// end

			//next word
		// else if (count < 53)			//1st word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h9999_1010_0101_1010;
			// end
		// else if (count < 57)		//inject header = 4 words
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'h0101_1010_0101_1010;			
			// end
		// else if (count < 63)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_0000;				
		// else if (count < 64)	//last word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h0101_1010_0101_9999;			
			// end
		// else if (count < 65)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_EEEE_EEEE_EEEE;
			// end
		// else if (count < 66)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_FFFF_EEEE_EEEE;
			// end
			
			
		// else if (count < 74)			//1st word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h9999_1010_0101_1010;
			// end
		// else if (count < 78)		//inject header = 4 words
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'h0101_1010_0101_1010;			
			// end
		// else if (count < 79)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_0000;		
		// else if (count < 80)	//inject data body = 6 words
						// fifo_data_in <= 'hABCD_AAAA_CAFE_1000;		
		// else if (count < 81)	//inject data body = 6 words
						// fifo_data_in <= 'hABCD_AAAA_CAFE_2000;		
		// else if (count < 82)	//inject data body = 6 words
						// fifo_data_in <= 'hABCD_AAAA_CAFE_3000;		
		// else if (count < 83)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_4000;						
		// else if (count < 84)	//inject data body = 6 words
				// fifo_data_in <= 'hABCD_AAAA_CAFE_5000;				
		// else if (count < 85)	//last word
			// begin
				// in_wr <= 1'b1;
				// fifo_ctrl_in <= 'hff;
				// fifo_data_in <= 'h0101_1010_0101_9999;			
			// end
		// else if (count < 86)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_EEEE_EEEE_EEEE;
			// end
		// else if (count < 87)
			// begin
				// fifo_ctrl_in <= 'h0;
				// fifo_data_in <= 'hEEEE_FFFF_EEEE_EEEE;
			// end
		
				//next word

			
		
		//enable PL, disable output read operation
		else if (count == 50 )
			begin
			//	PL_EN <= 1;
				out_rdy <= 0;
				in_wr <= 0;
			end
		else if (count == 100)
				out_rdy <= 1;
		// read data from HW regs

		if (addr_in == 'h6)	{dummy,HW_RDY_out,A_IM,A_DM} <= reg_addr_data_in;
		if (addr_in == 'h5)	Data_out_IM <= reg_addr_data_in;
		if (addr_in == 'h4)	Data_out_hi_DM <= reg_addr_data_in;
		if (addr_in == 'h3)	Data_out_lo_DM <= reg_addr_data_in;
		
		
		
		//take inputs from dummy reg file into fifo
		if (count > 13)
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
							
		
		
		
		
		
		//take outputs from fifo into dummy reg file
		if(out_wr)
			reg_wa <= reg_wa +1;
		else 
			reg_wa <= reg_wa;
			
		

		if (reg_req_in & reg_ack_in & reg_src_in == 0)
			begin
				$fdisplay (fq,"%d Success!! %h %h\t\t %h_%h",count,A_IM,A_DM, Data_out_hi_DM, Data_out_lo_DM);
			end

		else if (reg_req_in & !reg_ack_in & reg_src_in == 0)
			begin
				$fdisplay (fq,"Failed:( Ja mar ja");
			end
		else if (!reg_req_in)
			begin
				$fdisplay (fq,"Ab to Doob ke mar ja");
			end
		
		if (count == 40)	$fclose(fq);

	end


endmodule