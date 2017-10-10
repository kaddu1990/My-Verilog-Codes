// Amanpreet Kaur  and  Nishant Mathur
// Date: March 4, 2017
// changes: May 1, 2017


// Control Logic for Pipeline

`timescale 1ns / 1ps

// -----------function ----------- opcode --------//
`define 		ADDIW				'd27
`define 		IMMEDIATE			'd19   			//ADDI, LI, MOV, NOP, SLLI
`define			ADD_SUB_AND_OR		'd51
`define 		SD					'd35			//SD, SW
`define 		BLE					'd99			//BLE
`define 		LD					'd3				//LD, LW
`define 		J					'd111
`define			FIFO				'd25			//PL_done, FW_ld //generates raw PL_done and loads FW by setting FW_ld = 1
`define			Master_Control		'd30			//will give 3 bit data to master_controller

module control (
				input [31:0] instruction,
				output reg RegWrite,
				output reg ALUSrc,
				output reg MemWrite,
				output reg MemtoReg,	
				output reg Jump,	
				output reg ble,	
				output reg imme, 
				output reg PL_done,
				output reg FW_ld,
				output reg [3:0] ALUop,
				output reg [2:0] Master_reg,
				output reg mem_sel);

wire [6:0]opcode;
wire [2:0]func3;
wire [6:0]func7;

// assigns
assign opcode = instruction[6:0];
assign func3 = instruction[14:12];
assign func7 = instruction[31:25];


always @(*)
begin
RegWrite = 0;
ALUSrc = 0;
MemWrite = 0;
MemtoReg = 0;
Jump = 0;
ble = 0;
imme = 0;
ALUop = 'd0;
FW_ld = 0;
PL_done = 0;
Master_reg = 'd0;
mem_sel = 'b0;
	case (opcode)	
		
			`ADDIW: 
					begin										
					RegWrite = 'b1;
					ALUSrc = 'b1;
					MemtoReg = 'b1;
					imme = 'b1;
					PL_done = 0;
					end	
					
			`IMMEDIATE:
					begin
					RegWrite = 'b1;
					ALUSrc = 'b1;
					MemtoReg = 'b1;
					imme = 'b1;
					PL_done = 0;
					if (func3 == 'b000)     //operation requiring ALU add like ADDI, LI, MOV, NOP
						ALUop = 'd0;
					
					else if(func3 == 'b001) //for SLLI
						ALUop='d6;
					
					else if(func3 == 'b100) //for NOT
						ALUop='d4;
						
					else if(func3 == 'b101) 
						if ((func7 == 'b0000000) || (func7 == 'b0000001)) //for SRLI
							ALUop = 'd7;
						
						else if((func7 == 'b0100000) || (func7 == 'b0100001)) //for SRAI
							ALUop = 'd8;
						
					else
						ALUop='d0;
					end
					
			`ADD_SUB_AND_OR:
					begin
					RegWrite = 'b1;
					MemtoReg = 'b1;
					PL_done = 0;
					if (func7 == 'b0000000)
						if(func3 == 'b000) 
							ALUop = 'd0;
						else if(func3 == 'b110) 
							ALUop = 'd3;
						else if(func3 == 'b111) 
							ALUop = 'd2;
						
					else if(func7 == 'b0100000)
						ALUop = 'd1;
						
					else
						ALUop = 'd0;
					end
			
			`SD:
					begin
					if(func3 == 'b111) 
						mem_sel = 'b1;
					else
						mem_sel = 'b0;
					ALUSrc = 'b1;
					MemWrite = 'b1;
					PL_done = 0;
					end
			
			`BLE:	begin
					 ble = 'b1;
					 PL_done = 0;
					end
				
			`LD:
					begin
					if(func3 == 'b111) 
						mem_sel = 'b1;
					else
						mem_sel = 'b0;					
					RegWrite = 'b1;
					ALUSrc = 'b1;
					imme = 'b1;
					PL_done = 0;
					end
					
			`J:
					begin 
					RegWrite = 'b1;
					Jump = 'b1;
					PL_done = 0;
					end
					
			`FIFO:
					begin
					FW_ld = 0;
					PL_done = 0;
					if (func3 == 'b000)				//funct3 = 0 means load FW into register file
						begin
						FW_ld = 1;
						RegWrite = 'b1;
						end
					else if(func3 == 'b001)			//funct3 = 1 means my processing is done and this is the last instruction
						PL_done = 1;	
					else
					begin
						FW_ld = 0;
						PL_done = 0;
					end
					end
					
			`Master_Control:
					begin
					if (func3 == 'b000)				//no match
						Master_reg = 'b000;
					if (func3 == 'b001)				//source 0
						Master_reg = 'b001;
					if (func3 == 'b010)				//source 1
						Master_reg = 'b010;
					if (func3 == 'b011)				//dest 0
						Master_reg = 'b011;
					if (func3 == 'b100)				//dest 1
						Master_reg = 'b100;
					if (func3 == 'b101)				//dest 2
						Master_reg = 'b101;
					if (func3 == 'b110)				//dest 3
						Master_reg = 'b110;
					end
					
			default:
					begin
						RegWrite = 0;
						ALUSrc = 0;
						MemWrite = 0;
						MemtoReg = 0;
						Jump = 0;
						ble = 0;
						imme = 0;
						ALUop = 'd0;
						FW_ld = 0;
						PL_done = 0;
						Master_reg = 'd0;
						mem_sel = 'b0;
					end
	endcase
	
end
endmodule



`timescale 1ns / 1ps

module control_tb;

reg clk;
reg [31:0]instruction;
wire RegWrite;
wire ALUSrc;
wire MemWrite;
wire MemtoReg;
wire Jump;
wire ble;
wire imme;
wire [3:0] ALUop;
wire [2:0]Master_reg;
wire mem_sel;
integer count;
					
control control(.instruction(instruction), .RegWrite(RegWrite), .ALUSrc(ALUSrc), .MemWrite(MemWrite), .MemtoReg(MemtoReg), .Jump(Jump), .ble(ble), .imme(imme), .ALUop(ALUop), .Master_reg(Master_reg), .mem_sel(mem_sel));

initial
begin

instruction = 'h00000000;
clk = 0;
count = 0;

forever #10 clk = ~clk;
end

always @(posedge clk)
begin
	count <= count + 1;
	
	if (count == 1)
		begin
//		instruction <= 'h0017879b;			//addiw
		instruction <= 'h0000001e;			//0
		end
	
	if (count == 2)
		begin
//		instruction <= 'h00279793;			//slli
		instruction <= 'h0000101e;			//1
		end
		
	if (count == 3)
		begin
//		instruction <= 'hfec42783;			//lw
		instruction <= 'h0000201e;			//2
		end
		
	if (count == 4)
		begin
//		instruction <= 'hfe042623;			//sw
		instruction <= 'h0000301e;			//3
		end
	
	if (count == 5)
		begin
//		instruction <= 'h06e7d663;			//ble
		instruction <= 'h0000401e;			//4
		end
		
	if (count == 6)
		begin
//		instruction <= 'h00f707b3;			//add
		instruction <= 'h0000501e;			//5
		end
		
	if (count == 7)
		begin
//		instruction <= 'h0b40006f;			//jump
		instruction <= 'h0000601e;			//6
		end
		
	if (count == 8)
		begin
		// instruction <= 'h00000019;			//PL_done
		instruction <= 'hfe047623;			//special sw
		end 
		
	// if (count == 7)
		// begin
		// instruction <= 'h00001019;			//FW_ld
		// end
end
					
endmodule 