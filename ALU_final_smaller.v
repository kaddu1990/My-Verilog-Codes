//Amanpreet Kaur // Reetinder Kaur // Nishant Mathur
//Lab 7 // Part 1 // ALU design

`timescale 1ns/100ps

// ---------- function ------ opcode --------//
`define 		ADD				0
`define 		SUB				1
`define			AND				2
`define			OR				3
`define			NOT				4 
`define 		SHIFT_L			6
`define 		SHIFT_R			7
`define 		SHIFT_RA		8



//-------------------- ALU_CONTROL --------------------------//

module ALU_ctrl(input [63:0] A,
				input [63:0] B,
				input [3:0] cmd,
				output [31:0] psw,
				output reg [63:0] Data_out);

//psw = 32 bit register, psw[0] = Carry, psw[1] = Borrow, psw[2] = gt, psw[3] = eq, psw[4] = lt //


//wire [63:0] Data_out1;
//wire [63:0] Data_out2;
wire [63:0] Data_out6;
wire [63:0] Data_out7;
wire [63:0] Data_out8;					  

			// add_sub_64bit alu1(.A(A), .B(B), .Cin(1'b0), .add_sub(1'b0), .Cout(psw[0]), .Sum(Data_out1));
			
			// add_sub_64bit alu2(.A(A), .B(B), .Cin(1'b1), .add_sub(1'b1), .Cout(psw[1]), .Sum(Data_out2));			
						
			shift_L_register alu7(.data_in(A), .shamt(B), .data_out(Data_out6));

			shift_R_register alu8(.data_in(A), .shamt(B), .data_out(Data_out7));
			
			shift_RA_register alu9(.data_in(A), .shamt(B), .data_out(Data_out8));
					
			
always @ (*)
begin

	case (cmd)
		
		`ADD:
		Data_out = A + B;
		
		`SUB:
		Data_out = A - B;
		
		`AND:
		Data_out = A & B;
		
		`OR:
		Data_out = A | B;
		
		`NOT:
		Data_out = ~A;
		
		`SHIFT_L:
		Data_out = Data_out6;
		
		`SHIFT_R:
		Data_out = Data_out7;
		
		`SHIFT_RA:
		Data_out = Data_out8;						 

		default:
		Data_out = 64'h0;
	
	endcase

end			
			

endmodule



// ------------------  1-bit Adder  -------------------------------- //

module one_bit_add_Sub (input A,
						input B,
						input Cin,
						input b_invert,
						output Cout,
						output Sum);

						
//wire op;
//wire S;
//wire carry;
wire Bin;

//assign carry = b_invert ? 1 : Cin;		//Select Cin on the basis of add/sub
assign Bin = b_invert ^ B; 			//Invert B on subtraction

	xor(Sum, A, Cin, Bin);
    and(w1, A, Bin);   
    and(w2, A, Cin);
    and(w3, Bin, Cin);   
    or(Cout, w1, w2, w3);
	
// assign Sum = A ^ Bin ^ Cin;
// assign Cout = (A&Bin) | (Bin&Cin) | (Cin&A);

endmodule



// --------------------  64-bit Adder/Subtractor  ---------------------- //

module add_sub_64bit(input [63:0] A,
				input [63:0] B,
				input Cin,
				input add_sub,
				output Cout,
				output [63:0] Sum);
				
wire [62:0] carry_prop;
wire carry_out;
wire carry;
assign carry = add_sub ? 1 : Cin;

one_bit_add_Sub A0( .A(A[0]), .B(B[0]), .Cin(carry), .b_invert(add_sub), .Cout(carry_prop[0]), .Sum(Sum[0]));
genvar i;
generate
	for (i=1; i<=62; i=i+1)
	begin: adder
		one_bit_add_Sub A1( .A(A[i]), .B(B[i]), .Cin(carry_prop[i-1]), .b_invert(add_sub), .Cout(carry_prop[i]), .Sum(Sum[i]));
	end
endgenerate
one_bit_add_Sub A2( .A(A[63]), .B(B[63]), .Cin(carry_prop[62]), .b_invert(add_sub), .Cout(carry_out), .Sum(Sum[63]));

assign Cout = add_sub ? ~carry_out : carry_out;

endmodule



//-------------------- Shift left -----------------------------//

module shift_L_register (data_in,shamt,data_out);
parameter WIDTH = 64;
//	input clk;
	input [WIDTH-1:0] data_in;	
	input [WIDTH-1:0] shamt;		//Shift amount
	output [WIDTH-1:0]  data_out;	
	
	
//	reg [5:0] i;
	reg [WIDTH-1:0] data_out;
//	reg [WIDTH-1:0] temp;

always @ *
begin
	case(shamt[5:0])
		'h0: data_out = {data_in};
		'h1: data_out = {data_in[WIDTH-2:0],1'h0};
		'h2: data_out = {data_in[WIDTH-3:0],2'h0};
		'h3: data_out = {data_in[WIDTH-4:0],3'h0};
		'h4: data_out = {data_in[WIDTH-5:0],4'h0};
 		'h5: data_out = {data_in[WIDTH-6:0],5'h0};
		'h6: data_out = {data_in[WIDTH-7:0],6'h0};
		'h7: data_out = {data_in[WIDTH-8:0],7'h0};
		'h8: data_out = {data_in[WIDTH-9:0],8'h0};
		'h9: data_out = {data_in[WIDTH-10:0],9'h0};
		'hA: data_out = {data_in[WIDTH-11:0],10'h0};
		'hB: data_out = {data_in[WIDTH-12:0],11'h0};
		'hC: data_out = {data_in[WIDTH-13:0],12'h0};
		'hD: data_out = {data_in[WIDTH-14:0],13'h0};
		'hE: data_out = {data_in[WIDTH-15:0],14'h0};
		'hF: data_out = {data_in[WIDTH-16:0],15'h0};
		'h10: data_out = {data_in[WIDTH-17:0],16'h0};
		'h11: data_out = {data_in[WIDTH-18:0],17'h0};
		'h12: data_out = {data_in[WIDTH-19:0],18'h0};
		'h13: data_out = {data_in[WIDTH-20:0],19'h0};
		'h14: data_out = {data_in[WIDTH-21:0],20'h0};
		'h15: data_out = {data_in[WIDTH-22:0],21'h0};
 		'h16: data_out = {data_in[WIDTH-23:0],22'h0};
		'h17: data_out = {data_in[WIDTH-24:0],23'h0};
		'h18: data_out = {data_in[WIDTH-25:0],24'h0};
		'h19: data_out = {data_in[WIDTH-26:0],25'h0};
		'h1A: data_out = {data_in[WIDTH-27:0],26'h0};
		'h1B: data_out = {data_in[WIDTH-28:0],27'h0};
		'h1C: data_out = {data_in[WIDTH-29:0],28'h0};
		'h1D: data_out = {data_in[WIDTH-30:0],29'h0};
		'h1E: data_out = {data_in[WIDTH-31:0],30'h0};
		'h1F: data_out = {data_in[WIDTH-32:0],31'h0};
		'h20: data_out = {data_in[WIDTH-33:0],32'h0};
		'h21: data_out = {data_in[WIDTH-34:0],33'h0};
		'h22: data_out = {data_in[WIDTH-35:0],34'h0};
		'h23: data_out = {data_in[WIDTH-17:0],35'h0};
		'h24: data_out = {data_in[WIDTH-18:0],36'h0};
		'h25: data_out = {data_in[WIDTH-19:0],37'h0};
		'h26: data_out = {data_in[WIDTH-20:0],38'h0};
		'h27: data_out = {data_in[WIDTH-21:0],39'h0};
		'h28: data_out = {data_in[WIDTH-22:0],40'h0};
 		'h29: data_out = {data_in[WIDTH-23:0],41'h0};
		'h2A: data_out = {data_in[WIDTH-24:0],42'h0};
		'h2B: data_out = {data_in[WIDTH-25:0],43'h0};
		'h2C: data_out = {data_in[WIDTH-26:0],44'h0};
		'h2D: data_out = {data_in[WIDTH-27:0],45'h0};
		'h2E: data_out = {data_in[WIDTH-28:0],46'h0};
		'h2F: data_out = {data_in[WIDTH-29:0],47'h0};
		'h30: data_out = {data_in[WIDTH-30:0],48'h0};
		default: data_out = data_in;
	endcase

end	
	

endmodule

 
  //-------------------- Shift right -----------------------------//

module shift_R_register (data_in,shamt,data_out);
parameter WIDTH = 64;
//	input clk;
	input [WIDTH-1:0] data_in;	
	input [WIDTH-1:0] shamt;		//Shift amount
	output [WIDTH-1:0]  data_out;	
	
	
//	reg [5:0] i;
	reg [WIDTH-1:0] data_out;
//	reg [WIDTH-1:0] temp;

always @ *
begin
	case(shamt[5:0])
		'h0: data_out = {data_in};
		'h1: data_out = {1'h0,data_in[WIDTH-1:1]};
		'h2: data_out = {2'h0,data_in[WIDTH-1:2]};
		'h3: data_out = {3'h0,data_in[WIDTH-1:3]};
		'h4: data_out = {4'h0,data_in[WIDTH-1:4]};
		'h5: data_out = {5'h0,data_in[WIDTH-1:5]};
		'h6: data_out = {6'h0,data_in[WIDTH-1:6]};
		'h7: data_out = {7'h0,data_in[WIDTH-1:7]};
		'h8: data_out = {8'h0,data_in[WIDTH-1:8]};
		'h9: data_out = {9'h0,data_in[WIDTH-1:9]};
		'hA: data_out = {10'h0,data_in[WIDTH-1:10]};
		'hB: data_out = {11'h0,data_in[WIDTH-1:11]};
		'hC: data_out = {12'h0,data_in[WIDTH-1:12]};
		'hD: data_out = {13'h0,data_in[WIDTH-1:13]};
		'hE: data_out = {14'h0,data_in[WIDTH-1:14]};
		'hF: data_out = {15'h0,data_in[WIDTH-1:15]};
		'h10: data_out = {16'h0,data_in[WIDTH-1:16]};
		'h11: data_out = {17'h0,data_in[WIDTH-1:17]};
		'h12: data_out = {18'h0,data_in[WIDTH-1:18]};
		'h13: data_out = {19'h0,data_in[WIDTH-1:19]};
		'h14: data_out = {20'h0,data_in[WIDTH-1:20]};
		'h15: data_out = {21'h0,data_in[WIDTH-1:21]};
		'h16: data_out = {22'h0,data_in[WIDTH-1:22]};
		'h17: data_out = {23'h0,data_in[WIDTH-1:23]};
		'h18: data_out = {24'h0,data_in[WIDTH-1:24]};
		'h19: data_out = {25'h0,data_in[WIDTH-1:25]};
		'h1A: data_out = {26'h0,data_in[WIDTH-1:26]};
		'h1B: data_out = {27'h0,data_in[WIDTH-1:27]};
		'h1C: data_out = {28'h0,data_in[WIDTH-1:28]};
		'h1D: data_out = {29'h0,data_in[WIDTH-1:29]};
		'h1E: data_out = {30'h0,data_in[WIDTH-1:30]};
		'h1F: data_out = {31'h0,data_in[WIDTH-1:31]};
		'h20: data_out = {32'h0,data_in[WIDTH-1:32]};
		'h21: data_out = {33'h0,data_in[WIDTH-1:33]};
		'h22: data_out = {34'h0,data_in[WIDTH-1:34]};
		'h23: data_out = {35'h0,data_in[WIDTH-1:35]};
		'h24: data_out = {36'h0,data_in[WIDTH-1:36]};
		'h25: data_out = {37'h0,data_in[WIDTH-1:37]};
		'h26: data_out = {38'h0,data_in[WIDTH-1:38]};
		'h27: data_out = {39'h0,data_in[WIDTH-1:39]};
		'h28: data_out = {40'h0,data_in[WIDTH-1:40]};
		'h29: data_out = {41'h0,data_in[WIDTH-1:41]};
		'h2A: data_out = {42'h0,data_in[WIDTH-1:42]};
		'h2B: data_out = {43'h0,data_in[WIDTH-1:43]};
		'h2C: data_out = {44'h0,data_in[WIDTH-1:44]};
		'h2D: data_out = {45'h0,data_in[WIDTH-1:45]};
		'h2E: data_out = {46'h0,data_in[WIDTH-1:46]};
		'h2F: data_out = {47'h0,data_in[WIDTH-1:47]};
		'h30: data_out = {48'h0,data_in[WIDTH-1:48]};
		default: data_out = data_in;
	endcase

end	
	
	
// always @ *
// begin
// data_out = data_in;
// for (i=0;i<63;i=i+1)
	// begin
		// if (i<shamt[5:0])
		// data_out = data_out>>1;
		// else
		// data_out = data_out;
	// end
// end	
	

endmodule 


  //-------------------- Shift right -----------------------------//

module shift_RA_register (data_in,shamt,data_out);
parameter WIDTH = 64;
//	input clk;
	input [WIDTH-1:0] data_in;	
	input [WIDTH-1:0] shamt;		//Shift amount
	output [WIDTH-1:0]  data_out;	
	
	
//	reg [5:0] i;
	reg [WIDTH-1:0] data_out;
//	reg [WIDTH-1:0] temp;

always @ *
begin
	case(shamt[5:0])
		'h0: data_out = {data_in};
		'h1: data_out = {{data_in[WIDTH-1]},data_in[WIDTH-1:1]};
		'h2: data_out = {{2{data_in[WIDTH-1]}},data_in[WIDTH-1:2]};
		'h3: data_out = {{3{data_in[WIDTH-1]}},data_in[WIDTH-1:3]};
		'h4: data_out = {{4{data_in[WIDTH-1]}},data_in[WIDTH-1:4]};
		'h5: data_out = {{5{data_in[WIDTH-1]}},data_in[WIDTH-1:5]};
		'h6: data_out = {{6{data_in[WIDTH-1]}},data_in[WIDTH-1:6]};
		'h7: data_out = {{7{data_in[WIDTH-1]}},data_in[WIDTH-1:7]};
		'h8: data_out = {{8{data_in[WIDTH-1]}},data_in[WIDTH-1:8]};
		'h9: data_out = {{9{data_in[WIDTH-1]}},data_in[WIDTH-1:9]};
		'hA: data_out = {{10{data_in[WIDTH-1]}},data_in[WIDTH-1:10]};
		'hB: data_out = {{11{data_in[WIDTH-1]}},data_in[WIDTH-1:11]};
		'hC: data_out = {{12{data_in[WIDTH-1]}},data_in[WIDTH-1:12]};
		'hD: data_out = {{13{data_in[WIDTH-1]}},data_in[WIDTH-1:13]};
		'hE: data_out = {{14{data_in[WIDTH-1]}},data_in[WIDTH-1:14]};
		'hF: data_out = {{15{data_in[WIDTH-1]}},data_in[WIDTH-1:15]};
		'h10: data_out = {{16{data_in[WIDTH-1]}},data_in[WIDTH-1:16]};
		'h11: data_out = {{17{data_in[WIDTH-1]}},data_in[WIDTH-1:17]};
		'h12: data_out = {{18{data_in[WIDTH-1]}},data_in[WIDTH-1:18]};
		'h13: data_out = {{19{data_in[WIDTH-1]}},data_in[WIDTH-1:19]};
		'h14: data_out = {{20{data_in[WIDTH-1]}},data_in[WIDTH-1:20]};
		'h15: data_out = {{21{data_in[WIDTH-1]}},data_in[WIDTH-1:21]};
		'h16: data_out = {{22{data_in[WIDTH-1]}},data_in[WIDTH-1:22]};
		'h17: data_out = {{23{data_in[WIDTH-1]}},data_in[WIDTH-1:23]};
		'h18: data_out = {{24{data_in[WIDTH-1]}},data_in[WIDTH-1:24]};
		'h19: data_out = {{25{data_in[WIDTH-1]}},data_in[WIDTH-1:25]};
		'h1A: data_out = {{26{data_in[WIDTH-1]}},data_in[WIDTH-1:26]};
		'h1B: data_out = {{27{data_in[WIDTH-1]}},data_in[WIDTH-1:27]};
		'h1C: data_out = {{28{data_in[WIDTH-1]}},data_in[WIDTH-1:28]};
		'h1D: data_out = {{29{data_in[WIDTH-1]}},data_in[WIDTH-1:29]};
		'h1E: data_out = {{30{data_in[WIDTH-1]}},data_in[WIDTH-1:30]};
		'h1F: data_out = {{31{data_in[WIDTH-1]}},data_in[WIDTH-1:31]};
		'h20: data_out = {{32{data_in[WIDTH-1]}},data_in[WIDTH-1:32]};
		'h21: data_out = {{33{data_in[WIDTH-1]}},data_in[WIDTH-1:33]};
		'h22: data_out = {{34{data_in[WIDTH-1]}},data_in[WIDTH-1:34]};
		'h23: data_out = {{35{data_in[WIDTH-1]}},data_in[WIDTH-1:35]};
		'h24: data_out = {{36{data_in[WIDTH-1]}},data_in[WIDTH-1:36]};
		'h25: data_out = {{37{data_in[WIDTH-1]}},data_in[WIDTH-1:37]};
		'h26: data_out = {{38{data_in[WIDTH-1]}},data_in[WIDTH-1:38]};
		'h27: data_out = {{39{data_in[WIDTH-1]}},data_in[WIDTH-1:39]};
		'h28: data_out = {{40{data_in[WIDTH-1]}},data_in[WIDTH-1:40]};
		'h29: data_out = {{41{data_in[WIDTH-1]}},data_in[WIDTH-1:41]};
		'h2A: data_out = {{42{data_in[WIDTH-1]}},data_in[WIDTH-1:42]};
		'h2B: data_out = {{43{data_in[WIDTH-1]}},data_in[WIDTH-1:43]};
		'h2C: data_out = {{44{data_in[WIDTH-1]}},data_in[WIDTH-1:44]};
		'h2D: data_out = {{45{data_in[WIDTH-1]}},data_in[WIDTH-1:45]};
		'h2E: data_out = {{46{data_in[WIDTH-1]}},data_in[WIDTH-1:46]};
		'h2F: data_out = {{47{data_in[WIDTH-1]}},data_in[WIDTH-1:47]};
		'h30: data_out = {{48{data_in[WIDTH-1]}},data_in[WIDTH-1:48]};
		default: data_out = data_in;
	endcase

end	
	
	
// always @ *
// begin
// data_out = data_in;
// for (i=0;i<63;i=i+1)
	// begin
		// if (i<shamt[5:0])
		// data_out = data_out>>1;
		// else
		// data_out = data_out;
	// end
// end	
	

endmodule


module ALU_ctrl_tb();

reg [63:0] A;
reg [63:0] B;
reg [3:0] cmd;
reg clk;
reg rst;
wire [31:0] psw;
wire [63:0] Data_out;

ALU_ctrl ALU_ctrl(.A(A), .B(B), .cmd(cmd), .psw(psw), .Data_out(Data_out));	

initial 
begin
clk=0;
rst=1;

forever #10 clk=~clk;
			
end

initial
begin
#99
rst=0;
A='hFFFF_FFFF_FFFF_FFFF;
B='h0000_0000_0000_0030;
cmd=0;

#20
cmd=1;

#20
cmd=2;

#20
cmd=3;

#20
cmd=4;

#20
cmd=6;

#20
cmd=7;

#20
cmd=8;

end

endmodule