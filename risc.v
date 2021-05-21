`include "adder.v"
`include "alu.v"
`include "branchpredict.v"
`include "define.v"
`include "equality.v"
`include "increment.v"
`include "dmemory.v"
`include "forwarding.v"
`include "imemory.v"
`include "register.v"
`include "mux2.v"
`include "mux3.v"
`include "mux4.v"
`include "mux8.v"
`include "pcaddr.v"
`include "pencoder.v"
`include "regfile.v"
`include "signex6.v"
`include "signex9.v"
`include "zeropad.v"



module risc(input clk, input rst);
reg  [ 50:0] 	IFIDpipereg;
reg  [121:0]	IDEXpipereg;
reg  [136:0]	EXMMpipereg;
reg  [151:0]	MMWBpipereg;
reg  [ 18:0]	WBpipereg;

wire [ 15:0]	shifted_input, o_pc_rf, inc_o_pc_rf, instruction, i_pc_rf, o_data1_rf, o_data2_rf, i_data_rf, extended6, extended9, zeropad, IDextended, IDpc_p_ex, i_alu1, i_alu2, o_alu, i_data_dmem, o_data_dmem, addr_dmem, fordata1, fordata2, readdata1, readdata2, pcdata;
wire [  4:0]	stallwr;
wire [  3:0]	opcode;
wire [  2:0]	o_addr1_rf, o_addr2_rf, i_addr_rf;
wire [  1:0]	decision;
wire 		wr_rf, wr_dmem, carryout_alu, zeroout_alu, addornand, forward1, forward2, IFIDvalid, IFIDstall, IDRRstall, IDEXstall, EXMMstall, MMWBstall, stallf1, stallf2, stallpc, IFIDclear, IDRRclear, IDEXclear, EXMMclear, MMWBclear, nowr, branch;
reg  [  2:0]	IDraddr1, IDraddr2, IDwaddr;
reg  [  1:0]	extendsel, IDcbits, IDaluin, IDrfwrsel, EXflag, WBflag;
reg 		IDaluc, IDflagc, IDmemc, IDrfwr;

//*******************************************************************INSTRUCTION FETCH**************************************************************************//
//assign i_pc_rf = (IFIDstall) ? o_pc_rf : inc_o_pc_rf;
mux2 m2uut5(pcdata, o_pc_rf, IFIDstall, i_pc_rf);
increment incuut(o_pc_rf, inc_o_pc_rf);
imemory imuut(o_pc_rf, instruction);
regfile rfuut(clk, rst, wr_rf, o_addr1_rf, o_addr2_rf, i_addr_rf, o_data1_rf, o_data2_rf, i_data_rf, i_pc_rf, o_pc_rf);
//*****************************************************INSTRUCTION FETCH - INSTRUCTION DECODE********************************************************************//
assign IFIDvalid = 1'b1;
always @(posedge clk or posedge rst) begin
	if(rst)
		IFIDpipereg <= 0;
	else if(IFIDclear)
		IFIDpipereg <= {nowr, 50'b0};
	else if(IFIDstall)
		IFIDpipereg <= {nowr, stallwr[0], IFIDpipereg[48:0]};
	else
		IFIDpipereg <= {nowr, 1'b0, IFIDvalid, inc_o_pc_rf, o_pc_rf, instruction};
//				-nowr--stall---valid---------pc++------pc------instruction	
//				-[50]--[49]--[48]--------[47:32]----[31:16]------[15:0]---
end
//*******************************************************************INSTRUCTION DECODE**************************************************************************//
assign opcode = IFIDpipereg[15:12];
always @(*) begin
	IDcbits <= IFIDpipereg[1:0];
	case(opcode)
		`add: begin
			IDaluc <= 1;
			IDflagc <= 1;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[5:3];
			IDmemc <= 0;
			IDrfwr <= 1;
			IDaluin <= 2'b00;
			IDrfwrsel <= 2'b00;
			extendsel <= 2'b00;
			if (IDcbits == 2'b11)
				shiftsel <= 1;
			else
				shiftsel <= 0;
		end
		`adi: begin
			IDaluc <= 1;
			IDflagc <= 1;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[8:6];
			IDmemc <= 0;
			IDrfwr <= 1;
			IDaluin <= 2'b01;
			IDrfwrsel <= 2'b00;
			extendsel <= 2'b00;
			shiftsel <= 0;
		end
		`ndu: begin
			IDaluc <= 0;
			IDflagc <= 1;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[5:3];
			IDmemc <= 0;
			IDrfwr <= 1;
			IDaluin <= 2'b00;
			IDrfwrsel <= 2'b00;
			extendsel <= 2'b00;
			shiftsel <= 0;
		end
		`lhi: begin
			IDaluc <= 1;
			IDflagc <= 0;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[11:9];
			IDmemc <= 0;
			IDrfwr <= 1;
			IDaluin <= 2'b00;
			IDrfwrsel <= 2'b01;
			extendsel <= 2'b10;
			shiftsel <= 0;
		end
		`lw: begin
			IDaluc <= 1;
			IDflagc <= 0;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[11:9];
			IDmemc <= 0;
			IDrfwr <= 1;
			IDaluin <= 2'b11;
			IDrfwrsel <= 2'b10;
			extendsel <= 2'b00;
			shiftsel <= 0;
		end
		`sw: begin
			IDaluc <= 1;
			IDflagc <= 0;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[11:9];
			IDmemc <= 1;
			IDrfwr <= 0;
			IDaluin <= 2'b11;
			IDrfwrsel <= 2'b00;
			extendsel <= 2'b00;
			shiftsel <= 0;
		end
		`jal: begin
			IDaluc <= 1;
			IDflagc <= 0;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[5:3];
			IDmemc <= 0;
			IDrfwr <= 1;
			IDaluin <= 2'b11;
			IDrfwrsel <= 2'b11;
			extendsel <= 2'b01;
			shiftsel <= 0;
		end
		`jlr: begin
			IDaluc <= 1;
			IDflagc <= 0;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[11:9];
			IDmemc <= 0;
			IDrfwr <= 1;
			IDaluin <= 2'b11;
			IDrfwrsel <= 2'b11;
			extendsel <= 2'b01;
			shiftsel <= 0;
		end
		`beq: begin
			IDaluc <= 1;
			IDflagc <= 0;
			IDraddr1 <= IFIDpipereg[11:9];
			IDraddr2 <= IFIDpipereg[8:6];
			IDwaddr <= IFIDpipereg[11:9];
			IDmemc <= 0;
			IDrfwr <= 0;
			IDaluin <= 2'b11;
			IDrfwrsel <= 2'b11;
			extendsel <= 2'b00;
			shiftsel <= 0;
		end
		default: begin
			IDaluc <= 1'bz;
			IDflagc <= 1'bz;
			IDraddr1 <= 3'bz;
			IDraddr2 <= 3'bz;
			IDwaddr <= 3'bz;
			IDmemc <= 3'bz;
			IDrfwr <= 1'bz;
			IDaluin <= 2'bzz;
			IDrfwrsel <= 2'bzz;
			extendsel <= 2'bzz;
			shiftsel <= 1'bz;
		end
	endcase
end


assign o_addr1_rf = IFIDpipereg[11:9];
assign o_addr2_rf = IFIDpipereg[8:6];

mux2 m2uut3(o_data1_rf, fordata1, forward1, readdata1);
mux2 m2uut4(o_data2_rf, fordata2, forward2, readdata2);

signex6 s6uut(IFIDpipereg[5:0], extended6);
signex9 s9uut(IFIDpipereg[8:0], extended9);
zeropad zpuut(IFIDpipereg[8:0], zeropad);
//onebitshiftleft(o_data2_rf, shifted_o_data2_rf);
mux3 m3uut1(extended6, extended9, zeropad, extendsel, IDextended);
adder aduut(IDextended, IFIDpipereg[31:16], IDpc_p_ex);

//****************************************************************INSTRUCTION DECODE - EXECUTE************************************************************************//

always @(posedge clk or posedge rst) begin
	if(rst || IDEXclear)
		IDEXpipereg <= 0;
	else if(IDEXstall)
		IDEXpipereg <= {IDEXpipereg[122], IFIDpipereg[50], stallwr[1], IDEXpipereg[119:0]};
	else if(stallwr[1])
		IDEXpipereg <= {shiftsel, IFIDpipereg[50], stallwr[1], IFIDpipereg[48], opcode, IFIDpipereg[31:16], IDpc_p_ex, IDextended, IDrfwrsel, IDrfwr, IDaluin, IDaluc, IDflagc, IDcbits, IDmemc, readdata1, readdata2, IDraddr1, IDraddr2, IDwaddr, IFIDpipereg[47:32]};
	else
		IDEXpipereg <= {shiftsel, IFIDpipereg[50], IFIDpipereg[49], IFIDpipereg[48], opcode, IFIDpipereg[31:16], IDpc_p_ex, IDextended, IDrfwrsel, IDrfwr, IDaluin, IDaluc, IDflagc, IDcbits, IDmemc, readdata1, readdata2, IDraddr1, IDraddr2, IDwaddr, IFIDpipereg[47:32]};
//				---shiftsel-----nowr------------stall----------valid---------------opcode---------------pc----------------pc+ex------------------ex--------------rfwrdatasel---------rfwr---------------aluinp---------------aluop-----------flagwr------------last 2 bits--------memwr--------rdata1-------rdata2--------raddr1-----------raddr2--------------waddr-------------------pc++
//				-----[122]--------[121]------------[120]----------[119]--------------[118:115]----------[114:99]------------[98:83]---------------[82:67]-------------[66:65]-----------[64]---------------[63:62]--------------[61]-------------[60]---------------[59:58]----------[57]--------[56:41]-------[40:25]-------[24:22]----------[21:19]------------[18:16]---------------[15:0]
end

//****************************************************************************EXECUTE*****************************************************************************//

if(IDEXpipereg[122] == 1)
	shifted_input <= {IDEXpipereg[39:25], 1'b0};
else
	shifted_input <= IDEXpipereg[40:25];

mux2 m2uut1(IDEXpipereg[56:41], IDEXpipereg[40:25], IDEXpipereg[63], i_alu1);
mux2 m2uut2(shifted_input, IDEXpipereg[82:67], IDEXpipereg[62], i_alu2);
alu auut(i_alu1, i_alu2, o_alu, IDEXpipereg[61], carryout_alu, zeroout_alu);
always @(posedge clk or posedge rst) begin
	if(rst)
		EXflag <= 0;
	else if(IDEXpipereg[60])
		EXflag <= {carryout_alu, zeroout_alu};
end


//************************************************************************EXECUTE - MEMORY************************************************************************//

always @(posedge clk or posedge rst) begin
	if(rst || EXMMclear)
		EXMMpipereg <= 0;
	else if(EXMMstall)
		EXMMpipereg <= {IDEXpipereg[121], stallwr[2], EXMMpipereg[134:0]};
	else if(stallwr[2])
		EXMMpipereg <= {IDEXpipereg[121], stallwr[2], IDEXpipereg[119], IDEXpipereg[118:115], IDEXpipereg[114:99], IDEXpipereg[98:83], IDEXpipereg[82:67], IDEXpipereg[66:65], IDEXpipereg[64], IDEXpipereg[60], IDEXpipereg[59:58], IDEXpipereg[57], IDEXpipereg[56:41], IDEXpipereg[40:25], o_alu, EXflag, IDEXpipereg[24:22], IDEXpipereg[21:19], IDEXpipereg[18:16], IDEXpipereg[15:0]};
	else
		EXMMpipereg <= {IDEXpipereg[121], IDEXpipereg[120], IDEXpipereg[119], IDEXpipereg[118:115], IDEXpipereg[114:99], IDEXpipereg[98:83], IDEXpipereg[82:67], IDEXpipereg[66:65], IDEXpipereg[64], IDEXpipereg[60], IDEXpipereg[59:58], IDEXpipereg[57], IDEXpipereg[56:41], IDEXpipereg[40:25], o_alu, EXflag, IDEXpipereg[24:22], IDEXpipereg[21:19], IDEXpipereg[18:16], IDEXpipereg[15:0]};
//				-----nowr---------------stall------------valid-----------------opcode--------------pc----------------pc+ex------------------ex---------------rfwrdatasel---------rfwr--------------flagwr------------last 2 bits--------memwr--------------rdata1--------------rdata2---------ALUOUT--FLAG-------raddr1---------------raddr2-----------------waddr-----------------pc++
//				-----[136]-------------[135]---------------[134]----------------[133:130]--------[129:114]------------[113:98]--------------[97:82]---------------[81:80]----------[79]---------------[78]---------------[77:76]-----------[75]--------------[74:59]--------------[58:43]-----[42:27]--[26:25]----[24:22]---------------[21:19]---------------[18:16]---------------[15:0]
end

//*****************************************************************************MEMORY*****************************************************************************//

dmemory dmuut(clk, rst, wr_dmem, i_data_dmem, o_data_dmem, addr_dmem);
assign wr_dmem = (EXMMpipereg[135]) ? 1'b0 : (EXMMpipereg[75] & EXMMpipereg[134]);
assign addr_dmem = EXMMpipereg[42:27];
assign i_data_dmem = EXMMpipereg[74:59];

//************************************************************************MEMORY - WRITE BACK********************************************************************//

always @(posedge clk or posedge rst) begin
	if(rst || MMWBclear)
		MMWBpipereg <= 0;
	else if(MMWBstall)
		MMWBpipereg <= {EXMMpipereg[136], stallwr[3], MMWBpipereg[149:0]};
	else if(stallwr[3])	
		MMWBpipereg <= {EXMMpipereg[136], stallwr[3], EXMMpipereg[134], EXMMpipereg[133:130], EXMMpipereg[129:114], EXMMpipereg[113:98], EXMMpipereg[97:82], EXMMpipereg[81:80], EXMMpipereg[79], EXMMpipereg[78], EXMMpipereg[77:76], EXMMpipereg[74:59], EXMMpipereg[58:43], EXMMpipereg[42:27], EXMMpipereg[26:25], o_data_dmem, EXMMpipereg[24:22], EXMMpipereg[21:19], EXMMpipereg[18:16], EXMMpipereg[15:0]};
	else
		MMWBpipereg <= {EXMMpipereg[136], EXMMpipereg[135], EXMMpipereg[134], EXMMpipereg[133:130], EXMMpipereg[129:114], EXMMpipereg[113:98], EXMMpipereg[97:82], EXMMpipereg[81:80], EXMMpipereg[79], EXMMpipereg[78], EXMMpipereg[77:76], EXMMpipereg[74:59], EXMMpipereg[58:43], EXMMpipereg[42:27], EXMMpipereg[26:25], o_data_dmem, EXMMpipereg[24:22], EXMMpipereg[21:19], EXMMpipereg[18:16], EXMMpipereg[15:0]};
//				-----nowr-------------stall------------valid----------------opcode---------------pc----------------pc+ex--------------------ex---------------rfwrdatasel-----------rfwr-----------flagwr----------last 2 bits------------rdata1---------------rdata2-------------ALUOUT----------------FLAG------------memrddata-------raddr1---------------raddr2-----------------waddr-----------------pc++
//				-----[151]------------[150]------------[149]--------------[148:145]------------[144:129]----------[128:113]---------------[112:97]------------[96:95]--------------[94]------------[93]-------------[92:91]--------------[90:75]--------------[74:59]------------[58:43]--------------[42:41]-----------[40:25]--------[24:22]-------------[21:19]---------------[18:16]---------------[15:0]
end


//**************************************************************************WRITE BACK***************************************************************************//
mux4 m4uut1(MMWBpipereg[58:43], MMWBpipereg[112:97], MMWBpipereg[40:25], MMWBpipereg[15:0], MMWBpipereg[96:95], i_data_rf);
always @(posedge clk or posedge rst) begin
	if(rst)
		WBflag <= 0;
	else if(MMWBpipereg[93])
		EXflag <= MMWBpipereg[42:41];
end
assign addornand = (MMWBpipereg[148:145] == `add || MMWBpipereg[148:145] == `ndu);
assign wr_rf = (addornand & (MMWBpipereg[92:91] == 2'b00 || MMWBpipereg[92:91] == 2'b11) & ~MMWBpipereg[150] & ~MMWBpipereg[151]) ? MMWBpipereg[94] : 1'bz;
assign wr_rf = (addornand & MMWBpipereg[92:91] == 2'b01 & MMWBpipereg[41] == 1'b1 & ~MMWBpipereg[150] & ~MMWBpipereg[151]) ? MMWBpipereg[94] : 1'bz;
assign wr_rf = (addornand & MMWBpipereg[92:91] == 2'b10 & MMWBpipereg[42] == 1'b1 & ~MMWBpipereg[150] & ~MMWBpipereg[151]) ? MMWBpipereg[94] : 1'bz;
assign wr_rf = (~addornand & ~MMWBpipereg[150] & ~MMWBpipereg[151]) ? MMWBpipereg[94] : 1'bz;
assign wr_rf = (MMWBpipereg[150] || MMWBpipereg[151]) ? 1'b0 : 1'bz;

assign i_addr_rf = MMWBpipereg[18:16];


//**************************************************************************WRITE BACK REG***********************************************************************//

always @(posedge clk or posedge rst) begin
	if(rst)
		WBpipereg <= 0;
	else
		WBpipereg <= {i_data_rf, i_addr_rf};
//			       wr data-----wr addr----	
//				[18:3]------[2:0]-----
end


//**************************************************************************FORWARDING LOGIC**********************************************************************//

forwarding f1(RREXpipereg[119], EXMMpipereg[134], MMWBpipereg[149], RREXpipereg[18:16], EXMMpipereg[18:16], MMWBpipereg[18:16], IDRRpipereg[24:22], RREXpipereg[118:115], EXMMpipereg[133:130], MMWBpipereg[148:145], o_alu, RREXpipereg[82:67], EXMMpipereg[42:27], EXMMpipereg[97:82], o_data_dmem, MMWBpipereg[58:43], MMWBpipereg[112:97], MMWBpipereg[40:25], {carryout_alu, zeroout_alu}, RREXpipereg[59:58], EXMMpipereg[26:25], EXMMpipereg[77:76], MMWBpipereg[42:41], MMWBpipereg[92:91], forward1, fordata1, stallf1);
//-------------valid RREX----------valid EXMM-------valid MMWB---------wr addr RREX--------wr addr EXMM-------wr addr MMWB--------rd addr IDRR--------opcode RREX-----------opcode EXMM----------opcode MMWB---------aluoutEX--extended RREX------aluout EXMM---------extended EXMM-------memrd MM-------aluout MMWB---------extended MMWB---------memrddata MMWB--------flag EX-------------------last 2 bits RREX-----flag EXMM----------last 2 bits EXMM----flag MMWB-----------last 2 bits MMWB--
forwarding f2(RREXpipereg[119], EXMMpipereg[134], MMWBpipereg[149], RREXpipereg[18:16], EXMMpipereg[18:16], MMWBpipereg[18:16], IDRRpipereg[21:19], RREXpipereg[118:115], EXMMpipereg[133:130], MMWBpipereg[148:145], o_alu, RREXpipereg[82:67], EXMMpipereg[42:27], EXMMpipereg[97:82], o_data_dmem, MMWBpipereg[58:43], MMWBpipereg[112:97], MMWBpipereg[40:25], {carryout_alu, zeroout_alu}, RREXpipereg[59:58], EXMMpipereg[26:25], EXMMpipereg[77:76], MMWBpipereg[42:41], MMWBpipereg[92:91], forward2, fordata2, stallf2);


//*******************************************************FORWARDING LOGIC STALL FOR WHEN PREVIOUS INSTRUCTION IS LW***********************************************//

assign IFIDstall = (stallf1 || stallf2 || stallpc) ? 1'b1 : 1'b0;
assign IDEXstall = (stallf1 || stallf2) ? 1'b1 : 1'b0;
pencoder stallcycwr(IFIDstall, IDEXstall, EXMMstall, MMWBstall, stallwr);

//**************************************************************************WHEN WR ADDR IS PC**********************************************************************//

pcaddr newpc(branch, decision, IDRRpipereg[18:16], IDEXpipereg[18:16], EXMMpipereg[18:16], opcode, IDRRpipereg[86:83], IDEXpipereg[118:115], EXMMpipereg[133:130], inc_o_pc_rf, IDRRpipereg[15:0], IDextended, IDRRpipereg[50:35], IDpc_p_ex, IDRRpipereg[66:51], readdata2, o_alu, {carryout_alu, zeroout_alu}, IDEXpipereg[59:58], o_data_dmem, IFIDclear, IDRRclear, IDEXclear, EXMMclear, pcdata, stallpc, nowr);
//---------------------------------waddrIDRR----------waddrIDEX----------waddrEXMM--------opcodeID------opcodeIDRR------------opcodeIDEX----------opcodeEXMM-----------inc_pc-------IDRRpc++---------exID----------exIDRR-------------pcexID------pcexIDRR-------rddataIDEX--aluoutEX----------flagEX--------------condIDEX-------------memrdMM----flushIFID--flushIDRR--flushIDEX--flushEXMM--data----stall


//**************************************************************************BRANCH PREDICTION**********************************************************************//

branchpredict bp(clk, rst, opcode, IFIDpipereg[31:16], IDRRpipereg[86:83], readdata1, readdata2, branch, decision);

endmodule
