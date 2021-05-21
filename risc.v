module risc_top_module(input clk, input rst);
	
	reg  [50:0] 	IFIDpipereg;
	reg  [142:0]	IDEXpipereg;
	reg  [136:0]	EXMMpipereg;
	reg  [151:0]	MMWBpipereg;
	reg  [18:0]	WBpipereg;
	
	//**************************************I-fetch stage******************************//
	
	mux2 inst_m4(inc_o_pc, adder_out)
	mux4 inst_m1(branch_output_pc, adder_out, alu_out, rf_d2, pc_sel_mux, pcdata);
	mux2 inst_m2(pcdata, o_pc, IFIDstall, i_pc)
	incrementer inst_inc1(o_pc, inc_o_pc);
	imemory inst_imem1(o_pc, instr);
	register inst_pc(clk,rst,wr_pc,i_pc,o_pc);
	
	//*************************************IFIDpipe************************************//
	
	assign IFIDvalid = 1'b1;
	always @(posedge clk or posedge rst)
	begin
		if(rst)
			IFIDpipereg <= 0;
		else if(IFIDclear)
			IFIDpipereg <= {no_write, 34'b0};
		else if(IFIDstall)
			IFIDpipereg <= {no_write, stall_write[0], IFIDpipereg[32:0]};
		else
			IFIDpipereg <= {no_write, 1'b0, IFIDvalid, inc_o_pc, o_pc, instr};
	end				//nowr....stall...valid......pc+......pc....instruction//
					//[50].....[49]....[48]...[47,32]...[31,16]....[15,0]//
	//****************************I-decode stage*************************//
	assign opcode = IFIDpipereg[15:12];
	always @(*)
	begin
		cbits <= IFIDpipereg[1:0];
		case(opcode)
			0001 : begin
				if(cbits = 2'b00)				//add
				begin
					aluc <= 2'b01;
					flagc <= 1;
					rf_rd_addr1 <= IFIDpipereg[11:9];
					rf_rd_addr2 <= IFIDpipereg[8:6];
					rf_wr_addr3 <= IFIDpipereg[5:3];
					memc <= 0;
					wr_rf <= 1;
					rf_write_sel <= 2'b00;
					aluB_in_sel <= 2'b00;
					adderB_in_sel <= x;
					pc_sel_mux <= 2'b00;
				end
				else if(cbits = 2'b10)				//adc
				begin
					aluc <= 2'b01;
					flagc <= 1;
					rf_rd_addr1 <= IFIDpipereg[11:9];
					rf_rd_addr2 <= IFIDpipereg[8:6];
					rf_wr_addr3 <= IFIDpipereg[5:3];
					memc <= 0;
					wr_rf <= alu_cf;
					rf_write_sel <= 2'b00;
					aluB_in_sel <= 2'b00;
					adderB_in_sel <= 0;
					pc_sel_mux <= 2'b00;
				end
				else if(cbits = 2'b01)				//adz
				begin
					aluc <= 2'b01;
					flagc <= 1;
					rf_rd_addr1 <= IFIDpipereg[11:9];
					rf_rd_addr2 <= IFIDpipereg[8:6];
					rf_wr_addr3 <= IFIDpipereg[5:3];
					memc <= 0;
					wr_rf <= alu_zf;
					rf_write_sel <= 2'b00;
					aluB_in_sel <= 2'b00;
					adderB_in_sel <= 0;
					pc_sel_mux <= 2'b00;
				end
				else if(cbits = 2'b11)				//adl
				begin
					aluc <= 2'b01;
					flagc <= 1;
					rf_rd_addr1 <= IFIDpipereg[11:9];
					rf_rd_addr2 <= IFIDpipereg[8:6];
					rf_wr_addr3 <= IFIDpipereg[5:3];
					memc <= 0;
					wr_rf <= 1;
					rf_write_sel <= 2'b00;
					aluB_in_sel <= 2'b01;
					adderB_in_sel <= 0;
					pc_sel_mux <= 2'b00;
				end
			end
			0000 : begin						//adi
				aluc <= 2'b01;
				flagc <= 1;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_wr_addr3 <= IFIDpipereg[8:6];
				memc <= 0;
				wr_rf <= 1;
				rf_write_sel <= 2'b00;
				aluB_in_sel <= 2'b11;
				adderB_in_sel <= 0;
				pc_sel_mux <= 2'b00;
			end
			0010 : begin
				if(cbits = 00)					//ndu
				begin
					aluc <= 2'b10;
					flagc <= 1;
					rf_rd_addr1 <= IFIDpipereg[11:9];
					rf_rd_addr2 <= IFIDpipereg[8:6];
					rf_wr_addr3 <= IFIDpipereg[5:3];
					memc <= 0;
					wr_rf <= 1;
					rf_write_sel <= 2'b00;
					aluB_in_sel <= 2'b00;
					adderB_in_sel <= 0;
					pc_sel_mux <= 2'b00;
				end
				else if(cbits = 10)				//ndc
				begin
					aluc <= 2'b10;
					flagc <= 1;
					rf_rd_addr1 <= IFIDpipereg[11:9];
					rf_rd_addr2 <= IFIDpipereg[8:6];
					rf_wr_addr3 <= IFIDpipereg[5:3];
					memc <= 0;
					wr_rf <= alu_cf;
					rf_write_sel <= 2'b00;
					aluB_in_sel <= 2'b00;
					adderB_in_sel <= 0;
					pc_sel_mux <= 2'b00;
				end
				else if(cbits = 01)				//ndz
				begin
					aluc <= 2'b10;
					flagc <= 1;
					rf_rd_addr1 <= IFIDpipereg[11:9];
					rf_rd_addr2 <= IFIDpipereg[8:6];
					rf_wr_addr3 <= IFIDpipereg[5:3];
					memc <= 0;
					wr_rf <= alu_zf;
					rf_write_sel <= 2'b00;
					aluB_in_sel <= 2'b00;
					adderB_in_sel <= 0;
					pc_sel_mux <= 2'b00;
				end
			end
			0011 : begin						//lhi
				aluc <= 2'b00;
				flagc <= 0;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_wr_addr3 <= IFIDpipereg[11:9];
				memc <= 0;
				wr_rf <= 1;
				rf_write_sel <= 2'b01;
				aluB_in_sel <= 2'b00;
				adderB_in_sel <= 0;
				pc_sel_mux <= 2'b00;
			end
			0100 : begin						//lw
				aluc <= 2'b01;
				flagc <= 0;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_wr_addr3 <= IFIDpipereg[11:9];
				memw <= 0;
				wr_rf <= 1;
				rf_write_sel <= 2'b10;
				aluB_in_sel <= 2'b11;
				adderB_in_sel <= 0;
				pc_sel_mux <= 2'b00;
			end
			0101 : begin						//sw
				aluc <= 2'b01;
				flagc <= 0;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_wr_addr3 <= IFIDpipereg[11:9];
				memw <= 1;
				wr_rf <= 0;
				rf_write_sel <= 2'b00;
				aluB_in_sel <= 2'b11;
				adderB_in_sel <= 0;
				pc_sel_mux <= 2'b00;
			end
			1000 : begin						//beq
				aluc <= 2'b11;
				flagc <= 0;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_wr_addr3 <= IFIDpipereg[11:9];
				memw <= 0;
				wr_rf <= 0;
				rf_write_sel <= 2'b00;
				aluB_in_sel <= 2'b00;
				adderB_in_sel <= 1;
				pc_sel_mux <= 2'b00;
			end
			1001 : begin						//jal
				aluc <= 2'b00;
				flagc <= 0;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_rd_addr3 <= IFIDpipereg[5:3];
				memw <= 0;
				wr_rf <= 1;
				rf_write_sel <= 2'b11;
				aluB_in_sel <= 2'b00;
				adderB_in_sel <= 0;
				pc_sel_mux <= 2'b01;
			end
			1010 : begin						//jlr
				aluc <= 2'b00;
				flagc <= 0;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_rd_addr3 <= IFIDpipereg[11:9];
				memw <= 0;
				wr_rf <= 1;
				rf_write_sel <= 2'b11;
				aluB_in_sel <= 2'b00;
				adderB_in_sel <= 0;
				pc_sel_mux <= 2'b11;
			end
			1011 : begin						//jri
				aluc <= 2'b01;
				flagc <= 0;
				rf_rd_addr1 <= IFIDpipereg[11:9];
				rf_rd_addr2 <= IFIDpipereg[8:6];
				rf_rd_addr3 <= IFIDpipereg[5:3];
				memw <= 0;
				wr_rf <= 0;
				rf_write_sel <= 2'bxx;
				aluB_in_sel <= 2'b10;
				adderB_in_sel <= 0;
				pc_sel_mux <= 2'b10;
			end
			default : begin
				aluc <= 2'bz;
				flagc <= z;
				rf_rd_addr1 <= 3'bz;
				rf_rd_addr2 <= 3'bz;
				rf_rd_addr3 <= 3'bz;
				memw <= z;
				wr_rf <= z;
				rf_write_sel <= 2'bz;
				aluB_in_sel <= 2'bz;
				adderB_in_sel <= z;
				pc_sel_mux <= 2'bz;
			end
		endcase
	end

tenbitsignext(IFIDpipereg[5:0], signext6to16);			//10 bit shifter
sevenbitsignext(IFIDpipereg[8:0], signext9to16);		//7 bit shifter
onebitsll(r_data2_rf, shifted_r_data2_rf);

regfile inst_rf1(clk, rst, wr_rf, r_addr1_rf, r_addr2_rf, w_addr3_rf, r_data1_rf, r_data2_rf, w_data3_rf);

assign r_addr1_rf = IFIDpipereg[11:9];
assign r_addr2_rf = IFIDpipereg[8:6];

mux2 inst_m3(r_data1_rf, fordata1, forward1, readdata1);
mux2 inst_m4(r_data2_rf, fordata2, forward2, readdata2);

//************************************IDEXpipe*******************************************//

always @(posedge clk or posedge rst) begin
	if(rst || IDEXclear)
		IDEXpipereg <= 0;
	else if(IDEXstall)
		IDEXpipereg <= {IFIDpipereg[50], stallwr[1], IDEXpipereg[140:0]};
	else if(stallwr[1])
		IDEXpipereg <= {IFIDpipereg[50], stallwr[1], IFIDpipereg[48], IFIDpipereg[47:0], r_data1_rf, r_data2_rf, shifted_r_data2_rf, signext9to16, signext6to16, aluc, flagc, memw, wr_rf, rf_wr_sel, aluB_in_sel, adderB_in_sel, pc_sel_mux};
	else
		IDEXpipereg <= {IDRRpipereg[50], IDRRpipereg[49], IDRRpipereg[48],  IFIDpipereg[47:0],          r_data1_rf,     r_data2_rf,     shifted_r_data2_rf,        signext9to16,       signext6to16, aluc, flagc, memw, wr_rf, rf_wr_sel, aluB_in_sel, adderB_in_sel, pc_sel_mux};
				[142]                  [141]            [140]        [139:124][123:108][107:92]   [91:76]         [75:60]              [59:44]                  [43:28]               [27:12]         [11:10]       [9]      [8]      [7]       [6:5]              [4:3]             [2]          [1:0]
end

//********************************************Execute stage********************************//

adder(IFIDpipereg[139:124], adderB_in , adder_out);
mux2 inst_m5(IDEXpipe[43:28], IDEXpipe[27:12], adderB_in_sel, adderB_in);
alu(IDEXpipereg[91:76], aluB_in, alu_out, op, alu_cf, alu_zf);
mux4 inst_m6(IDEXpipe[75:60], IDEXpipe[59:44], IDEXpipe[43:28], IDEXpipe[27:12], aluB_in_sel, aluB_in);
zeropad(IDEXpipereg[100:92], zero_pad_out);
always @(posedge clk or posedge rst) begin
	if(rst)
		EXflag <= 0;
	else if(IDEXpipereg[60])
		EXflag <= {alu_cf, alu_zf};
end

//********************************************EXMMpipereg***********************************//

always @(posedge clk or posedge rst) begin
	if(rst || EXMMclear)
		EXMMpipereg <= 0;
	else if(EXMMstall)
		EXMMpipereg <= {IDEXpipereg[142], stallwr[2], EXMMpipereg[]};
	else if(stallwr[2])
		EXMMpipereg <= {IDEXpipereg[142], stallwr[2], IDEXpipereg[140], IDEXpipereg[139:0], adder_out, alu_out, zero_pad_out};
	else
		EXMMpipereg <= {IDEXpipereg[142], IDEXpipereg[141], IDEXpipereg[140], IDEXpipereg[139:0], adder_out, alu_out, zero_pad_out};
											[187:48]           [47:32]   [31:16]	[15:0]
end

//************************************************Memory access*****************************//

dmemory inst_dmem1(clk, rst, EXMMpipereg[56], dmem_w_data, dmem_r_data, dmem_addr);
assign dmem_wr_en = (EXMMpipereg[141]) ? 1'b0 : (EXMMpipereg[56] & EXMMpipereg[140]);
assign dmem_addr = EXMMpipereg[31:16];
assign dmem_w_data = EXMMpipereg[123:108];
