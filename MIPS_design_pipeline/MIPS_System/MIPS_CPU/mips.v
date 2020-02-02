`timescale 1ns/1ps
`define mydelay 1

//--------------------------------------------------------------
// mips.v
// David_Harris@hmc.edu and Sarah_Harris@hmc.edu 23 October 2005
// Single-cycle MIPS processor
//--------------------------------------------------------------

// single-cycle MIPS processor
module mips(input         clk, reset,
            output [31:0] pc,
            input  [31:0] instr,
            output        memwrite,
            output [31:0] memaddr,
            output [31:0] memwritedata,
            input  [31:0] memreaddata);

  wire        signext, shiftl16, memtoreg, branch;
  wire        pcsrc, zero;
  wire        alusrc, regdst, regwrite, jump;
  wire [3:0]  alucontrol;

  // ###### Jaemin Kim: Start ######
  wire [31:0] ID_instr;
  wire [4:0] EX_rs, EX_rt, MEM_writereg, WB_writereg; // forwarding
  wire [1:0] fmux1c, fmux2c, fmux3c, fmux4c;  // forwarding
  wire MEM_regwrite; // forwarding
  wire EX_memread;  // hazard detecting
  wire [4:0] ID_rs, ID_rt, MEM_rt; // hazard detecting
  wire pc_c, IF_ID_c; // hazard detecting
  wire jumpreg; // control hazard detecting
  // ###### Jaemin Kim: End ######

  // Instantiate Controller
  controller c(
    // ###### Jaemin Kim: Start ######
    .clk        (clk),
    .reset      (reset),
    .op         (ID_instr[31:26]), 
		.funct      (ID_instr[5:0]),
    // ###### Jaemin Kim: End ###### 
		.zero       (zero),
		.signext    (signext),
		.shiftl16   (shiftl16),
		.memtoreg   (memtoreg),
		.memwrite   (memwrite),
		.pcsrc      (pcsrc),
		.alusrc     (alusrc),
		.regdst     (regdst),
		.regwrite   (regwrite),
		.jump       (jump),
    // ###### Jaemin Kim: Start ######
    .memread    (memread),
    .MEM_regwrite (MEM_regwrite), // forwarding
    .mux_c       (mux_c), // hazard detecting
    .EX_memread (EX_memread), // hazard detecting
    // ###### Jaemin Kim: End ######
		.alucontrol (alucontrol));

  // Instantiate Datapath
  datapath dp(
    .clk        (clk),
    .reset      (reset),
    .signext    (signext),
    .shiftl16   (shiftl16),
    .memtoreg   (memtoreg),
    .pcsrc      (pcsrc),
    .alusrc     (alusrc),
    .regdst     (regdst),
    .regwrite   (regwrite),
    .jump       (jump),
    .alucontrol (alucontrol),
    .zero       (zero),
    .pc         (pc),
    .instr      (instr),
    .aluout     (memaddr), 
    .writedata  (memwritedata),
    .readdata   (memreaddata),
    // ###### Jaemin Kim: Start ######
    .fmux1c     (fmux1c),   // forwarding
    .fmux2c     (fmux2c),   // forwarding
    .fmux3c     (fmux3c),   // forwarding
    .fmux4c     (fmux4c),   // forwarding
    .EX_rs      (EX_rs),    // forwarding
    .EX_rt      (EX_rt),    // forwarding
    .MEM_writereg (MEM_writereg),   // forwarding
    .WB_writereg  (WB_writereg),    // forwarding 
    .ID_instr   (ID_instr),
    .ID_rs      (ID_rs),    // hazard detecting
    .ID_rt      (ID_rt),    // hazard detecting
    .MEM_rt     (MEM_rt),   // hazard detecting
    .pc_c       (pc_c),     // hazard detecting
    .IF_ID_c    (IF_ID_c),   // hazard detecting
    .jumpreg    (jumpreg)   // control hazard detecting
    );
    // ###### Jaemin Kim: End ######

    // ###### Jaemin Kim: Start ######
    forwarding fu(
      .ID_rs    (ID_rs),
      .ID_rt    (ID_rt),
      .EX_rs    (EX_rs),
      .EX_rt    (EX_rt),
      .MEM_writereg (MEM_writereg),
      .WB_writereg  (WB_writereg),
      .MEM_regwrite (MEM_regwrite),
      .WB_regwrite  (regwrite),
      .mux1c       (fmux1c),
      .mux2c       (fmux2c),
      .mux3c       (fmux3c),
      .mux4c       (fmux4c)
    );

    hazard_detection hdu(
      .ID_rs    (ID_rs),
      .ID_rt    (ID_rt),
      .EX_rt    (EX_rt),
      .MEM_rt   (MEM_rt),
      .EX_memread (EX_memread),
      .MEM_memread (memread),
      .jump     (jump), // control hazard
      .pcsrc   (pcsrc), // control hazard
      .jumpreg (jumpreg), // control hazard
      .mux_c    (mux_c),
      .pc_c     (pc_c),
      .IF_ID_c  (IF_ID_c)
    );
    // ###### Jaemin Kim End ######
endmodule

module controller(input  [5:0] op, funct,
                  input        zero,
                  // ###### Jaemin Kim: Start ######
                  input  clk, reset,
                  input   mux_c, // hazard detecting
                  output  memread, // hazard detecting
                  output  EX_memread, // hazard detecting
                  output  MEM_regwrite, // forwarding
                  // ###### Jaemin Kim: End ######
                  output       signext,
                  output       shiftl16,
                  output       memtoreg, memwrite,
                  output       pcsrc, alusrc,
                  output       regdst, regwrite,
                  output       jump,
                  output [3:0] alucontrol);

  wire [1:0] aluop;
  wire       branch;

  // ###### Jaemin Kim: Start ######

  wire [17:0] ID_EX_C_input;
  wire op0;

  wire tmp_memtoreg;
  wire tmp_memwrite;
  wire tmp_branch;
  wire tmp_alusrc;
  wire tmp_memread;
  wire tmp_regdst;
  wire tmp_regwrite;
  wire tmp_shiftl16;
  wire tmp_jump;
  wire [1:0] tmp_aluop;

  wire EX_regdst;
  wire EX_alusrc;
  wire EX_memwrite;
  wire EX_regwrite;
  wire EX_memtoreg;
  wire [5:0] EX_funct;

  wire MEM_memwrite;
  wire MEM_memtoreg;

  assign tmp_memread = op == 6'b100011 ? 1 : 0;

  // ###### Jaemin Kim: End ######

  maindec md(
    .op       (op),
    .signext  (signext),
    // ###### Jaemin Kim: Start ######
    .shiftl16 (tmp_shiftl16),
    .memtoreg (tmp_memtoreg),
    .memwrite (tmp_memwrite),
    .branch   (tmp_branch),
    .alusrc   (tmp_alusrc),
    .regdst   (tmp_regdst),
    .regwrite (tmp_regwrite),
    .jump     (tmp_jump),
    .aluop    (tmp_aluop));
    // ###### Jaemin Kim: End ###### 

  aludec ad(
    // ###### Jaemin Kim: Start ###### 
    .funct      (EX_funct),
    .aluop      (aluop),
    // ###### Jaemin Kim: End ###### 
    .alucontrol (alucontrol));

  // ###### Jaemin Kim: Start ######
  //milestone3 & 5
  assign pcsrc = (op0 ? branch & ~zero : branch & zero);
  // ###### Jaemin Kim: End ######

  // ###### Jaemin Kim: Start ######
  // hazard detecting
  // At milestone5, I added jump and op[0] to flipflop.
  mux2 #(18) hazard_mux(
    .d0({tmp_regdst, tmp_aluop, tmp_alusrc, tmp_shiftl16, tmp_branch, tmp_memread, tmp_memwrite, tmp_regwrite, tmp_memtoreg, tmp_jump, funct, op[0]}),
    .d1(18'b0),
    .s(mux_c),
    .y(ID_EX_C_input)
  );

  flopr #(18) ID_EX_C(
    .clk  (clk),
    .reset  (reset),
    .d    (ID_EX_C_input),
    .q    ({regdst, aluop, alusrc, shiftl16, branch, EX_memread, EX_memwrite, EX_regwrite, EX_memtoreg, jump, EX_funct, op0})
  );

  flopr #(4) EX_MEM_C(
    .clk  (clk),
    .reset  (reset),
    .d    ({EX_memread, EX_memwrite, EX_regwrite, EX_memtoreg}),
    .q    ({memread, memwrite, MEM_regwrite, MEM_memtoreg})
  );

  flopr #(2) MEM_WB_C(
    .clk    (clk),
    .reset  (reset),
    .d      ({MEM_regwrite, MEM_memtoreg}),
    .q      ({regwrite, memtoreg})
  );

// ###### Jaemin Kim: End ######

endmodule


module maindec(input  [5:0] op,
               output       signext,
               output       shiftl16,
               output       memtoreg, memwrite,
               output       branch, alusrc,
               output       regdst, regwrite,
               output       jump,
               output [1:0] aluop);

  reg [10:0] controls;

  assign {signext, shiftl16, regwrite, regdst, alusrc, branch, memwrite,
          memtoreg, jump, aluop} = controls;

  always @(*)
    case(op)
      6'b000000: controls <= #`mydelay 11'b00110000011; // Rtype
      6'b100011: controls <= #`mydelay 11'b10101001000; // LW
      6'b101011: controls <= #`mydelay 11'b10001010000; // SW
      6'b000100: controls <= #`mydelay 11'b10000100001; // BEQ
      6'b000101: controls <= #`mydelay 11'b10000100001; // BNE* milestone3
      6'b001000, 
      6'b001001: controls <= #`mydelay 11'b10101000000; // ADDI, ADDIU: only difference is exception
      6'b001010: controls <= #`mydelay 11'b10101000001; // SLTI*
      6'b001101: controls <= #`mydelay 11'b00101000010; // ORI
      6'b001111: controls <= #`mydelay 11'b01101000000; // LUI
      6'b000010: controls <= #`mydelay 11'b00000000100; // J
      6'b000011: controls <= #`mydelay 11'b00100000100; // JAL* milestone3
      default:   controls <= #`mydelay 11'bxxxxxxxxxxx; // ???
    endcase

endmodule

module aludec(input      [5:0] funct,
              input      [1:0] aluop,
              output reg [3:0] alucontrol);

  always @(*)
    case(aluop)
      2'b00: alucontrol <= #`mydelay 4'b0100;  // add
      2'b01: alucontrol <= #`mydelay 4'b1100;  // sub
      2'b10: alucontrol <= #`mydelay 4'b0010;  // or
      default: case(funct)          // RTYPE
          6'b100000,
          6'b100001,
          6'b001000: alucontrol <= #`mydelay 4'b0100; // ADD, ADDU: only difference is exception, jr* milestone3
          6'b100010,
          6'b100011: alucontrol <= #`mydelay 4'b1100; // SUB, SUBU: only difference is exception
          6'b100100: alucontrol <= #`mydelay 4'b0000; // AND
          6'b100101: alucontrol <= #`mydelay 4'b0010; // OR
          6'b101010: alucontrol <= #`mydelay 4'b1110; // SLT
          6'b101011: alucontrol <= #`mydelay 4'b1111; // SLTU* milestone3
          default:   alucontrol <= #`mydelay 4'bxxxx; // ???
        endcase
    endcase
    
endmodule

module datapath(input         clk, reset,
                input         signext,
                input         shiftl16,
                input         memtoreg, pcsrc,
                input         alusrc, regdst,
                input         regwrite, jump,
                input  [3:0]  alucontrol,
                output        zero,
                output [31:0] pc,
                input  [31:0] instr,
                output [31:0] aluout, writedata,
                // ###### Jaemin Kim: Start ######
                input [1:0] fmux1c, fmux2c, fmux3c, fmux4c,//forwarding
                output [31:0] ID_instr, //forwarding
                output [4:0] EX_rs, EX_rt, MEM_writereg, WB_writereg, //forwarding
                input pc_c, IF_ID_c,  // hazard detecting
                output [4:0] ID_rs, ID_rt, MEM_rt,// hazard detecting
                output jumpreg, // control hazard detecting
                // ###### Jaemin Kim: End ######
                input  [31:0] readdata);

  wire [4:0]  writereg;
  wire [31:0] pcnext, pcnexttmp, pcnextbr, pcplus4, pcbranch;
  wire [31:0] signimm, signimmsh, shiftedimm;
  wire [31:0] srca, srcb;
  wire [31:0] result;
  wire        shift;
  //milestone3

  // ###### Jaemin Kim: Start ######

  wire [31:0] ID_pcplus4;
  wire [31:0] ID_srca;
  wire [31:0] ID_srcb;
  wire [31:0] ID_writedata;

  wire [31:0] EX_pcplus4;
  wire [31:0] EX_srca;
  wire [31:0] EX_writedata;
  wire [31:0] EX_srcb;
  wire [31:0] EX_shiftedimm;
  wire [5:0] EX_funct;
  wire [31:0] EX_aluout;
  wire [31:0] EX_tmp_aluout;
  wire [31:0] EX_signimm;
  wire [4:0] EX_rd;
  wire [31:0] EX_instr;

  wire [31:0] MEM_pcbranch;
  wire [31:0] MEM_srcb;

  wire [31:0] WB_readdata;
  wire [31:0] WB_aluout;

  wire [31:0] f_EX_srca; // forwarding
  wire [31:0] f_EX_writedata; // forwarding
  wire [31:0] f_ID_srca; // forwarding
  wire [31:0] f_ID_writedata; // forwarding

  // ###### Jaemin Kim: End ######

  // next PC logic
  // ###### Jaemin Kim: Start ######
  flopenr #(32) pcreg(
    .clk   (clk),
    .reset (reset),
    .en    (pc_c), // data, control hazard detecting
    // ###### Jaemin Kim: End ######
    .d     (pcnext),
    .q     (pc));

  adder pcadd1(
    .a (pc),
    .b (32'b100),
    .y (pcplus4));

  sl2 immsh(
    .a (EX_signimm),
    .y (signimmsh));
				 
  adder pcadd2(
    // ###### Jaemin Kim: Start ######
    .a (EX_pcplus4),
    .b (signimmsh),
    // ###### Jaemin Kim: End ######
    .y (pcbranch));

  mux2 #(32) pcbrmux(
    .d0  (pcplus4),
    .d1  (pcbranch),
    .s   (pcsrc),
    .y   (pcnextbr));

  mux2 #(32) pcmux(
    .d0   (pcnextbr),
    // ###### Jaemin Kim: Start ###### control hazard detecting
    .d1   ({EX_pcplus4[31:28], EX_instr[25:0], 2'b00}),
    // ###### Jaemin Kim: End ######
    .s    (jump),
    .y    (pcnexttmp));

// ###### Jaemin Kim: Start ######
  //milestone3
  //Is it jr?
  mux2 #(32) pcmux2(
    .d0   (pcnexttmp),
    .d1   (EX_srca),
    .s    (jumpreg),
    .y    (pcnext));
// ###### Jaemin Kim: End ######


  // register file logic
  regfile rf(
    .clk     (clk),
    .we      (regwrite),
    // ###### Jaemin Kim: Start ######
    .ra1     (ID_instr[25:21]),
    .ra2     (ID_instr[20:16]),
    .wa      (WB_writereg),
    // ###### Jaemin Kim: End ######
    .wd      (result),
    // ###### Jaemin Kim: Start ######
    .rd1     (ID_srca),
    .rd2     (ID_writedata));
    // ###### Jaemin Kim: End ######

  //milestone3
  //If jal, register destination is $ra(=$31)
  // ###### Jaemin Kim: Start ######
  mux3 #(5) wrmux(
    .d0 (EX_rt),
    .d1 (EX_rd),
  // ###### Jaemin Kim: End ######
    .d2 (5'b11111),
    .s  ({jump, regdst}),
    .y  (writereg));


  mux2 #(32) resmux(  
    .d0 (WB_aluout),
    .d1 (WB_readdata),
    .s  (memtoreg),
    .y  (result));

  sign_zero_ext sze(
    // ###### Jaemin Kim: Start ######
    .a       (ID_instr[15:0]),
    // ###### Jaemin Kim: End ######
    .signext (signext),
    .y       (signimm[31:0]));

  shift_left_16 sl16(
    // ###### Jaemin Kim: Start ######
    .a         (EX_signimm[31:0]),
    .shiftl16  (shiftl16),
    .y         (shiftedimm[31:0]));
    // ###### Jaemin Kim: End ######

  // ALU logic
  //mux2 #(32) srcbmux(
  //  .d0 (writedata),
  //  .d1 (shiftedimm[31:0]),
  //  .s  (alusrc),
  //  .y  (srcb));

  //milestone3
  //if jal, pcplus4 must be stored in $ra
  mux3 #(32) srcbmux(
    // ###### Jaemin Kim: Start ######
    .d0 (f_EX_writedata),
    .d1 (shiftedimm[31:0]),
    .d2 (EX_pcplus4),
    .s  ({jump, alusrc}),
    .y  (EX_srcb));
    // ###### Jaemin Kim: End ######

  // ###### Jaemin Kim: Start ######
  // muxes for forwarding
  mux3 #(32) forwarding_mux1(
    .d0 (EX_srca),
    .d1 (result),
    .d2 (aluout),
    .s  (fmux1c),
    .y  (f_EX_srca)
  );

  mux3 #(32) forwarding_mux2(
    .d0 (EX_writedata),
    .d1 (result),
    .d2 (aluout),
    .s  (fmux2c),
    .y  (f_EX_writedata)
  );

  mux3 #(32) forwarding_mux3(
    .d0 (ID_srca),
    .d1 (result),
    .d2 (aluout),
    .s  (fmux3c),
    .y  (f_ID_srca)
  );

  mux3 #(32) forwarding_mux4(
    .d0 (ID_writedata),
    .d1 (result),
    .d2 (aluout),
    .s  (fmux4c),
    .y  (f_ID_writedata)
  );

  // ###### Jaemin Kim: End ######

  alu alu(
    // ###### Jaemin Kim: Start ######
    .a       (f_EX_srca),
    .b       (EX_srcb),
    // ###### Jaemin Kim: End ######
    .alucont (alucontrol),
    // ###### Jaemin Kim: Start ######
    .result  (EX_tmp_aluout), // for slti
    .zero    (zero));
    // ###### Jaemin Kim: End ######

  // for slti
  slti_module slti(
    .op     (EX_instr[31:26]),
    .aluout (EX_tmp_aluout),
    .out    (EX_aluout)
  );

  // ###### Jaemin Kim: Start ######
  //milestone3
  //Is it jr? If it is jr, jumpreg = 1
  isjr jr(
    .op   (EX_instr[31:26]),
    .funct (EX_instr[5:0]),
    .d (jumpreg)
  );
  // ###### Jaemin Kim: End ######

  // ###### Jaemin Kim: Start ######
  assign ID_rs = ID_instr[25:21]; // hazard detecting
  assign ID_rt = ID_instr[20:16]; // hazard detecting

  flopenr #(64) IF_ID(
    .clk  (clk),
    .reset  (reset|pcsrc|jump), // control hazard detecting. If jump or branch, IF_ID members become all 0.
    .en (IF_ID_c), //stalling
    .d  ({pcplus4, instr}),
    .q  ({ID_pcplus4, ID_instr})
  );

  flopr #(180) ID_EX(
    .clk  (clk),
    .reset  (reset),
    .d  ({ID_pcplus4, f_ID_srca, f_ID_writedata, signimm, ID_instr[25:21], ID_instr[20:16], ID_instr[15:11], ID_instr}),
    .q  ({EX_pcplus4, EX_srca, EX_writedata, EX_signimm, EX_rs, EX_rt, EX_rd, EX_instr}) // for using instruction on EX stage
  );

  flopr #(133) EX_MEM(
    .clk    (clk),
    .reset  (reset),
    .d      ({pcbranch, EX_aluout, f_EX_writedata, writereg, EX_rt}),
    .q      ({MEM_pcbranch, aluout, writedata, MEM_writereg, MEM_rt})
  );

  flopr #(69) MEM_WB(
    .clk    (clk),
    .reset  (reset),
    .d      ({readdata, aluout, MEM_writereg}),
    .q      ({WB_readdata, WB_aluout, WB_writereg})
  );
  // ###### Jaemin Kim: End ######
    
endmodule

// ###### Jaemin Kim: Start ######
module forwarding(
  input [4:0] ID_rs, ID_rt, EX_rs, EX_rt, MEM_writereg, WB_writereg,
  input MEM_regwrite, WB_regwrite,
  output reg [1:0] mux1c, mux2c, mux3c, mux4c
);

  always @(*)
  begin
    if (MEM_regwrite == 1'b1 && MEM_writereg == EX_rs && EX_rs != 0) mux1c = 2'b10;
    else if (WB_regwrite == 1'b1 && WB_writereg == EX_rs && EX_rs != 0) mux1c = 2'b01;
    else mux1c = 2'b00;

    if (MEM_regwrite == 1'b1 && MEM_writereg == EX_rt && EX_rt != 0) mux2c = 2'b10;
    else if (WB_regwrite == 1'b1 && WB_writereg == EX_rt && EX_rt != 0) mux2c = 2'b01;
    else mux2c = 2'b00;

    if (MEM_regwrite == 1'b1 && MEM_writereg == ID_rs && ID_rs != 0) mux3c = 2'b10;
    else if (WB_regwrite == 1'b1 && WB_writereg == ID_rs && ID_rs != 0) mux3c = 2'b01;
    else mux3c = 2'b00;

    if (MEM_regwrite == 1'b1 && MEM_writereg == ID_rt && ID_rt != 0) mux4c = 2'b10;
    else if (WB_regwrite == 1'b1 && WB_writereg == ID_rt && ID_rt != 0) mux4c = 2'b01;
    else mux4c = 2'b00;
  end

endmodule

module hazard_detection(
  input [4:0] MEM_rt, EX_rt, ID_rs, ID_rt,
  input EX_memread, MEM_memread, jump, pcsrc, jumpreg,
  output reg mux_c, pc_c, IF_ID_c
);

  always @(*)
  begin
    if(EX_memread == 1'b1 & (EX_rt == ID_rs | EX_rt == ID_rt))
    begin
      mux_c = 1'b1;
      pc_c = 1'b0;
      IF_ID_c = 1'b0;
    end
    else if(MEM_memread == 1'b1 & (MEM_rt == ID_rs | MEM_rt == ID_rt))
    begin
      mux_c = 1'b1;
      pc_c = 1'b0;
      IF_ID_c = 1'b0;
    end
    // milestion5, if branch or jump, stall but pcreg don't stop at this milestone5.
    else if(pcsrc | jump | jumpreg)
    begin
      mux_c = 1'b1;
      pc_c = 1'b1;
      IF_ID_c = 1'b0;
    end
    else
    begin
      mux_c = 1'b0;
      pc_c = 1'b1;
      IF_ID_c = 1'b1;
    end
  end

endmodule

// ###### Jaemin Kim: End ######