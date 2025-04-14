// riscvpipelined.sv

// RISC-V pipelined processor
// From Section 7.6 of Digital Design & Computer Architecture: RISC-V Edition
// 27 April 2020
// David_Harris@hmc.edu 
// Sarah.Harris@unlv.edu

// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)

// Pipelined implementation of RISC-V (RV32I)
// User-level Instruction Set Architecture V2.2 (May 7, 2017)
// Implements a subset of the base integer instructions:
//    lw, sw
//    add, sub, and, or, slt, 
//    addi, andi, ori, slti
//    beq
//    jal
// Exceptions, traps, and interrupts not implemented
// little-endian memory

// 31 32-bit registers x1-x31, x0 hardwired to 0
// R-Type instructions
//   add, sub, and, or, slt
//   INSTR rd, rs1, rs2
//   Instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// I-Type Instructions
//   lw, I-type ALU (addi, andi, ori, slti)
//   lw:         INSTR rd, imm(rs1)
//   I-type ALU: INSTR rd, rs1, imm (12-bit signed)
//   Instr[31:20] = imm[11:0]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// S-Type Instruction
//   sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
//   Instr[31:25] = imm[11:5] (offset[11:5])
//   Instr[24:20] = rs2 (src)
//   Instr[19:15] = rs1 (base)
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:0]  (offset[4:0])
//   Instr[6:0]   = opcode
// B-Type Instruction
//   beq rs1, rs2, imm (PCTarget = PC + (signed imm x 2))
//   Instr[31:25] = imm[12], imm[10:5]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:1], imm[11]
//   Instr[6:0]   = opcode
// J-Type Instruction
//   jal rd, imm  (signed imm is multiplied by 2 and added to PC, rd = PC+4)
//   Instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode

//   Instruction  opcode    funct3    funct7
//   add          0110011   000       0000000
//   sub          0110011   000       0100000
//   and          0110011   111       0000000
//   or           0110011   110       0000000
//   slt          0110011   010       0000000
//   addi         0010011   000       immediate
//   andi         0010011   111       immediate
//   ori          0010011   110       immediate
//   slti         0010011   010       immediate
//   beq          1100011   000       immediate
//   lw	          0000011   010       immediate
//   sw           0100011   010       immediate
//   jal          1101111   immediate immediate

module testbench();

   logic        clk;
   logic        reset;

   logic [31:0] WriteData, DataAdr;
   logic        MemWrite;

   // instantiate device to be tested
   top dut(clk, reset, WriteData, DataAdr, MemWrite);

   initial
     begin
	string memfilename;
        memfilename = {"../riscvtest/test_hw.memfile"};
	$readmemh(memfilename, dut.imem.RAM);
     end
   
   // initialize test
   initial
     begin
	reset <= 1; # 22; reset <= 0;
     end

   // generate clock to sequence tests
   always
     begin
	clk <= 1; # 5; clk <= 0; # 5;
     end

   // check results
   always @(negedge clk)
     begin
	if(MemWrite) begin
           if(DataAdr === 100 & WriteData === 25) begin
              $display("Simulation succeeded");
              $stop;
           end else if (DataAdr !== 96) begin
              $display("Simulation failed");
              //$stop;
           end
	end
     end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteDataM, DataAdrM, 
           output logic        MemWriteM);

   logic [31:0] 	       PCF, InstrF, ReadDataM;
   
   // instantiate processor and memories
   riscv rv32pipe (clk, reset, PCF, InstrF, MemWriteM, DataAdrM, 
		   WriteDataM, ReadDataM);
   imem imem (PCF, InstrF);
   dmem dmem (clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
   
endmodule

module riscv(input  logic        clk, reset,
             output logic [31:0] PCF,
             input logic [31:0]  InstrF,
             output logic 	 MemWriteM,
             output logic [31:0] ALUResultM, WriteDataM,
             input logic [31:0]  ReadDataM);

   logic [6:0] 			 opD;
   logic [2:0] 			 funct3D;
   logic 			 funct7b5D;
   logic [2:0] 			 ImmSrcD;
   logic 			 ZeroE;
   logic       NegE;
   logic       CarryE;
   logic 			 PCSrcE;
   logic [3:0] 			 ALUControlE;
   logic 			 ALUSrcE;
   logic 			 ResultSrcEb0;
   logic 			 RegWriteM;
   logic [1:0] 			 ResultSrcW;
   logic 			 RegWriteW;

   logic [1:0] 			 ForwardAE, ForwardBE;
   logic 			 StallF, StallD, FlushD, FlushE;

   logic [4:0] 			 Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;

   logic       AuipcSrcE;
   logic       JumpSrcE;
   logic [2:0] LoadSrcM;
   logic [1:0] StoreSrcM;
   
   controller c(clk, reset,
		opD, funct3D, funct7b5D, ImmSrcD,
		FlushE, ZeroE, NegE, CarryE, OvE, PCSrcE, ALUControlE, ALUSrcE, ResultSrcEb0, AuipcSrcE, JumpSrcE,
		MemWriteM, RegWriteM, LoadSrcM, StoreSrcM,
		RegWriteW, ResultSrcW);

   datapath dp(clk, reset,
               StallF, PCF, InstrF,
	       opD, funct3D, funct7b5D, StallD, FlushD, ImmSrcD,
	       FlushE, ForwardAE, ForwardBE, PCSrcE, ALUControlE, ALUSrcE, AuipcSrcE, JumpSrcE, ZeroE, NegE, CarryE, OvE,
               MemWriteM, WriteDataM, ALUResultM, ReadDataM, LoadSrcM, StoreSrcM,
               RegWriteW, ResultSrcW,
               Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW);

   hazard  hu(Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              PCSrcE, ResultSrcEb0, RegWriteM, RegWriteW,
              ForwardAE, ForwardBE, StallF, StallD, FlushD, FlushE);			 
endmodule

module controller(input  logic		 clk, reset,
                  // Decode stage control signals
                  input logic [6:0]  opD,
                  input logic [2:0]  funct3D,
                  input logic 	     funct7b5D,
                  output logic [2:0] ImmSrcD,
                  // Execute stage control signals
                  input logic 	     FlushE, 
                  input logic 	     ZeroE, 
                  input logic        NegE,
                  input logic        CarryE,
                  input logic        OvE,
                  output logic 	     PCSrcE, // for datapath and Hazard Unit
                  output logic [3:0] ALUControlE, 
                  output logic 	     ALUSrcE,
                  output logic 	     ResultSrcEb0, // for Hazard Unit
                  output logic       AuipcSrcE,
                  output logic       JumpSrcE,
                  // Memory stage control signals
                  output logic 	     MemWriteM,
                  output logic 	     RegWriteM, // for Hazard Unit
                  output logic [2:0] LoadSrcM,	  
                  output logic [1:0] StoreSrcM,
                  // Writeback stage control signals
                  output logic 	     RegWriteW, // for datapath and Hazard Unit
                  output logic [1:0] ResultSrcW);

   // pipelined control signals
   logic 			     RegWriteD, RegWriteE;
   logic [1:0] 			     ResultSrcD, ResultSrcE, ResultSrcM;
   logic 			     MemWriteD, MemWriteE;
   logic 			     JumpD, JumpE;
   logic 			     BranchD, BranchE;
   logic [1:0] 			     ALUOpD;
   logic [3:0] 			     ALUControlD;
   logic 			     ALUSrcD;
   logic [2:0]     funct3E, funct3M;
   logic           AuipcSrcD;
   logic           JumpSrcD;
   
   // Decode stage logic
   maindec md(opD, ResultSrcD, MemWriteD, BranchD,
              ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOpD, AuipcSrcD, JumpSrcD);
   aludec  ad(opD[5], funct3D, funct7b5D, ALUOpD, ALUControlD);
   
   // Execute stage pipeline control register and logic
   floprc #(16) controlregE(clk, reset, FlushE,
                            {RegWriteD, ResultSrcD, MemWriteD, JumpD, BranchD, ALUControlD, ALUSrcD, funct3D, AuipcSrcD, JumpSrcD},
                            {RegWriteE, ResultSrcE, MemWriteE, JumpE, BranchE, ALUControlE, ALUSrcE, funct3E, AuipcSrcE, JumpSrcE});

   always_comb begin
    if(JumpE) PCSrcE = JumpE;
    else if(BranchE) begin
      case(funct3E)
        3'b000: PCSrcE = ZeroE;
        3'b001: PCSrcE = ~ZeroE;
        3'b100: PCSrcE = NegE ^ OvE;
        3'b101: PCSrcE = ~(NegE ^ OvE);
        3'b110: PCSrcE = ~CarryE;
        3'b111: PCSrcE = CarryE;
      endcase
    end else PCSrcE = 1'b0;
   end

   assign ResultSrcEb0 = ResultSrcE[0];
   
   // Memory stage pipeline control register
   flopr #(7) controlregM(clk, reset,
                          {RegWriteE, ResultSrcE, MemWriteE, funct3E},
                          {RegWriteM, ResultSrcM, MemWriteM, funct3M});

   always_comb begin
    case(ResultSrcM)
      2'b01: case(funct3M)
        3'b000: LoadSrcM = 3'b000; //lb
        3'b001: LoadSrcM = 3'b001; //lh
        3'b010: LoadSrcM = 3'b100; //lw
        3'b100: LoadSrcM = 3'b010; //lbu
        3'b101: LoadSrcM = 3'b011; //lhu
      endcase
      default: LoadSrcM = 3'b100; //lw
    endcase

    if(MemWriteM) begin
      case(funct3M)
        3'b000: StoreSrcM = 2'b00; //sb
        3'b001: StoreSrcM = 2'b01; //sh
        3'b010: StoreSrcM = 2'b10; //sw
        default: StoreSrcM = 2'b10; //sw
      endcase
    end
   end
   
   // Writeback stage pipeline control register
   flopr #(3) controlregW(clk, reset,
                          {RegWriteM, ResultSrcM},
                          {RegWriteW, ResultSrcW});   
endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic 	    MemWrite,
               output logic 	    Branch, ALUSrc,
               output logic 	    RegWrite, Jump,
               output logic [2:0] ImmSrc,
               output logic [1:0] ALUOp,
               output logic       AuipcSrc,
               output logic       JumpSrc);

   logic [13:0] 		  controls;

   assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
           ResultSrc, Branch, ALUOp, Jump, AuipcSrc, JumpSrc} = controls;

   always_comb
     case(op)
       // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump_AuipcSrc_JumpSrc
       7'b0000011: controls = 14'b1_000_1_0_01_0_00_0_0_0; // lw
       7'b0100011: controls = 14'b0_001_1_1_00_0_00_0_0_0; // sw
       7'b0110011: controls = 14'b1_xxx_0_0_00_0_10_0_0_0; // R-type 
       7'b1100011: controls = 14'b0_010_0_0_00_1_01_0_0_0; // B-type
       7'b0010011: controls = 14'b1_000_1_0_00_0_10_0_0_0; // I-type ALU
       7'b1101111: controls = 14'b1_011_0_0_10_0_00_1_0_0; // jal
       7'b1100111: controls = 14'b1_000_1_0_10_0_00_1_0_1; // jalr
       7'b0110111: controls = 14'b1_100_1_0_00_0_11_0_0_0; // lui
       7'b0010111: controls = 14'b1_100_0_0_00_0_00_0_1_0; // auipc
       7'b0000000: controls = 14'b0_000_0_0_00_0_00_0_0_0; // need valid values at reset
       default:    controls = 14'bx_xxx_x_x_xx_x_xx_x_x_x; // non-implemented instruction
     endcase
endmodule

module aludec(input  logic       opb5,
              input logic [2:0]  funct3,
              input logic 	 funct7b5, 
              input logic [1:0]  ALUOp,
              output logic [3:0] ALUControl);

   logic 			 RtypeSub;
   assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

   always_comb
     case(ALUOp)
       2'b00:                ALUControl = 4'b0000; // addition
       2'b01:                ALUControl = 4'b0001; // subtraction
       2'b11:                ALUControl = 4'b1000; //lui
       default: case(funct3) // R-type or I-type ALU
                  3'b000:  if (RtypeSub) 
                    ALUControl = 4'b0001; // sub
                  else          
                    ALUControl = 4'b0000; // add, addi
                  3'b001:    ALUControl = 4'b0110; // sll, slli
                  3'b010:    ALUControl = 4'b0101; // slt, slti
                  3'b011:    ALUControl = 4'b1011; // sltu, sltiu
                  3'b100:    ALUControl = 4'b0100; // xor, xori
                  3'b101: if(funct7b5)
                      ALUControl = 4'b1001; // sra, srai
                    else
                      ALUControl = 4'b0111; // srl, srli
                  3'b110:    ALUControl = 4'b0011; // or, ori
                  3'b111:    ALUControl = 4'b0010; // and, andi
                  default:   ALUControl = 4'bxxxx; // ???
		endcase
     endcase
endmodule

module datapath(input logic clk, reset,
                // Fetch stage signals
                input logic 	    StallF,
                output logic [31:0] PCF,
                input logic [31:0]  InstrF,
                // Decode stage signals
                output logic [6:0]  opD,
                output logic [2:0]  funct3D, 
                output logic 	    funct7b5D,
                input logic 	    StallD, FlushD,
                input logic [2:0]   ImmSrcD,
                // Execute stage signals
                input logic 	    FlushE,
                input logic [1:0]   ForwardAE, ForwardBE,
                input logic 	    PCSrcE,
                input logic [3:0]   ALUControlE,
                input logic 	    ALUSrcE,
                input logic       AuipcSrcE,
                input logic       JumpSrcE,
                output logic 	    ZeroE,
                output logic      NegE,
                output logic      CarryE,
                output logic      OvE,
                // Memory stage signals
                input logic 	    MemWriteM, 
                output logic [31:0] WriteDataM, ALUResultM,
                input logic [31:0]  ReadDataM,
                input logic [2:0]  LoadSrcM,
                input logic [1:0]  StoreSrcM,
                // Writeback stage signals
                input logic 	    RegWriteW, 
                input logic [1:0]   ResultSrcW,
                // Hazard Unit signals 
                output logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E,
                output logic [4:0]  RdE, RdM, RdW);

   // Fetch stage signals
   logic [31:0] 		    PCNextF, PCPlus4F;
   // Decode stage signals
   logic [31:0] 		    InstrD;
   logic [31:0] 		    PCD, PCPlus4D;
   logic [31:0] 		    RD1D, RD2D;
   logic [31:0] 		    ImmExtD;
   logic [4:0] 			    RdD;
   // Execute stage signals
   logic [31:0] 		    RD1E, RD2E;
   logic [31:0] 		    PCE, ImmExtE;
   logic [31:0] 		    SrcAE, SrcBE;
   logic [31:0] 		    ALUResultE;
   logic [31:0] 		    WriteDataE;
   logic [31:0] 		    PCPlus4E;
   logic [31:0] 		    PCTargetE;
   logic [31:0]         ResultsE;
   logic [31:0]         PCImmE;
   // Memory stage signals
   logic [31:0] 		    PCPlus4M;
   logic [31:0]         LoadOutM;
   logic [31:0]         StoreInM;
   // Writeback stage signals
   logic [31:0] 		    ALUResultW;
   logic [31:0] 		    ReadDataW;
   logic [31:0] 		    PCPlus4W;
   logic [31:0] 		    ResultW;

   // Fetch stage pipeline register and logic
   mux2    #(32) pcmux(PCPlus4F, PCTargetE, PCSrcE, PCNextF);
   flopenr #(32) pcreg(clk, reset, ~StallF, PCNextF, PCF);
   adder         pcadd(PCF, 32'h4, PCPlus4F);

   // Decode stage pipeline register and logic
   flopenrc #(96) regD(clk, reset, FlushD, ~StallD, 
                       {InstrF, PCF, PCPlus4F},
                       {InstrD, PCD, PCPlus4D});
   assign opD       = InstrD[6:0];
   assign funct3D   = InstrD[14:12];
   assign funct7b5D = InstrD[30];
   assign Rs1D      = InstrD[19:15];
   assign Rs2D      = InstrD[24:20];
   assign RdD       = InstrD[11:7];
   
   regfile        rf(clk, RegWriteW, Rs1D, Rs2D, RdW, ResultW, RD1D, RD2D);
   extend         ext(InstrD[31:7], ImmSrcD, ImmExtD);
   
   // Execute stage pipeline register and logic
   floprc #(175) regE(clk, reset, FlushE, 
                      {RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D}, 
                      {RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E});
   
   mux3   #(32)  faemux(RD1E, ResultW, ALUResultM, ForwardAE, SrcAE);
   mux3   #(32)  fbemux(RD2E, ResultW, ALUResultM, ForwardBE, WriteDataE);
   mux2   #(32)  srcbmux(WriteDataE, ImmExtE, ALUSrcE, SrcBE);
   alu           alu(SrcAE, SrcBE, ALUControlE, ALUResultE, ZeroE, NegE, CarryE, OvE);
   adder         branchadd(ImmExtE, PCE, PCImmE);
   mux2   #(32)  jalmux(PCImmE, ALUResultE, JumpSrcE, PCTargetE);
   mux2   #(32)  auimux(ALUResultE, PCTargetE, AuipcSrcE, ResultsE);

   // Memory stage pipeline register
   flopr  #(101) regM(clk, reset, 
                      {ResultsE, WriteDataE, RdE, PCPlus4E},
                      {ALUResultM, StoreInM, RdM, PCPlus4M});

   //Loads
   load          ld(ALUResultM, ReadDataM, LoadSrcM, LoadOutM);
   store         str(ReadDataM, StoreInM, ALUResultM, StoreSrcM, WriteDataM);
   
   // Writeback stage pipeline register and logic
   flopr  #(101) regW(clk, reset, 
                      {ALUResultM, LoadOutM, RdM, PCPlus4M},
                      {ALUResultW, ReadDataW, RdW, PCPlus4W});

   mux3   #(32)  resultmux(ALUResultW, ReadDataW, PCPlus4W, ResultSrcW, ResultW);	
endmodule

module load (input  logic [31:0] ALUResult,
             input  logic [31:0] ReadData,
             input  logic [2:0]  LoadSrc,
             output logic [31:0] ld);
  
  always_comb
  case(LoadSrc)
    3'b000: case(ALUResult[1:0]) //lb
        2'b00: ld = {{24{ReadData[7]}}, ReadData[7:0]};    //Zero offset
        2'b01: ld = {{24{ReadData[15]}}, ReadData[15:8]};  //One offset
        2'b10: ld = {{24{ReadData[23]}}, ReadData[23:16]}; //Two offset
        2'b11: ld = {{24{ReadData[31]}}, ReadData[31:24]}; //Three offset
      endcase
    3'b001: case(ALUResult[1]) //lh
        1'b0: ld = {{16{ReadData[15]}}, ReadData[15:0]};
        1'b1: ld = {{16{ReadData[31]}}, ReadData[31:16]};
      endcase
    3'b010: case(ALUResult[1:0]) //lbu
        2'b00: ld = {24'b0, ReadData[7:0]};   //Zero offset
        2'b01: ld = {24'b0, ReadData[15:8]};  //One offset
        2'b10: ld = {24'b0, ReadData[23:16]}; //Two offset
        2'b11: ld = {24'b0, ReadData[31:24]}; //Three offset
      endcase
    3'b011: case(ALUResult[1]) //lhu
        1'b0: ld = {16'b0, ReadData[15:0]};
        1'b1: ld = {16'b0, ReadData[31:16]};
      endcase
    default: ld = ReadData; //lw
  endcase
endmodule

module store (input logic  [31:0] ReadData,
              input logic  [31:0] StrIn,
              input logic  [31:0] ALUResult,
              input logic  [1:0]  StoreSrc,
              output logic [31:0] WriteData);
  always_comb
  case(StoreSrc)
    2'b00: case(ALUResult[1:0])
        2'b00: WriteData = {ReadData[31:8], StrIn[7:0]};
        2'b01: WriteData = {ReadData[31:16], StrIn[7:0], ReadData[7:0]};
        2'b10: WriteData = {ReadData[31:24], StrIn[7:0], ReadData[15:0]};
        2'b11: WriteData = {StrIn[7:0], ReadData[23:0]};
      endcase
    2'b01: case(ALUResult[1])
        1'b0: WriteData = {ReadData[31:16], StrIn[15:0]};
        1'b1: WriteData = {StrIn[15:0], ReadData[15:0]};
      endcase
    default: WriteData = StrIn;
  endcase
endmodule

// Hazard Unit: forward, stall, and flush
module hazard(input  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              input logic 	 PCSrcE, ResultSrcEb0, 
              input logic 	 RegWriteM, RegWriteW,
              output logic [1:0] ForwardAE, ForwardBE,
              output logic 	 StallF, StallD, FlushD, FlushE);

   logic 			 lwStallD;
   
   // forwarding logic
   always_comb begin
      ForwardAE = 2'b00;
      ForwardBE = 2'b00;
      if (Rs1E != 5'b0)
	if      ((Rs1E == RdM) & RegWriteM) ForwardAE = 2'b10;
	else if ((Rs1E == RdW) & RegWriteW) ForwardAE = 2'b01;
      
      if (Rs2E != 5'b0)
	if      ((Rs2E == RdM) & RegWriteM) ForwardBE = 2'b10;
	else if ((Rs2E == RdW) & RegWriteW) ForwardBE = 2'b01;
   end
   
   // stalls and flushes
   assign lwStallD = ResultSrcEb0 & ((Rs1D == RdE) | (Rs2D == RdE));  
   assign StallD = lwStallD;
   assign StallF = lwStallD;
   assign FlushD = PCSrcE;
   assign FlushE = lwStallD | PCSrcE;
endmodule

module regfile(input  logic        clk, 
               input logic 	   we3, 
               input logic [ 4:0]  a1, a2, a3, 
               input logic [31:0]  wd3, 
               output logic [31:0] rd1, rd2);

   logic [31:0] 		   rf[31:0];

   // three ported register file
   // read two ports combinationally (A1/RD1, A2/RD2)
   // write third port on rising edge of clock (A3/WD3/WE3)
   // write occurs on falling edge of clock
   // register 0 hardwired to 0

   always_ff @(negedge clk)
     if (we3) rf[a3] <= wd3;	

   assign rd1 = (a1 != 0) ? rf[a1] : 0;
   assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

   assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input logic [2:0]   immsrc,
              output logic [31:0] immext);
   
   always_comb
     case(immsrc) 
       // I-type 
       3'b000:   immext = {{20{instr[31]}}, instr[31:20]};  
       // S-type (stores)
       3'b001:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
       // B-type (branches)
       3'b010:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
       // J-type (jal)
       3'b011:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};
       // U-type (lui)
       3'b100:  immext = {instr[31:12], 12'b0};
       default: immext = 32'bx; // undefined
     endcase             
endmodule

module flopr #(parameter WIDTH = 8)
   (input  logic             clk, reset,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
   (input  logic             clk, reset, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) q <= d;
endmodule

module flopenrc #(parameter WIDTH = 8)
   (input  logic             clk, reset, clear, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) 
       if (clear) q <= 0;
       else       q <= d;
endmodule

module floprc #(parameter WIDTH = 8)
   (input  logic clk,
    input logic 	     reset,
    input logic 	     clear,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       
       if (clear) q <= 0;
       else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, 
    input logic 	     s, 
    output logic [WIDTH-1:0] y);

   assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, d2,
    input logic [1:0] 	     s, 
    output logic [WIDTH-1:0] y);

   assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem (input  logic [31:0] a,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[8191:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   
endmodule // imem

module dmem (input  logic        clk, we,
	     input  logic [31:0] a, wd,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[8191:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   always_ff @(posedge clk)
     if (we) RAM[a[31:2]] <= wd;
   
endmodule // dmem

module alu(input  logic [31:0] a, b,
           input logic [3:0]   alucontrol,
           output logic [31:0] result,
           output logic        zero,
           output logic        neg,
           output logic        carry,
           output logic        ov);

   logic [31:0] 	 condinvb;
   logic [32:0]    sum;
   logic 		       isAddSub;       // true when is add or sub

   assign condinvb = alucontrol[0] ? ~b : b;
   assign sum = a + condinvb + alucontrol[0];
   assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                     ~alucontrol[1] &  alucontrol[0];

   always_comb
     case (alucontrol)
       4'b0000:  result = sum[31:0];              // add
       4'b0001:  result = sum[31:0];              // subtract
       4'b0010:  result = a & b;                  // and
       4'b0011:  result = a | b;                  // or
       4'b0100:  result = a ^ b;                  // xor, xori
       4'b0101:  result = sum[31] ^ ov;           // slt, slti
       4'b0110:  result = a << b[4:0];            // sll, slli
       4'b0111:  result = a >> b[4:0];            // srl, srli
       4'b1000:  result = b;                      // lui
       4'b1001:  result = $signed(a) >>> b[4:0];  // sra, srai
       4'b1011:  result = {{31{1'b0}}, ~carry};   // sltu, sltiu
       default: result = 32'bx;
     endcase

   logic Asign, Bsign;

   assign Asign = a[31];
   assign Bsign = b[31];

   assign zero = (result == 32'b0);
   assign neg = result[31];
   assign carry = sum[32];

   assign ov = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
   
endmodule

