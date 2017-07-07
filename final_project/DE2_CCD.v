// --------------------------------------------------------------------
// Copyright (c) 2005 by Terasic Technologies Inc. 
// --------------------------------------------------------------------
//
// Permission:
//
//   Terasic grants permission to use and modify this code for use
//   in synthesis for all Terasic Development Boards and Altera Development 
//   Kits made by Terasic.  Other use of this code, including the selling 
//   ,duplication, or modification of any portion is strictly prohibited.
//
// Disclaimer:
//
//   This VHDL/Verilog or C/C++ source code is intended as a design reference
//   which illustrates how these types of functions can be implemented.
//   It is the user's responsibility to verify their design for
//   consistency and functionality through the use of formal
//   verification methods.  Terasic provides no warranty regarding the use 
//   or functionality of this code.
//
// --------------------------------------------------------------------
//           
//                     Terasic Technologies Inc
//                     356 Fu-Shin E. Rd Sec. 1. JhuBei City,
//                     HsinChu County, Taiwan
//                     302
//
//                     web: http://www.terasic.com/
//                     email: support@terasic.com
//
// --------------------------------------------------------------------
//
// Major Functions:	DE2 CMOS Camera Demo
//
// --------------------------------------------------------------------
//
// Revision History :
// --------------------------------------------------------------------
//   Ver  :| Author            :| Mod. Date :| Changes Made:
//   V1.0 :| Johnny Chen       :| 06/01/06  :|      Initial Revision
//   V1.1 :| Johnny Chen       :| 06/02/06  :|      Modify Image Quality
//   V1.2 :| Johnny Chen       :| 06/03/22  :|      Change Pin Assignment For New Sensor
//   V1.3 :| Abhishek	       :| 11/25/10  :|      Virtual Paint Development
// --------------------------------------------------------------------

module DE2_CCD
	(
		////////////////////	Clock Input	 	////////////////////	 
		CLOCK_27,						//	27 MHz
		CLOCK_50,						//	50 MHz
		EXT_CLOCK,						//	External Clock
		////////////////////	Push Button		////////////////////
		KEY,							//	Pushbutton[3:0]
		////////////////////	DPDT Switch		////////////////////
		SW,								//	Toggle Switch[17:0]
		////////////////////	7-SEG Dispaly	////////////////////
		HEX0,							//	Seven Segment Digit 0
		HEX1,							//	Seven Segment Digit 1
		HEX2,							//	Seven Segment Digit 2
		HEX3,							//	Seven Segment Digit 3
		HEX4,							//	Seven Segment Digit 4
		HEX5,							//	Seven Segment Digit 5
		HEX6,							//	Seven Segment Digit 6
		HEX7,							//	Seven Segment Digit 7
		////////////////////////	LED		////////////////////////
		LEDG,							//	LED Green[8:0]
		LEDR,							//	LED Red[17:0]
		////////////////////////	UART	////////////////////////
		UART_TXD,						//	UART Transmitter
		UART_RXD,						//	UART Receiver
		////////////////////////	IRDA	////////////////////////
		IRDA_TXD,						//	IRDA Transmitter
		IRDA_RXD,						//	IRDA Receiver
		/////////////////////	SDRAM Interface		////////////////
		DRAM_DQ,						//	SDRAM Data bus 16 Bits
		DRAM_ADDR,						//	SDRAM Address bus 12 Bits
		DRAM_LDQM,						//	SDRAM Low-byte Data Mask 
		DRAM_UDQM,						//	SDRAM High-byte Data Mask
		DRAM_WE_N,						//	SDRAM Write Enable
		DRAM_CAS_N,						//	SDRAM Column Address Strobe
		DRAM_RAS_N,						//	SDRAM Row Address Strobe
		DRAM_CS_N,						//	SDRAM Chip Select
		DRAM_BA_0,						//	SDRAM Bank Address 0
		DRAM_BA_1,						//	SDRAM Bank Address 0
		DRAM_CLK,						//	SDRAM Clock
		DRAM_CKE,						//	SDRAM Clock Enable
		////////////////////	Flash Interface		////////////////
		FL_DQ,							//	FLASH Data bus 8 Bits
		FL_ADDR,						//	FLASH Address bus 22 Bits
		FL_WE_N,						//	FLASH Write Enable
		FL_RST_N,						//	FLASH Reset
		FL_OE_N,						//	FLASH Output Enable
		FL_CE_N,						//	FLASH Chip Enable
		////////////////////	SRAM Interface		////////////////
		SRAM_DQ,						//	SRAM Data bus 16 Bits
		SRAM_ADDR,						//	SRAM Address bus 18 Bits
		SRAM_UB_N,						//	SRAM High-byte Data Mask 
		SRAM_LB_N,						//	SRAM Low-byte Data Mask 
		SRAM_WE_N,						//	SRAM Write Enable
		SRAM_CE_N,						//	SRAM Chip Enable
		SRAM_OE_N,						//	SRAM Output Enable
		////////////////////	ISP1362 Interface	////////////////
		OTG_DATA,						//	ISP1362 Data bus 16 Bits
		OTG_ADDR,						//	ISP1362 Address 2 Bits
		OTG_CS_N,						//	ISP1362 Chip Select
		OTG_RD_N,						//	ISP1362 Write
		OTG_WR_N,						//	ISP1362 Read
		OTG_RST_N,						//	ISP1362 Reset
		OTG_FSPEED,						//	USB Full Speed,	0 = Enable, Z = Disable
		OTG_LSPEED,						//	USB Low Speed, 	0 = Enable, Z = Disable
		OTG_INT0,						//	ISP1362 Interrupt 0
		OTG_INT1,						//	ISP1362 Interrupt 1
		OTG_DREQ0,						//	ISP1362 DMA Request 0
		OTG_DREQ1,						//	ISP1362 DMA Request 1
		OTG_DACK0_N,					//	ISP1362 DMA Acknowledge 0
		OTG_DACK1_N,					//	ISP1362 DMA Acknowledge 1
		////////////////////	LCD Module 16X2		////////////////
		LCD_ON,							//	LCD Power ON/OFF
		LCD_BLON,						//	LCD Back Light ON/OFF
		LCD_RW,							//	LCD Read/Write Select, 0 = Write, 1 = Read
		LCD_EN,							//	LCD Enable
		LCD_RS,							//	LCD Command/Data Select, 0 = Command, 1 = Data
		LCD_DATA,						//	LCD Data bus 8 bits
		////////////////////	SD_Card Interface	////////////////
		SD_DAT,							//	SD Card Data
		SD_DAT3,						//	SD Card Data 3
		SD_CMD,							//	SD Card Command Signal
		SD_CLK,							//	SD Card Clock
		////////////////////	USB JTAG link	////////////////////
		TDI,  							// CPLD -> FPGA (data in)
		TCK,  							// CPLD -> FPGA (clk)
		TCS,  							// CPLD -> FPGA (CS)
	    TDO,  							// FPGA -> CPLD (data out)
		////////////////////	I2C		////////////////////////////
		I2C_SDAT,						//	I2C Data
		I2C_SCLK,						//	I2C Clock
		////////////////////	PS2		////////////////////////////
		PS2_DAT,						//	PS2 Data
		PS2_CLK,						//	PS2 Clock
		////////////////////	VGA		////////////////////////////
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK,						//	VGA BLANK
		VGA_SYNC,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,  						//	VGA Blue[9:0]
		////////////	Ethernet Interface	////////////////////////
		ENET_DATA,						//	DM9000A DATA bus 16Bits
		ENET_CMD,						//	DM9000A Command/Data Select, 0 = Command, 1 = Data
		ENET_CS_N,						//	DM9000A Chip Select
		ENET_WR_N,						//	DM9000A Write
		ENET_RD_N,						//	DM9000A Read
		ENET_RST_N,						//	DM9000A Reset
		ENET_INT,						//	DM9000A Interrupt
		ENET_CLK,						//	DM9000A Clock 25 MHz
		////////////////	Audio CODEC		////////////////////////
		AUD_ADCLRCK,					//	Audio CODEC ADC LR Clock
		AUD_ADCDAT,						//	Audio CODEC ADC Data
		AUD_DACLRCK,					//	Audio CODEC DAC LR Clock
		AUD_DACDAT,						//	Audio CODEC DAC Data
		AUD_BCLK,						//	Audio CODEC Bit-Stream Clock
		AUD_XCK,						//	Audio CODEC Chip Clock
		////////////////	TV Decoder		////////////////////////
		TD_DATA,    					//	TV Decoder Data bus 8 bits
		TD_HS,							//	TV Decoder H_SYNC
		TD_VS,							//	TV Decoder V_SYNC
		TD_RESET,						//	TV Decoder Reset
		////////////////////	GPIO	////////////////////////////
		GPIO_0,							//	GPIO Connection 0
		GPIO_1							//	GPIO Connection 1
	);

////////////////////////	Clock Input	 	////////////////////////
input			CLOCK_27;				//	27 MHz
input			CLOCK_50;				//	50 MHz
input			EXT_CLOCK;				//	External Clock
////////////////////////	Push Button		////////////////////////
input	[3:0]	KEY;					//	Pushbutton[3:0]
////////////////////////	DPDT Switch		////////////////////////
input	[17:0]	SW;						//	Toggle Switch[17:0]
////////////////////////	7-SEG Dispaly	////////////////////////
output	[6:0]	HEX0;					//	Seven Segment Digit 0
output	[6:0]	HEX1;					//	Seven Segment Digit 1
output	[6:0]	HEX2;					//	Seven Segment Digit 2
output	[6:0]	HEX3;					//	Seven Segment Digit 3
output	[6:0]	HEX4;					//	Seven Segment Digit 4
output	[6:0]	HEX5;					//	Seven Segment Digit 5
output	[6:0]	HEX6;					//	Seven Segment Digit 6
output	[6:0]	HEX7;					//	Seven Segment Digit 7
////////////////////////////	LED		////////////////////////////
output	[8:0]		LEDG;					//	LED Green[8:0]
output	[17:0]	LEDR;					//	LED Red[17:0]
////////////////////////////	UART	////////////////////////////
output		UART_TXD;				//	UART Transmitter
input			UART_RXD;				//	UART Receiver
////////////////////////////	IRDA	////////////////////////////
output			IRDA_TXD;				//	IRDA Transmitter
input			IRDA_RXD;				//	IRDA Receiver
///////////////////////		SDRAM Interface	////////////////////////
inout	[15:0]	DRAM_DQ;				//	SDRAM Data bus 16 Bits
output[11:0]	DRAM_ADDR;				//	SDRAM Address bus 12 Bits
output			DRAM_LDQM;				//	SDRAM Low-byte Data Mask 
output			DRAM_UDQM;				//	SDRAM High-byte Data Mask
output			DRAM_WE_N;				//	SDRAM Write Enable
output			DRAM_CAS_N;				//	SDRAM Column Address Strobe
output			DRAM_RAS_N;				//	SDRAM Row Address Strobe
output			DRAM_CS_N;				//	SDRAM Chip Select
output			DRAM_BA_0;				//	SDRAM Bank Address 0
output			DRAM_BA_1;				//	SDRAM Bank Address 0
output			DRAM_CLK;				//	SDRAM Clock
output			DRAM_CKE;				//	SDRAM Clock Enable
////////////////////////	Flash Interface	////////////////////////
inout	[7:0]		FL_DQ;					//	FLASH Data bus 8 Bits
output[21:0]	FL_ADDR;				//	FLASH Address bus 22 Bits
output			FL_WE_N;				//	FLASH Write Enable
output			FL_RST_N;				//	FLASH Reset
output			FL_OE_N;				//	FLASH Output Enable
output			FL_CE_N;				//	FLASH Chip Enable
////////////////////////	SRAM Interface	////////////////////////
inout	[15:0]	SRAM_DQ;				//	SRAM Data bus 16 Bits
output[17:0]	SRAM_ADDR;				//	SRAM Address bus 18 Bits
output			SRAM_UB_N;				//	SRAM High-byte Data Mask 
output			SRAM_LB_N;				//	SRAM Low-byte Data Mask 
output			SRAM_WE_N;				//	SRAM Write Enable
output			SRAM_CE_N;				//	SRAM Chip Enable
output			SRAM_OE_N;				//	SRAM Output Enable
////////////////////	ISP1362 Interface	////////////////////////
inout	[15:0]	OTG_DATA;				//	ISP1362 Data bus 16 Bits
output	[1:0]	OTG_ADDR;				//	ISP1362 Address 2 Bits
output			OTG_CS_N;				//	ISP1362 Chip Select
output			OTG_RD_N;				//	ISP1362 Write
output			OTG_WR_N;				//	ISP1362 Read
output			OTG_RST_N;				//	ISP1362 Reset
output			OTG_FSPEED;				//	USB Full Speed,	0 = Enable, Z = Disable
output			OTG_LSPEED;				//	USB Low Speed, 	0 = Enable, Z = Disable
input				OTG_INT0;				//	ISP1362 Interrupt 0
input				OTG_INT1;				//	ISP1362 Interrupt 1
input				OTG_DREQ0;				//	ISP1362 DMA Request 0
input				OTG_DREQ1;				//	ISP1362 DMA Request 1
output			OTG_DACK0_N;			//	ISP1362 DMA Acknowledge 0
output			OTG_DACK1_N;			//	ISP1362 DMA Acknowledge 1
////////////////////	LCD Module 16X2	////////////////////////////
inout	[7:0]	LCD_DATA;				//	LCD Data bus 8 bits
output			LCD_ON;					//	LCD Power ON/OFF
output			LCD_BLON;				//	LCD Back Light ON/OFF
output			LCD_RW;					//	LCD Read/Write Select, 0 = Write, 1 = Read
output			LCD_EN;					//	LCD Enable
output			LCD_RS;					//	LCD Command/Data Select, 0 = Command, 1 = Data
////////////////////	SD Card Interface	////////////////////////
inout			SD_DAT;					//	SD Card Data
inout			SD_DAT3;				//	SD Card Data 3
inout			SD_CMD;					//	SD Card Command Signal
output			SD_CLK;					//	SD Card Clock
////////////////////////	I2C		////////////////////////////////
inout			I2C_SDAT;				//	I2C Data
output		I2C_SCLK;				//	I2C Clock
////////////////////////	PS2		////////////////////////////////
input		 	PS2_DAT;				//	PS2 Data
input			PS2_CLK;				//	PS2 Clock
////////////////////	USB JTAG link	////////////////////////////
input  			TDI;					// CPLD -> FPGA (data in)
input  			TCK;					// CPLD -> FPGA (clk)
input  			TCS;					// CPLD -> FPGA (CS)
output 			TDO;					// FPGA -> CPLD (data out)
////////////////////////	VGA			////////////////////////////
output			VGA_CLK;   				//	VGA Clock
output			VGA_HS;					//	VGA H_SYNC
output			VGA_VS;					//	VGA V_SYNC
output			VGA_BLANK;				//	VGA BLANK
output			VGA_SYNC;				//	VGA SYNC
output	[9:0]	VGA_R;   				//	VGA Red[9:0]
output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
////////////////	Ethernet Interface	////////////////////////////
inout	[15:0]	ENET_DATA;				//	DM9000A DATA bus 16Bits
output			ENET_CMD;				//	DM9000A Command/Data Select, 0 = Command, 1 = Data
output			ENET_CS_N;				//	DM9000A Chip Select
output			ENET_WR_N;				//	DM9000A Write
output			ENET_RD_N;				//	DM9000A Read
output			ENET_RST_N;				//	DM9000A Reset
input				ENET_INT;				//	DM9000A Interrupt
output			ENET_CLK;				//	DM9000A Clock 25 MHz
////////////////////	Audio CODEC		////////////////////////////
inout			AUD_ADCLRCK;			//	Audio CODEC ADC LR Clock
input			AUD_ADCDAT;				//	Audio CODEC ADC Data
inout			AUD_DACLRCK;			//	Audio CODEC DAC LR Clock
output		AUD_DACDAT;				//	Audio CODEC DAC Data
inout			AUD_BCLK;				//	Audio CODEC Bit-Stream Clock
output		AUD_XCK;				//	Audio CODEC Chip Clock
////////////////////	TV Devoder		////////////////////////////
input	[7:0]	TD_DATA;    			//	TV Decoder Data bus 8 bits
input			TD_HS;					//	TV Decoder H_SYNC
input			TD_VS;					//	TV Decoder V_SYNC
output		TD_RESET;				//	TV Decoder Reset
////////////////////////	GPIO	////////////////////////////////
inout	[35:0]	GPIO_0;					//	GPIO Connection 0
inout	[35:0]	GPIO_1;					//	GPIO Connection 1

assign	LCD_ON		=	1'b1;
assign	LCD_BLON	=	1'b1;
assign	TD_RESET	=	1'b1;

//	All inout port turn to tri-state
assign	FL_DQ		=	8'hzz;
assign	SRAM_DQ		=	16'hzzzz;
assign	OTG_DATA	=	16'hzzzz;
assign	LCD_DATA	=	8'hzz;
assign	SD_DAT		=	1'bz;
assign	I2C_SDAT	=	1'bz;
assign	ENET_DATA	=	16'hzzzz;
assign	AUD_ADCLRCK	=	1'bz;
assign	AUD_DACLRCK	=	1'bz;
assign	AUD_BCLK	=	1'bz;

//	CCD
wire	[9:0]	CCD_DATA;
wire			CCD_SDAT;
wire			CCD_SCLK;
wire			CCD_FLASH;
wire			CCD_FVAL;
wire			CCD_LVAL;
wire			CCD_PIXCLK;
reg			CCD_MCLK;	//	CCD Master Clock

wire	[15:0]	Read_DATA1;
wire	[15:0]	Read_DATA2;
wire				VGA_CTRL_CLK;
wire				AUD_CTRL_CLK;
wire	[9:0]		mCCD_DATA;
wire				mCCD_DVAL;
wire				mCCD_DVAL_d;
wire	[10:0]	X_Cont;
wire	[10:0]	Y_Cont;
wire	[31:0]	Frame_Cont;
wire	[9:0]		mCCD_R;
wire	[9:0]		mCCD_G;
wire	[9:0]		mCCD_B;
wire				DLY_RST_0;
wire				DLY_RST_1;
wire				DLY_RST_2;
wire				Read;
reg	[9:0]		rCCD_DATA;
reg				rCCD_LVAL;
reg				rCCD_FVAL;
wire	[9:0]		sCCD_R;
wire	[9:0]		sCCD_G;
wire	[9:0]		sCCD_B;
wire				sCCD_DVAL;

//	For Sensor 1
assign	CCD_DATA[0]	=	GPIO_1[0];
assign	CCD_DATA[1]	=	GPIO_1[1];
assign	CCD_DATA[2]	=	GPIO_1[5];
assign	CCD_DATA[3]	=	GPIO_1[3];
assign	CCD_DATA[4]	=	GPIO_1[2];
assign	CCD_DATA[5]	=	GPIO_1[4];
assign	CCD_DATA[6]	=	GPIO_1[6];
assign	CCD_DATA[7]	=	GPIO_1[7];
assign	CCD_DATA[8]	=	GPIO_1[8];
assign	CCD_DATA[9]	=	GPIO_1[9];
assign	GPIO_1[11]	=	CCD_MCLK;
assign	GPIO_1[15]	=	CCD_SDAT;
assign	GPIO_1[14]	=	CCD_SCLK;
assign	CCD_FVAL	=	GPIO_1[13];
assign	CCD_LVAL	=	GPIO_1[12];
assign	CCD_PIXCLK	=	GPIO_1[10];

assign	LEDR[15:0]= now;

assign	LEDG		=	Y_Cont;
assign	VGA_CTRL_CLK=	CCD_MCLK;
assign	VGA_CLK		=	~CCD_MCLK;

/////////////////////////////////////////////
wire	RED_match, GREEN_match, BLUE_match;
wire	detect;
//VGA input RGB data
wire	[9:0]	VGA_data_iRed, VGA_data_iGreen, VGA_data_iBlue;//data from SDRAM
reg	[9:0]	VGA_iRed, VGA_iGreen, VGA_iBlue;//data into VGA
wire	[9:0]	X_addr, Y_addr;
reg	[21:0] Xaddr_sum, Yaddr_sum;
reg	[9:0]	Xaddr_center;
reg	[8:0]	Yaddr_center;
reg	[9:0]	counter;
//original data coming from SDRAM
assign	VGA_data_iRed = Read_DATA2[9:0];
assign	VGA_data_iGreen = {Read_DATA1[14:10],Read_DATA2[14:10]};
assign	VGA_data_iBlue	= Read_DATA1[9:0];

//intensity check
assign	RED_match=((VGA_data_iRed > 10'd220) && (VGA_data_iRed < 10'd225))? 1'b1:1'b0;
assign	GREEN_match=((VGA_data_iGreen > 10'd160) && (VGA_data_iGreen < 10'd170))? 1'b1:1'b0;
assign	BLUE_match=((VGA_data_iBlue > 10'd100) && (VGA_data_iBlue < 10'd120))? 1'b1:1'b0;

assign	detect = RED_match & GREEN_match & BLUE_match;
reg detect_out;
always@(posedge VGA_CTRL_CLK)
begin
	if(~KEY[0])
	begin
		detect_out <= detect;	
	end
	else
	begin
		detect_out <= ((detect_out-detect)-(detect_out-detect)>>>10)+detect; //low pass filter
	end	
end

/////////////////////////paddle/////////////////
reg    [9:0]paddle_x;
reg    [9:0]paddle_y;
always@(posedge VGA_CTRL_CLK)
begin
	if(~KEY[0])
	begin
		paddle_x <= 320;	
      paddle_y <=	Yaddr_center;	
	end
	else if  (Xaddr_center<160)
   begin paddle_x <= 0;paddle_y <=	Yaddr_center;end////
	else if  (150<Xaddr_center<470)
   begin paddle_x <= (Xaddr_center<<<1)-320;paddle_y <=	Yaddr_center;end////	
	else if (480<Xaddr_center)
   begin paddle_x <= 640;paddle_y <=	Yaddr_center;end////
   else
	begin
	begin paddle_x <= 320;paddle_y <=	Yaddr_center;	end					
	end	
end
/////////////////////////paddle//////////////////
reg [9:0] paddle_x_out;
reg [9:0] paddle_y_out;
reg [22:0] divider1;
reg [22:0] divider2;
always@(posedge VGA_CTRL_CLK)
begin
	if(~KEY[0])
	begin
		divider1<=0;
		paddle_x_out<=320;
	end
	else
	begin
		if(divider1==400000)
		begin
			
			if(paddle_x_out<paddle_l_display/2) paddle_x_out<=paddle_l_display/2;
			else if (paddle_x_out>640-paddle_l_display/2) paddle_x_out<=640-paddle_l_display/2;
			else 
			begin
				paddle_x_out <=paddle_x_out- (paddle_x_out>>>4)+ (paddle_x>>>4) ;
		   end
			divider1<=0;
		end
		else 
		   divider1<=divider1+1;
	end		
end

always@(posedge VGA_CTRL_CLK)
begin
	if(~KEY[0])
	begin
		divider2<=0;
		paddle_y_out<=479;
	end
	else
	begin
		if(divider2==1000000)
		begin
				paddle_y_out <=paddle_y_out- (paddle_y_out>>>4)+ (paddle_y>>>4) ;
			   divider2<=0;
		end
		else 
		   divider2<=divider2+1;
	end	
end

always@(posedge VGA_CTRL_CLK) begin

if(~KEY[0]) begin   //if KEY0 is pressed KEY0 becomes 0
	VGA_iRed <= 10'b0;
	VGA_iGreen <= 10'b0;
	VGA_iBlue <= 10'b0;
	counter <= 10'b0;
	Xaddr_sum <= 22'b0;
	Yaddr_sum <= 22'b0;
	Xaddr_center <= 10'b0;
	Yaddr_center <= 9'b0;
end

else begin
	//////////////////paddle	
	if((Y_addr>469)&&(Y_addr<475))
	begin
	   if((X_addr>paddle_x_out-51)&&(X_addr<paddle_x_out+51) )
		begin
			VGA_iRed<=10'h3ff; VGA_iGreen<=10'h3ff; VGA_iBlue<=10'h3ff;
		end
		else 
		begin
			VGA_iRed<=0; VGA_iGreen<=0; VGA_iBlue<=0;
		end
	end 
	////////////////////paddle

	else
	begin		
		if(Y_addr < 'd478)
		begin
			if (detect_out)
			begin
				Xaddr_sum <= Xaddr_sum + X_addr;	
				Yaddr_sum <= Yaddr_sum + Y_addr;
				counter <= counter + 1;			
			end
		end
		else if((X_addr <= 'd2) && (Y_addr == 'd478))
		begin //calculate the center pixel and draw the pixel
			Xaddr_center <= Xaddr_sum/counter;
			Yaddr_center <= Yaddr_sum/counter;
		end		
		else if(Y_addr == 'd478)
		begin
			Xaddr_sum <= 22'h0;
			Yaddr_sum <= 22'h0;
			counter <= 10'b0;
		end
	end
end
end // end always

always@(posedge CLOCK_50)	CCD_MCLK	<=	~CCD_MCLK;
always@(posedge CCD_PIXCLK)
begin
	rCCD_DATA	<=	CCD_DATA;
	rCCD_LVAL	<=	CCD_LVAL;
	rCCD_FVAL	<=	CCD_FVAL;
end

VGA_Controller			u1	(	//	Host Side
							.oRequest(Read),
							.iRed(VGA_R_in),
							.iGreen(VGA_G_in),
							.iBlue(VGA_B_in),
							.iCursor_RGB_EN({ (|Xaddr_center),3'b0}), //penup &&
							.iCursor_X(Xaddr_center),
							.iCursor_Y(Yaddr_center),
							.iCursor_R(10'd186),
							.iCursor_G(10'd85),
							.iCursor_B(10'd211),
							.H_Cont_out(X_addr),
							.V_Cont_out(Y_addr),
							//	VGA Side
							.oVGA_R(VGA_R),
							.oVGA_G(VGA_G),
							.oVGA_B(VGA_B),
							.oVGA_H_SYNC(VGA_HS),
							.oVGA_V_SYNC(VGA_VS),
							.oVGA_SYNC(VGA_SYNC),
							.oVGA_BLANK(VGA_BLANK),
							//	Control Signal
							.iCLK(VGA_CTRL_CLK),
							.iRST_N(DLY_RST_2)	);

Reset_Delay				u2	(	.iCLK(CLOCK_50),
							.iRST(KEY[0]),
							.oRST_0(DLY_RST_0),
							.oRST_1(DLY_RST_1),
							.oRST_2(DLY_RST_2)	);

CCD_Capture				u3	(	.oDATA(mCCD_DATA),
							.oDVAL(mCCD_DVAL),
							.oX_Cont(X_Cont),
							.oY_Cont(Y_Cont),
							.oFrame_Cont(Frame_Cont),
							.iDATA(rCCD_DATA),
							.iFVAL(rCCD_FVAL),
							.iLVAL(rCCD_LVAL),
							.iSTART(SW[17]),
							.iEND(!KEY[2]),
							.iCLK(CCD_PIXCLK),
							.iRST(DLY_RST_1)	);

RAW2RGB					u4	(	.oRed(mCCD_R),
							.oGreen(mCCD_G),
							.oBlue(mCCD_B),
							.oDVAL(mCCD_DVAL_d),
							.iX_Cont(X_Cont),
							.iY_Cont(Y_Cont),
							.iDATA(mCCD_DATA),
							.iDVAL(mCCD_DVAL),
							.iCLK(CCD_PIXCLK),
							.iRST(DLY_RST_1)	);

SEG7_LUT_8 				u5	(	.oSEG0(HEX0),.oSEG1(HEX1),
							.oSEG2(HEX2),.oSEG3(HEX3),							
							.iDIG(Frame_Cont) );

Sdram_Control_4Port	u6	(	//	HOST Side
						    .REF_CLK(CLOCK_50),
						    .RESET_N(1'b1),
							//	FIFO Write Side 1
						    .WR1_DATA(	{sCCD_G[9:5],
										 sCCD_B[9:0]}),
							.WR1(sCCD_DVAL),
							.WR1_ADDR(0),
							.WR1_MAX_ADDR(640*512),
							.WR1_LENGTH(9'h100),
							.WR1_LOAD(!DLY_RST_0),
							.WR1_CLK(CCD_PIXCLK),
							//	FIFO Write Side 2
						    .WR2_DATA(	{sCCD_G[4:0],
										 sCCD_R[9:0]}),
							.WR2(sCCD_DVAL),
							.WR2_ADDR(22'h100000),
							.WR2_MAX_ADDR(22'h100000+640*512),
							.WR2_LENGTH(9'h100),
							.WR2_LOAD(!DLY_RST_0),
							.WR2_CLK(CCD_PIXCLK),
							//	FIFO Read Side 1
						    .RD1_DATA(Read_DATA1),
				        	.RD1(Read),
				        	.RD1_ADDR(640*16),
							.RD1_MAX_ADDR(640*496),
							.RD1_LENGTH(9'h100),
				        	.RD1_LOAD(!DLY_RST_0),
							.RD1_CLK(VGA_CTRL_CLK),
							//	FIFO Read Side 2
						    .RD2_DATA(Read_DATA2),
				        	.RD2(Read),
				        	.RD2_ADDR(22'h100000+640*16),
							.RD2_MAX_ADDR(22'h100000+640*496),
							.RD2_LENGTH(9'h100),
				        	.RD2_LOAD(!DLY_RST_0),
							.RD2_CLK(VGA_CTRL_CLK),
							//	SDRAM Side
						    .SA(DRAM_ADDR),
						    .BA({DRAM_BA_1,DRAM_BA_0}),
						    .CS_N(DRAM_CS_N),
						    .CKE(DRAM_CKE),
						    .RAS_N(DRAM_RAS_N),
				            .CAS_N(DRAM_CAS_N),
				            .WE_N(DRAM_WE_N),
						    .DQ(DRAM_DQ),
				            .DQM({DRAM_UDQM,DRAM_LDQM}),
							.SDR_CLK(DRAM_CLK)	);

I2C_CCD_Config 		u7	(	//	Host Side
							.iCLK(CLOCK_50),
							.iRST_N(KEY[1]),
							.iExposure(SW[15:0]),
							//	I2C Side
							.I2C_SCLK(CCD_SCLK),
							.I2C_SDAT(CCD_SDAT)	);

Mirror_Col				u8	(	//	Input Side
							.iCCD_R(mCCD_R),
							.iCCD_G(mCCD_G),
							.iCCD_B(mCCD_B),
							.iCCD_DVAL(mCCD_DVAL_d),
							.iCCD_PIXCLK(CCD_PIXCLK),
							.iRST_N(DLY_RST_1),
							//	Output Side
							.oCCD_R(sCCD_R),
							.oCCD_G(sCCD_G),
							.oCCD_B(sCCD_B),
							.oCCD_DVAL(sCCD_DVAL));
/////////////////////////////////////////////////////////////////////////////////////////

wire [31:0]	mSEG7_DIG;
reg	 [31:0]	Cont;
wire [9:0]	mVGA_R;
wire [9:0]	mVGA_G;
wire [9:0]	mVGA_B;
wire [19:0]	mVGA_ADDR;			//video memory address
wire		DLY_RST;
assign	TD_RESET	=	1'b1;	//	Allow 27 MHz input
assign	TD_RESET	=	1'b1;	//	Allow 27 MHz
assign	AUD_ADCLRCK	=	AUD_DACLRCK;
assign	AUD_XCK		=	AUD_CTRL_CLK;


AUDIO_DAC_ADC 			audio	(	//	Audio Side
							.oAUD_BCK(AUD_BCLK),
							.oAUD_DATA(AUD_DACDAT),
							.oAUD_LRCK(AUD_DACLRCK),
							.oAUD_inL(audio_inL), // audio data from ADC 
							.oAUD_inR(audio_inR), // audio data from ADC 
							.iAUD_ADCDAT(AUD_ADCDAT),
							.iAUD_extL(audio_outL), // audio data to DAC
							.iAUD_extR(audio_outR), // audio data to DAC
							//	Control Signals
				         .iCLK_18_4(AUD_CTRL_CLK),
							.iRST_N(DLY_RST_0)
							);

VGA_Audio_PLL 		p1	(	.areset(~DLY_RST_0),.inclk0(CLOCK_27),.c1(AUD_CTRL_CLK)	);

// DLA state machine
assign reset = ~KEY[0];

//state names
reg [17:0]count;
reg low_freq_clk;
reg touch;

assign paddle_x_display= (paddle_x_out<paddle_l_display)?paddle_l_display:((paddle_x_out>640-paddle_l_display)?640-paddle_l_display:paddle_x_out);
assign paddle_y_display= (paddle_y_out>270)?470:((paddle_y_out<200)?400:paddle_y_out+200);
reg [2:0] ball_state;
wire [9:0]paddle_x_display;   // control x position
reg [18:0]slow_key1;
reg [18:0]slow_key2;
wire [9:0]ball_x, ball_y;
wire [2:0]i;// i,j is the number of the current brick
wire [2:0]j;
wire dx,dy;
reg [3:0]out_situation;
reg [3:0] situation;
reg [9:0] paddle_l_display;
wire [9:0] paddle_y_display;  // control y position
reg [3:0] score0,score1,score2; //display the score

match_the_brick match(VGA_CTRL_CLK, ball_x, ball_y,i,j);

parameter radius=3;

//////////////////////////// generate state_update clk //////////////////////////////
always@(negedge VGA_CTRL_CLK)
begin	
			if(count=={paddle_y_out+300,8'd0})
			begin
			count<=0;
			low_freq_clk<= !low_freq_clk;
			end
			
			else 
			begin
			count<=count+1;
			end
end
///////////////////////////////////////////////////////////////////////////////////

rgb rgb_control(reset,VGA_CTRL_CLK,X_addr,Y_addr,paddle_x_display,paddle_y_display,paddle_l_display,ball_x,ball_y,mVGA_R,mVGA_G,mVGA_B, 
				color[0][0],color[0][1],color[0][2],color[0][3],color[0][4],color[0][5],
				color[1][0],color[1][1],color[1][2],color[1][3],color[1][4],color[1][5],
				color[2][0],color[2][1],color[2][2],color[2][3],color[2][4],color[2][5],
				color[3][0],color[3][1],color[3][2],color[3][3],color[3][4],color[3][5]);

ball_position_control ball_position(reset, low_freq_clk, ball_state, ball_x, ball_y, dx,dy, paddle_x_display,paddle_y_display);

dxdy_control dxdy(low_freq_clk, reset, out_situation, dx,dy);




reg [2:0] m[3:0][5:0];
parameter value=7;
/////////////////////////////////////////control part//////////////////////////////////////////////
always@(posedge low_freq_clk)
begin
	if(reset)
		begin
			ball_state<=0;
			m[0][0]<=value;m[0][1]<=value;m[0][2]<=value;m[0][3]<=value;m[0][4]<=value;m[0][5]<=value;
			m[1][0]<=value;m[1][1]<=value;m[1][2]<=value;m[1][3]<=value;m[1][4]<=value;m[1][5]<=value;
			m[2][0]<=value;m[2][1]<=value;m[2][2]<=value;m[2][3]<=value;m[2][4]<=value;m[2][5]<=value;
			m[3][0]<=value;m[3][1]<=value;m[3][2]<=value;m[3][3]<=value;m[3][4]<=value;m[3][5]<=value;	
			paddle_l_display<=80;
		end
	else 
		begin	   
		   case(ball_state)
			   0:
				begin
					if(!KEY[1]) begin ball_state<=1; end
					else begin ball_state<=0; end
				end
				1:                   // moving left,up direction 
				begin
					if(ball_x==5&&ball_y==5) ball_state<=3;  // touch the left,right corner
					else if(ball_x==5)       ball_state<=2;  // touch the left side
					else if(ball_y==5)		 ball_state<=4;  // touch the up side 
					///////////////// touch the brick ///////////////
			   	else if ( m[i][j]&&(ball_y==60+radius||ball_y==100+radius||ball_y==140+radius||ball_y==180+radius)&& 
								 ((ball_x>=50-radius &&ball_x<=radius+100) ||     
								  (ball_x>=150-radius&&ball_x<=radius+200 )|| 
								  (ball_x>=250-radius&&ball_x<=radius+300 )|| 
								  (ball_x>=350-radius&&ball_x<=radius+400) ||     
								  (ball_x>=450-radius&&ball_x<=radius+500) ||     
								  (ball_x>=550-radius&&ball_x<=radius+600 ) ) 
								)               begin sound<=1; ball_state<=4; if(m[i][j])m[i][j]<=m[i][j]-1; end
					else if (  m[i][j]&&(ball_x==100+radius||ball_x==200+radius||ball_x==300+radius||ball_x==400+radius||ball_x==500+radius||ball_x==600+radius)&&
								 ((ball_y>=40-radius &&ball_y<=radius+60) ||     
								  (ball_y>=80-radius&&ball_y<=radius+100 )||
								  (ball_y>=120-radius&&ball_y<=radius+140 )||
								  (ball_y>=160-radius&&ball_y<=radius+180 ) )							 
								)               begin sound<=1;ball_state<=2; if(m[i][j])m[i][j]<=m[i][j]-1; end
					else 							 begin sound<=0;ball_state<=1; touch<=0; end
					        
			
				end
				2:							// moving right,up direction 
				begin
					if(ball_x==635&&ball_y==5) ball_state<=4;
					else if(ball_x==635)       ball_state<=1;
					else if(ball_y==5)		   ball_state<=3;
					///////////////// touch the brick ///////////////
					else if (m[i][j]&& (ball_y==60+radius||ball_y==100+radius||ball_y==140+radius||ball_y==180+radius)&& 
								 ((ball_x>=50-radius &&ball_x<=radius+100) ||     
								  (ball_x>=150-radius&&ball_x<=radius+200 )|| 
								  (ball_x>=250-radius&&ball_x<=radius+300 )|| 
								  (ball_x>=350-radius&&ball_x<=radius+400) ||     
								  (ball_x>=450-radius&&ball_x<=radius+500) ||     
								  (ball_x>=550-radius&&ball_x<=radius+600 ) ) 
								)               begin sound<=1;ball_state<=3; if(m[i][j])m[i][j]<=m[i][j]-1; end
					else if ( m[i][j]&& (ball_x==50-radius||ball_x==150-radius||ball_x==250-radius||ball_x==350-+radius||ball_x==450-+radius||ball_x==550-radius)&&
								 ((ball_y>=40-radius &&ball_y<=radius+60) ||     
								  (ball_y>=80-radius&&ball_y<=radius+100 )||
								  (ball_y>=120-radius&&ball_y<=radius+140 )||
								  (ball_y>=160-radius&&ball_y<=radius+180 ) )							 
								)               begin sound<=1;ball_state<=1; if(m[i][j])m[i][j]<=m[i][j]-1; end
	
					else 							  begin sound<=0;ball_state<=2; touch<=0; end
				end
				3:							// moving right,down direction 
				begin
					if(ball_x==635&&ball_y==475) ball_state<=1;
					else if(ball_x==635)         ball_state<=4;
					else if(ball_y==475)		     ball_state<=2;
					/////////////////// touch the brick ///////////////
					else if ( m[i][j]&&(ball_y==40-radius||ball_y==80-radius||ball_y==120-radius||ball_y==160-radius)&& 
								 ((ball_x>=50-radius &&ball_x<=radius+100) ||     
								  (ball_x>=150-radius&&ball_x<=radius+200 )|| 
								  (ball_x>=250-radius&&ball_x<=radius+300 )|| 
								  (ball_x>=350-radius&&ball_x<=radius+400) ||     
								  (ball_x>=450-radius&&ball_x<=radius+500) ||     
								  (ball_x>=550-radius&&ball_x<=radius+600 ) ) 
								)               begin sound<=1;ball_state<=2; if(m[i][j])m[i][j]<=m[i][j]-1; end
					else if ( m[i][j]&& (ball_x==50-radius||ball_x==150-radius||ball_x==250-radius||ball_x==350-+radius||ball_x==450-+radius||ball_x==550-radius)&&
								 ((ball_y>=40-radius &&ball_y<=radius+60) ||     
								  (ball_y>=80-radius&&ball_y<=radius+100 )||
								  (ball_y>=120-radius&&ball_y<=radius+140 )||
								  (ball_y>=160-radius&&ball_y<=radius+180 ) )							 
								)               begin sound<=1;ball_state<=4; if(m[i][j])m[i][j]<=m[i][j]-1; end
	             //////////////////touch the paddle////////////////////////
					else if (ball_y==paddle_y_display-10-radius && ball_x>=paddle_x_display-paddle_l_display && ball_x<=paddle_x_display+paddle_l_display)
													 begin ball_state<=2;  touch=1;end
					else if (ball_y==paddle_y_display-5-radius) ball_state<=5;
	
					else 							 begin sound<=0;ball_state<=3; touch<=0; end
				end
				4:
				begin					  // moving left,down direction 
					if(ball_x==5&&ball_y==475) ball_state<=2;
					else if(ball_x==5)         ball_state<=3;
					else if(ball_y==475)		   ball_state<=1;
					///////////////// touch the brick ///////////////
			   	else if ( m[i][j]&&(ball_y==40-radius||ball_y==80-radius||ball_y==120-radius||ball_y==160-radius)&& 
								 ((ball_x>=50-radius &&ball_x<=radius+100) ||     
								  (ball_x>=150-radius&&ball_x<=radius+200 )|| 
								  (ball_x>=250-radius&&ball_x<=radius+300 )|| 
								  (ball_x>=350-radius&&ball_x<=radius+400) ||     
								  (ball_x>=450-radius&&ball_x<=radius+500) ||     
								  (ball_x>=550-radius&&ball_x<=radius+600 ) ) 
								)               begin sound<=1;ball_state<=1; if(m[i][j])m[i][j]<=m[i][j]-1; end
					else if ( m[i][j]&& (ball_x==100+radius||ball_x==200+radius||ball_x==300+radius||ball_x==400+radius||ball_x==500+radius||ball_x==600+radius)&&
								 ((ball_y>=40-radius &&ball_y<=radius+60) ||     
								  (ball_y>=80-radius&&ball_y<=radius+100 )||
								  (ball_y>=120-radius&&ball_y<=radius+140 )||
								  (ball_y>=160-radius&&ball_y<=radius+180 ) )							 
								)               begin sound<=1;ball_state<=3; if(m[i][j])m[i][j]<=m[i][j]-1; end
					else if (ball_y==paddle_y_display-10-radius && ball_x>=paddle_x_display-paddle_l_display && ball_x<=paddle_x_display+paddle_l_display)
													 begin ball_state<=1; touch<=1; end
					else if (ball_y==paddle_y_display-5-radius) ball_state<=5;
					else 							 begin sound<=0; ball_state<=4; touch<=0; end
				end
				5:
				begin
					ball_state<=5;
				end
			endcase
		end
end

//////////////////////////////////////////////////////////////////
reg [9:0] color[3:0][5:0];
always@(posedge VGA_CTRL_CLK)
if(reset)
begin
color[0][0]<=10'h3ff;color[0][1]<=10'h3ff;color[0][2]<=10'h3ff;color[0][3]<=10'h3ff;color[0][4]<=10'h3ff;color[0][5]<=10'h3ff;
color[1][0]<=10'h3ff;color[1][1]<=10'h3ff;color[1][2]<=10'h3ff;color[1][3]<=10'h3ff;color[1][4]<=10'h3ff;color[1][5]<=10'h3ff;
color[2][0]<=10'h3ff;color[2][1]<=10'h3ff;color[2][2]<=10'h3ff;color[2][3]<=10'h3ff;color[2][4]<=10'h3ff;color[2][5]<=10'h3ff;
color[3][0]<=10'h3ff;color[3][1]<=10'h3ff;color[3][2]<=10'h3ff;color[3][3]<=10'h3ff;color[3][4]<=10'h3ff;color[3][5]<=10'h3ff;
end
else
begin
	if(m[i][j]==0) 	  color[i][j]<=10'h000;
	else if(m[i][j]==1) color[i][j]<=10'h030;
	else if(m[i][j]==2) color[i][j]<=10'h08f;
	else if(m[i][j]==3) color[i][j]<=10'h0af;
	else if(m[i][j]==4) color[i][j]<=10'h0ff;
	else if(m[i][j]==5) color[i][j]<=10'h1ff;
	else if(m[i][j]==6) color[i][j]<=10'h2ff;
	else                color[i][j]<=10'h3ff;
end	


/// audio stuff /////////////////////////////////////////////////
// output to audio DAC
wire signed [15:0] audio_outL, audio_outR ;

reg signed [15:0] now,pp,p;
reg [20:0] count1;
reg CLK_10001;
assign audio_outL= now;
assign audio_outR= now;

reg [17:0] freq;
reg [17:0] ct;
reg sound;
reg [2:0] state_sound;

always@(m[i][j])
begin
	case(m[i][j])
	6: freq<=12288;              // do
	5: freq<=11129;              // rai
	4: freq<=10080;
	3: freq<=9129;
	2: freq<=8269;
	1: freq<=7489;
	0: freq<=6783;
	default: freq<=freq;
   endcase
end

always@ (posedge 	AUD_CTRL_CLK)    // finally this is the frequency of updating 
begin 
	if(count1==freq)
	begin
		CLK_10001 <= !CLK_10001;
		count1 <=0;
	end
	else
	begin
		count1 <= count1 +1;
	end
end


always @(posedge CLK_10001)	
if(reset)
begin	
	now<=16'd0;
	p<=16'd3276;
	pp<=16'd0;
	state_sound<=0;
   ct<=0;
end

else 
   case(state_sound)
	0:
	begin
	   if(sound==1) state_sound<=1;
		else state_sound<=0;
	end
	1:
	begin
		pp<= p;
		p <= now;
		now<= -pp+(pp>>>4);
		if(ct==80) 
			begin
				state_sound<=2;
				ct<=0;
			end
		else 
			begin
				ct<=ct+1; state_sound<=1; 
			end	
	end
   2:
	begin
		if(sound==1) state_sound<=2;
		else 
			begin
				state_sound<=0;
				now<= 16'd0;
				p<= 16'd3276;
				pp<=16'd0;
			end
	    end
	endcase
	
/////////////////display the number of times the ball hits the brick////////////////////////
always@(posedge sound or posedge reset)
begin
	if(reset) 
		begin
			score0<=0;score1<=0;score2<=0;
		end
	else
		begin
			if(score0==9&&score1==9)
				begin score0<=0; score1<=0;score2<=score1+1; end
			else if(score0==9)
				begin score0<=0;score1<=score1+1; end
			else score0<=score0+1;
		end
end

assign       HEX4=7'b1111111;
SEG7_LUT display0(HEX6,score0);
SEG7_LUT display1(HEX7,score1);
SEG7_LUT display2(HEX5,score2);

//////////////////////////display the image when the game is over////////////////////////////
wire [9:0]VGA_R_in,VGA_G_in,VGA_B_in;
assign VGA_R_in= (ball_state==5)? m4k_to_vga:mVGA_R;
assign VGA_G_in= (ball_state==5)? m4k_to_vga:mVGA_G;
assign VGA_B_in= (ball_state==5)? m4k_to_vga:mVGA_B;						
wire [9:0]m4k_to_vga;
wire [18:0] m4k_address;
assign m4k_address = 640*Y_addr + X_addr;
m4k m4k_block(       //generated by megawizard, store the image showing that "the game is over"
	.address(m4k_address),
	.clock(VGA_CTRL_CLK),
	.wren(0),
	.q(m4k_out));
	
assign m4k_to_vga= ((m4k_out==1)?10'h3ff:0);

endmodule //top module

