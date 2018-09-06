/*
 *  LED Controller
 *
 *  Copyright (C) 2018, Xiaohai Li (haixiaolee@gmail.com), All Rights Reserved
 *  This program is lincensed under Apache-2.0/GPLv3 dual-license
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  Apache-2.0 License and GNU General Public License for more details.
 *
 *  You may obtain a copy of Apache-2.0 License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 * 
 *  And GPLv3 License at
 *
 *  http://www.gnu.org/licenses
 *
 */

module led_ctrl(
// Global Clock and Reset
clk_sys_i,
rst_n_i,

// Memory Mapped Bus Slave Interface
mm_s_addr_i,
mm_s_wdata_i,
mm_s_rdata_o,
mm_s_we_i,

// Blink Control Clocks
clk_16hz_i,
clk_8hz_i,
clk_1hz_i,

// LED Signals
led_ctrl_o
//led_debug_o
);

/***************************************************
 LED Controller Parameters
***************************************************/
parameter MM_ADDR_WIDTH = 8;
parameter MM_DATA_WIDTH = 16;
// S3: LED Controller
parameter REG_ADDR_LED_CTRL = 'h0E;

//LED Blink Speed Definition
parameter BLINK_STOP = 2'b00;
parameter BLINK_SLOW = 2'b01;
parameter BLINK_MID  = 2'b10;
parameter BLINK_FAST = 2'b11;

//---------- Global Clock and Reset -----------
input clk_sys_i, rst_n_i;

//---------- Master Interface Signals -----------
// Address and Data Bus
input[MM_ADDR_WIDTH-1:0] mm_s_addr_i;
input[MM_DATA_WIDTH-1:0] mm_s_wdata_i;
output[MM_DATA_WIDTH-1:0] mm_s_rdata_o;
// Bus Control
input mm_s_we_i;

//---------- LED Control Signals -----------
output[3:0]	led_ctrl_o;//, led_debug_o;

input clk_16hz_i, clk_8hz_i, clk_1hz_i;

//---------- Internal Regs -----------
reg[15:0] led_ctrl_reg;
reg[MM_DATA_WIDTH-1:0] mm_s_rdata_o; 

reg[3:0] led_ctrl_o;
//wire[3:0] led_debug_o;

/***************************************************
 LED Control Functions
***************************************************/
// MM Write Function
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		led_ctrl_reg <= 0;
	end
	else if(mm_s_we_i) begin
		case(mm_s_addr_i)
			REG_ADDR_LED_CTRL:
			begin
				led_ctrl_reg <= mm_s_wdata_i;
			end
			default:
				led_ctrl_reg <= led_ctrl_reg;
		endcase
	end
end

// MM Read Function
always @(mm_s_addr_i, led_ctrl_reg, rst_n_i)
begin
	if(!rst_n_i) begin
		mm_s_rdata_o <= 0;
	end
	else begin
		case(mm_s_addr_i)
			REG_ADDR_LED_CTRL:
				mm_s_rdata_o <= led_ctrl_reg;
			default:
				mm_s_rdata_o <= 0;
		endcase
	end
end

// LED Blink Logic
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		led_ctrl_o <= 4'b1111;
	end
	else begin		
		case(led_ctrl_reg[2:1])
			BLINK_STOP:
				led_ctrl_o[0] <= ~led_ctrl_reg[0];
			BLINK_SLOW:
				led_ctrl_o[0] <= ~(clk_1hz_i & led_ctrl_reg[0]);
			BLINK_MID:
				led_ctrl_o[0] <= ~(clk_8hz_i & led_ctrl_reg[0]);
			BLINK_FAST:
				led_ctrl_o[0] <= ~(clk_16hz_i & led_ctrl_reg[0]);
		endcase		
		
		case(led_ctrl_reg[5:4])
			BLINK_STOP:
				led_ctrl_o[1] <= ~led_ctrl_reg[3];
			BLINK_SLOW:
				led_ctrl_o[1] <= ~(clk_1hz_i & led_ctrl_reg[3]);
			BLINK_MID:
				led_ctrl_o[1] <= ~(clk_8hz_i & led_ctrl_reg[3]);
			BLINK_FAST:
				led_ctrl_o[1] <= ~(clk_16hz_i & led_ctrl_reg[3]);
		endcase	
		
		case(led_ctrl_reg[8:7])
			BLINK_STOP:
				led_ctrl_o[2] <= ~led_ctrl_reg[6];
			BLINK_SLOW:
				led_ctrl_o[2] <= ~(clk_1hz_i & led_ctrl_reg[6]);
			BLINK_MID:
				led_ctrl_o[2] <= ~(clk_8hz_i & led_ctrl_reg[6]);
			BLINK_FAST:
				led_ctrl_o[2] <= ~(clk_16hz_i & led_ctrl_reg[6]);
		endcase	
		
		case(led_ctrl_reg[11:10])
			BLINK_STOP:
				led_ctrl_o[3] <= ~led_ctrl_reg[9];
			BLINK_SLOW:
				led_ctrl_o[3] <= ~(clk_1hz_i & led_ctrl_reg[9]);
			BLINK_MID:
				led_ctrl_o[3] <= ~(clk_8hz_i & led_ctrl_reg[9]);
			BLINK_FAST:
				led_ctrl_o[3] <= ~(clk_16hz_i & led_ctrl_reg[9]);
		endcase			
	end
end

//// Debug LED Logic
//assign led_debug_o = ~led_ctrl_reg[15:12];

endmodule
