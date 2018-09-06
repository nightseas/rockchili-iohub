/*
 *  System Watch Dog Timer
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

module sys_wdt(
// Global Clock and Reset
clk_sys_i,
rst_n_i,

// Memory Mapped Bus Slave Interface
mm_s_addr_i,
mm_s_wdata_i,
mm_s_rdata_o,
mm_s_we_i,

// WDT Signals
clk_8hz_i,
wdt_ot_o
);

/***************************************************
 System WDT Parameters
***************************************************/
parameter MM_ADDR_WIDTH = 8;
parameter MM_DATA_WIDTH = 16;
// S2: System WDT
parameter REG_ADDR_SWDT_CTRL = 'h0A;
parameter REG_ADDR_SWDT_VAL = 'h0C;

//---------- Global Clock and Reset -----------
input clk_sys_i, rst_n_i;

//---------- Master Interface Signals -----------
// Address and Data Bus
input[MM_ADDR_WIDTH-1:0] mm_s_addr_i;
input[MM_DATA_WIDTH-1:0] mm_s_wdata_i;
output[MM_DATA_WIDTH-1:0] mm_s_rdata_o;
// Bus Control
input mm_s_we_i;

//---------- WDT Signals -----------
input clk_8hz_i;
output wdt_ot_o;

//---------- Internal Regs -----------
reg swdt_ctrl_reg;
reg[7:0] swdt_pwd_reg;
reg[12:0] swdt_val_reg;
reg[MM_DATA_WIDTH-1:0] mm_s_rdata_o; 
reg[12:0] wdt_cnt;
reg wdt_ot_o;
reg reg_clk_8hz_i;

wire wdt_en;

/***************************************************
 System WDT Functions
***************************************************/
// MM Write Function
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		swdt_ctrl_reg <= 1'b0;
		swdt_val_reg <= 13'h0960;
		swdt_pwd_reg <= 8'h27;
	end
	else begin
		if(mm_s_we_i) begin
			case(mm_s_addr_i)
				REG_ADDR_SWDT_CTRL:
				begin
					swdt_ctrl_reg <= mm_s_wdata_i[0];
					swdt_pwd_reg <= mm_s_wdata_i[15:8];
				end
				
				REG_ADDR_SWDT_VAL:
					swdt_val_reg <= mm_s_wdata_i[12:0];
					
				default:
				begin
					swdt_pwd_reg <= 8'h27;
					swdt_ctrl_reg <= swdt_ctrl_reg;
					swdt_val_reg <= swdt_val_reg;
				end
			endcase
		end
		else begin
			swdt_pwd_reg <= 8'h27;
		end
	end
end

// MM Read Function
always @(mm_s_addr_i, swdt_pwd_reg, swdt_ctrl_reg, swdt_val_reg, rst_n_i)
begin
	if(!rst_n_i) begin
		mm_s_rdata_o <= 0;
	end
	else begin
		case(mm_s_addr_i)
			REG_ADDR_SWDT_CTRL:
				mm_s_rdata_o <= {swdt_pwd_reg, 7'h0, swdt_ctrl_reg};
				
			REG_ADDR_SWDT_VAL:
				mm_s_rdata_o <= {3'h0, swdt_val_reg};
				
			default:
				mm_s_rdata_o <= 0;
		endcase
	end
end

// WDT Clock Source Edge Detection
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		reg_clk_8hz_i <= 1'b0;
	end
	else begin
		reg_clk_8hz_i <= clk_8hz_i;
	end
end

// WDT Counter
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		wdt_cnt <= 13'h0960;
	end
	else begin
		if(wdt_en) begin
			if(swdt_pwd_reg == 8'h5A)
				wdt_cnt <= swdt_val_reg;
			else if(swdt_pwd_reg == 8'hA5)
				wdt_cnt <= 13'h0;
			else if({reg_clk_8hz_i,clk_8hz_i} == 2'b01 && wdt_cnt != 0)
				wdt_cnt <= wdt_cnt - 1'b1;
			else
				wdt_cnt <= wdt_cnt;
		end
		else begin
			wdt_cnt <= wdt_cnt;
		end
	end
end

// WDT Overtime Flag
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		wdt_ot_o <= 1'b0;
	end
	else begin
		if(wdt_en == 1'b1 && wdt_cnt == 13'h0)
			wdt_ot_o <= 1'b1;
		else
			wdt_ot_o <= 1'b0;
	end
end

assign wdt_en = swdt_ctrl_reg;

endmodule
