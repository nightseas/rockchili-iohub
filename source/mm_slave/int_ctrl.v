/*
 *  Interrupt Controller
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

module int_ctrl(
// Global Clock and Reset
clk_sys_i,
rst_n_i,

// Memory Mapped Bus Slave Interface
mm_s_addr_i,
mm_s_wdata_i,
mm_s_rdata_o,
mm_s_we_i,

// Interrupt Signals
int_a_i, 
int_b_i, 
int_c_i, 
int_wdt_i,
sys_int_o
);

/***************************************************
 Interrupt Controller Parameters
***************************************************/
parameter MM_ADDR_WIDTH = 8;
parameter MM_DATA_WIDTH = 16;
// S1: Interrupt Controller
parameter REG_ADDR_INT_PND = 'h04;
parameter REG_ADDR_INT_CLR = 'h06;
parameter REG_ADDR_INT_MSK = 'h08;

//---------- Global Clock and Reset -----------
input clk_sys_i, rst_n_i;

//---------- Master Interface Signals -----------
// Address and Data Bus
input[MM_ADDR_WIDTH-1:0] mm_s_addr_i;
input[MM_DATA_WIDTH-1:0] mm_s_wdata_i;
output[MM_DATA_WIDTH-1:0] mm_s_rdata_o;
// Bus Control
input mm_s_we_i;

//---------- Interrupt Signals -----------
input int_a_i, int_b_i, int_c_i, int_wdt_i;
output sys_int_o;

//---------- Internal Regs & Wires -----------
reg[3:0] int_pnd_reg, int_clr_reg, int_msk_reg;
reg ie_msk_reg;
reg[MM_DATA_WIDTH-1:0] mm_s_rdata_o; 
reg[2:0] sync_int_a, sync_int_b, sync_int_c;
reg sys_int_o;

/***************************************************
 Interrupt Controller Functions
***************************************************/
// MM Write Function
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		int_clr_reg <= 4'b0000;
		int_msk_reg <= 4'b1111;
		ie_msk_reg <= 1'b1;
	end
	else begin
		if(mm_s_we_i) begin
			case(mm_s_addr_i)				
				REG_ADDR_INT_CLR:
					begin
						int_clr_reg[0] <= mm_s_wdata_i[0] ? 1'b1 : int_clr_reg[0];	
						int_clr_reg[1] <= mm_s_wdata_i[1] ? 1'b1 : int_clr_reg[1];	
						int_clr_reg[2] <= mm_s_wdata_i[2] ? 1'b1 : int_clr_reg[2];	
						int_clr_reg[3] <= mm_s_wdata_i[3] ? 1'b1 : int_clr_reg[3];
					end					
				REG_ADDR_INT_MSK:
					begin
						ie_msk_reg <= mm_s_wdata_i[15];
						int_msk_reg <= mm_s_wdata_i[3:0];
					end					
				default:
					begin
						int_clr_reg <= 4'b0000;
						int_msk_reg <= int_msk_reg;
						ie_msk_reg <= ie_msk_reg;
					end
			endcase
		end
		else begin
			int_clr_reg <= 4'b0000;
			int_msk_reg <= int_msk_reg;
			ie_msk_reg <= ie_msk_reg;
		end
	end
end

// MM Read Function
always @(mm_s_addr_i, int_pnd_reg, ie_msk_reg, int_msk_reg, rst_n_i)
begin
	if(!rst_n_i) begin
		mm_s_rdata_o <= 0;
	end
	else begin
		case(mm_s_addr_i)
			REG_ADDR_INT_PND:
				mm_s_rdata_o <= {12'h0, int_pnd_reg[3:0]};
			REG_ADDR_INT_MSK:
				mm_s_rdata_o <= {ie_msk_reg, 11'h0, int_msk_reg[3:0]};
			default:
				mm_s_rdata_o <= 0;
		endcase
	end
end

// Input Sync and Edge Detection
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		sync_int_a <= 3'b111;
		sync_int_b <= 3'b111;
		sync_int_c <= 3'b111;
	end
	else begin		
		sync_int_a <= {sync_int_a[1], sync_int_a[0], int_a_i};
		sync_int_b <= {sync_int_b[1], sync_int_b[0], int_b_i};
		sync_int_c <= {sync_int_c[1], sync_int_c[0], int_c_i};
	end
end

// Interrupt Pending Fill-in
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		int_pnd_reg <= 4'b0000;
	end
	else begin
		if(int_clr_reg[0])
			int_pnd_reg[0] <= 1'b0;
		else
			int_pnd_reg[0] <= (({sync_int_a[2], sync_int_a[1]} == 2'b10) && (int_msk_reg[0] == 1'b0)) ? 1'b1 : int_pnd_reg[0];
			
		if(int_clr_reg[1])
			int_pnd_reg[1] <= 1'b0;
		else
			int_pnd_reg[1] <= (({sync_int_b[2], sync_int_b[1]} == 2'b10) && (int_msk_reg[1] == 1'b0)) ? 1'b1 : int_pnd_reg[1];
		
		if(int_clr_reg[2])
			int_pnd_reg[2] <= 1'b0;
		else
			int_pnd_reg[2] <= (({sync_int_c[2], sync_int_c[1]} == 2'b10) && (int_msk_reg[2] == 1'b0)) ? 1'b1 : int_pnd_reg[2];
		
		if(int_clr_reg[3])
			int_pnd_reg[3] <= 1'b0;
		else
			int_pnd_reg[3] <= ((int_wdt_i == 1'b1) && (int_msk_reg[3] == 1'b0)) ? 1'b1 : int_pnd_reg[3];			
	end
end

// System Interrupt Output Logic
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		sys_int_o <= 1'b1;
	end
	else begin
		sys_int_o <= (~(int_pnd_reg[3] | int_pnd_reg[2] | int_pnd_reg[1] | int_pnd_reg[0])) | ie_msk_reg;
	end
end


endmodule
