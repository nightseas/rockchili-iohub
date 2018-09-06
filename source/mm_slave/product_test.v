/*
 *  Device Information & Test Logic
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
 
module product_test(
// Global Clock and Reset
clk_sys_i,
rst_n_i,

// Memory Mapped Bus Slave Interface
mm_s_addr_i,
mm_s_wdata_i,
mm_s_rdata_o,
mm_s_we_i,

// Bootstrap Signals
pid_strap_i
);

/***************************************************
 Misc Logic Parameters
***************************************************/
parameter MM_ADDR_WIDTH = 8;
parameter MM_DATA_WIDTH = 16;
// S0: Product ID
parameter REG_ADDR_PID = 'h00;
parameter REG_ADDR_TST = 'h02;

parameter FW_VERSION = 8'h01;

//---------- Global Clock and Reset -----------
input clk_sys_i, rst_n_i;

//---------- Master Interface Signals -----------
// Address and Data Bus
input[MM_ADDR_WIDTH-1:0] mm_s_addr_i;
input[MM_DATA_WIDTH-1:0] mm_s_wdata_i;
output[MM_DATA_WIDTH-1:0] mm_s_rdata_o;
// Bus Control
input mm_s_we_i;

//---------- Bootstrap Signals -----------
input[3:0] pid_strap_i;

//---------- Internal Regs -----------
reg[3:0] pid_reg;
reg[MM_DATA_WIDTH-1:0] test_reg;
reg[MM_DATA_WIDTH-1:0] mm_s_rdata_o; 

reg edge_rst;

/***************************************************
 Device Information & Test Functions
***************************************************/
//MM Write Function
 always @(posedge clk_sys_i, negedge rst_n_i)
 begin
 	if(!rst_n_i) begin
 		test_reg <= 0;
 	end
 	else if(mm_s_we_i) begin
 		case(mm_s_addr_i)
 			REG_ADDR_TST:
 				test_reg <= mm_s_wdata_i;
 			default:
 				test_reg <= test_reg;
 		endcase
 	end
 end

// MM Read Function
always @(mm_s_addr_i, pid_reg, test_reg, rst_n_i)
begin
	if(!rst_n_i) begin
		mm_s_rdata_o <= 0;
	end
	else begin
		case(mm_s_addr_i)
			REG_ADDR_PID:
				mm_s_rdata_o <= {FW_VERSION, 4'h0, pid_reg[3:0]};
			REG_ADDR_TST:
 				mm_s_rdata_o <= test_reg;
			default:
				mm_s_rdata_o <= 0;
		endcase
	end
end


// PID Bootstrap on Reset
always @(posedge clk_sys_i)
begin
	edge_rst <= rst_n_i;
end

// PID Bootstrap on Reset
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i)
		pid_reg <= 0;
	else if(!edge_rst)
		pid_reg <= pid_strap_i;
	else
		pid_reg <= pid_reg;
end


endmodule
