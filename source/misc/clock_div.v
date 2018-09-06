/*
 *  Clock Divider
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
 
module clock_div(
clk_sys_i,
rst_n_i,

clk_16hz_o,
clk_8hz_o,
clk_1hz_o
);

/***************************************************
 Clock Generator Parameters
***************************************************/
parameter FREQ_SYSCLK = 25_000_000;

/***************************************************
 Declaration of IO Ports and Variable
***************************************************/
input clk_sys_i, rst_n_i;
output clk_16hz_o, clk_8hz_o, clk_1hz_o;

reg[20:0] clk_pre_cnt;
reg[4:0] clk_post_cnt;

/***************************************************
 Clock Generator Functions
***************************************************/
// Pre-dividing counter for 
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		clk_pre_cnt <= 0;
	end
	else begin
		// Generate 32Hz clock
		if(clk_pre_cnt == (FREQ_SYSCLK / 32 - 1))
			clk_pre_cnt <= 0;
		else
			clk_pre_cnt <= clk_pre_cnt + 1'b1;
	end
end

// Post-dividing counter
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		clk_post_cnt <= 0;
	end
	else if(clk_pre_cnt == 0) begin
		clk_post_cnt <= clk_post_cnt + 1'b1;
	end
end

/***************************************************
 Wire Connections
***************************************************/
wire clk_16hz_o = clk_post_cnt[0];
wire clk_8hz_o = clk_post_cnt[1];
wire clk_1hz_o = clk_post_cnt[4];

endmodule
