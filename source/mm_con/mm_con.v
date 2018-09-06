/*
 *  Memory Mapped Bus Interconnection
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

module mm_con(
// Global Clock and Reset
clk_sys_i,
rst_n_i,

// Memory Mapped Bus Master Interface
m_addr_i,
m_wdata_i,
m_rdata_o,
m_we_i,

// Memory Mapped Bus Slave Interface
s_addr_o,
s_wdata_o,
s_rdata0_i,
s_rdata1_i,
s_rdata2_i,
s_rdata3_i,
s_rdata4_i,
s_we_o
);

/***************************************************
 Memory Mapped Bus Interconnection Parameters
***************************************************/
parameter MM_ADDR_WIDTH = 8;
parameter MM_DATA_WIDTH = 16;
// S0: Product Test
parameter REG_ADDR_PID = 'h00;
parameter REG_ADDR_TST = 'h02;
// S1: Interrupt Controller
parameter REG_ADDR_INT_PND = 'h04;
parameter REG_ADDR_INT_CLR = 'h06;
parameter REG_ADDR_INT_MSK = 'h08;
// S2: System WDT
parameter REG_ADDR_SWDT_CTRL = 'h0A;
parameter REG_ADDR_SWDT_VAL = 'h0C;
// S3: LED Controller
parameter REG_ADDR_LED_CTRL = 'h0E;
// S4: Power Controller
parameter REG_ADDR_PWR_CTRL = 'h10;
parameter REG_ADDR_PSU_STA  = 'h12;
parameter REG_ADDR_PWR_STA  = 'h14;
parameter REG_ADDR_PWR_ERR  = 'h16;


/***************************************************
 Declaration of IO Ports and Variable
***************************************************/
//---------- Global Clock and Reset -----------
input clk_sys_i, rst_n_i;

//---------- Master Interface Signals -----------
// Address and Data Bus
input[MM_ADDR_WIDTH-1:0] m_addr_i;
input[MM_DATA_WIDTH-1:0] m_wdata_i;
output[MM_DATA_WIDTH-1:0] m_rdata_o;
// Bus Control
input m_we_i;

//---------- Slave Interface Signals -----------
// Address and Data Bus
output[MM_ADDR_WIDTH-1:0] s_addr_o;
output[MM_DATA_WIDTH-1:0] s_wdata_o;
input[MM_DATA_WIDTH-1:0] s_rdata0_i, s_rdata1_i, s_rdata2_i, s_rdata3_i, s_rdata4_i;
// Bus Control
output s_we_o;

wire[MM_ADDR_WIDTH-1:0] m_addr_i;
wire[MM_DATA_WIDTH-1:0] m_wdata_i;
reg[MM_DATA_WIDTH-1:0] m_rdata_o;
wire m_we_i;
wire[MM_ADDR_WIDTH-1:0] s_addr_o;
wire[MM_DATA_WIDTH-1:0] s_wdata_o;
wire[MM_DATA_WIDTH-1:0] s_rdata0_i, s_rdata1_i, s_rdata2_i, s_rdata3_i, s_rdata4_i;
wire s_we_o;

reg[MM_DATA_WIDTH-1:0] rdata_buff;

/***************************************************
 MM Interconnection Functions
***************************************************/
// rdata bus switch
always @(m_addr_i, s_rdata0_i, s_rdata1_i, s_rdata2_i, s_rdata3_i, s_rdata4_i, rst_n_i)
begin
	if(!rst_n_i) begin
		rdata_buff <= 0;
	end
	else begin
		case(m_addr_i)		
			// S0: Product Test
			REG_ADDR_PID:
				rdata_buff <= s_rdata0_i;
			REG_ADDR_TST:
				rdata_buff <= s_rdata0_i;
				
			// S1: Interrupt Controller
			REG_ADDR_INT_PND:
				rdata_buff <= s_rdata1_i;
			REG_ADDR_INT_CLR:
				rdata_buff <= s_rdata1_i;
			REG_ADDR_INT_MSK:
				rdata_buff <= s_rdata1_i;
				
			// S2: System WDT
			REG_ADDR_SWDT_CTRL:
				rdata_buff <= s_rdata2_i;
			REG_ADDR_SWDT_VAL:
				rdata_buff <= s_rdata2_i;
				
			// S3: LED Controller
			REG_ADDR_LED_CTRL:
				rdata_buff <= s_rdata3_i;
				
			// S4: Power Controller
			REG_ADDR_PWR_CTRL:
				rdata_buff <= s_rdata4_i;
			REG_ADDR_PSU_STA:
				rdata_buff <= s_rdata4_i;
			REG_ADDR_PWR_STA:
				rdata_buff <= s_rdata4_i;
			REG_ADDR_PWR_ERR:
				rdata_buff <= s_rdata4_i;
			
			default:
				rdata_buff <= 0;
		endcase
	end
end


// Read Data Buffer
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		m_rdata_o <= 0;
	end
	else begin
		m_rdata_o <= rdata_buff;
	end
end

/***************************************************
 Wire Connections
***************************************************/
assign s_addr_o[MM_ADDR_WIDTH-1:0] = m_addr_i[MM_ADDR_WIDTH-1:0];
assign s_wdata_o[MM_DATA_WIDTH-1:0] = m_wdata_i[MM_DATA_WIDTH-1:0];
assign s_we_o = m_we_i;

endmodule
