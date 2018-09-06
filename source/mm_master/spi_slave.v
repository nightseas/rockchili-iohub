/*
 *  SPI Slave to Memory Mapped Bus Master Controller
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

module spi_slave(
//// For simulation only
//spi_main_fsm,
//spi_addr_cnt,
//spi_wdata_cnt,
//spi_rdata_cnt,
//spi_addr_buf,
//spi_wdata_buf,


// Global Clock and Reset
clk_sys_i,
rst_n_i,

// SPI Slave Interface
spi_sclk_i,
spi_mosi_i,
spi_miso_o,
spi_cs_n_i,

// Memory Mapped Bus Master Interface
mm_m_addr_o,
mm_m_wdata_o,
mm_m_rdata_i,
mm_m_we_o
);


/***************************************************
 SPI Slave Parameters
***************************************************/
parameter MM_ADDR_WIDTH = 8;
parameter MM_DATA_WIDTH = 16;
parameter DUMMY_DATA 	= 0;


// SPI Slave Main FSM
parameter STATE_IDLE	= 6'b00_0001;
parameter STATE_ADDR	= 6'b00_0010;
parameter STATE_WRITE	= 6'b00_0100;
parameter STATE_READ	= 6'b00_1000;
parameter STATE_W_DONE	= 6'b01_0000;
parameter STATE_R_DONE	= 6'b10_0000;

/***************************************************
 Declaration of IO Ports, Registers and Wires
***************************************************/
//---------- Global Clock and Reset -----------
input clk_sys_i, rst_n_i;

//---------- SPI Slave Interface -----------
input spi_sclk_i, spi_mosi_i, spi_cs_n_i;
output spi_miso_o;


//---------- Memory Mapped Bus Master Interface -----------
// Address and Data Bus
output[MM_ADDR_WIDTH-1:0] mm_m_addr_o;
output[MM_DATA_WIDTH-1:0] mm_m_wdata_o;
input[MM_DATA_WIDTH-1:0] mm_m_rdata_i;
// Bus Control
output mm_m_we_o;

//// For simulation only
//output[4:0] spi_addr_cnt;
//output[5:0] spi_wdata_cnt, spi_rdata_cnt;
//output[5:0] spi_main_fsm;
//output[MM_ADDR_WIDTH-1:0] spi_addr_buf;
//output[MM_DATA_WIDTH-1:0] spi_wdata_buf;


//---------- Internal Registers & Wires -----------
// SPI Interface
reg[2:0] sync_spi_sclk, sync_spi_cs_n;
reg[1:0] sync_spi_mosi;
reg spi_miso;

// MM Interface
wire[MM_ADDR_WIDTH-1:0] mm_m_addr_o;
wire[MM_DATA_WIDTH-1:0] mm_m_wdata_o;
wire[MM_DATA_WIDTH-1:0] mm_m_rdata_i;
reg mm_m_we_o;

reg[4:0] spi_addr_cnt;
reg[5:0] spi_wdata_cnt, spi_rdata_cnt;
reg[5:0] spi_main_fsm;

reg[MM_ADDR_WIDTH-1:0] spi_addr_buf;
reg[MM_DATA_WIDTH-1:0] spi_wdata_buf;

/***************************************************
 SPI Slave Functions
***************************************************/
// SPI Input Synchronous
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		sync_spi_sclk <= 2'b00;
		sync_spi_mosi <= 2'b00;
		sync_spi_cs_n <= 2'b00;
	end
	else begin
		sync_spi_sclk[2] <= sync_spi_sclk[1];
		sync_spi_sclk[1] <= sync_spi_sclk[0];
		sync_spi_sclk[0] <= spi_sclk_i;
		
		sync_spi_mosi[1] <= sync_spi_mosi[0];
		sync_spi_mosi[0] <= spi_mosi_i;
		
		sync_spi_cs_n[2] <= sync_spi_cs_n[1];
		sync_spi_cs_n[1] <= sync_spi_cs_n[0];
		sync_spi_cs_n[0] <= spi_cs_n_i;
	end
end

// SPI Slave Main FSM
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		spi_main_fsm <= STATE_IDLE;
		spi_addr_cnt <= 0;
		spi_wdata_cnt <= 0;
		spi_rdata_cnt <= 0;
		spi_addr_buf <= 0;
		spi_wdata_buf <= 0;
	end
	else if(sync_spi_cs_n[1] == 1'b1) begin
		spi_main_fsm <= STATE_IDLE;
	end
	else begin
		case(spi_main_fsm)
			STATE_IDLE:
			begin
				if(sync_spi_cs_n[2:1] == 2'b10) begin
					spi_main_fsm <= STATE_ADDR;
				end
				
				spi_addr_cnt <= 0;
				spi_wdata_cnt <= 0;
				spi_rdata_cnt <= 0;
				spi_addr_buf <= 0;
				spi_wdata_buf <= 0;
			end
			
			STATE_ADDR:
			begin
				if(spi_addr_cnt == MM_ADDR_WIDTH) begin
					if(spi_addr_buf[7] == 1'b1)
						spi_main_fsm <= STATE_READ;
					else
						spi_main_fsm <= STATE_WRITE;
				end
				else if(sync_spi_sclk[2:1] == 2'b01) begin
					spi_addr_buf[MM_ADDR_WIDTH-1-spi_addr_cnt] <= sync_spi_mosi[1];
					spi_addr_cnt <= spi_addr_cnt + 1'b1;
				end
			end
			
			STATE_WRITE:
			begin
				if(spi_wdata_cnt == MM_DATA_WIDTH) begin
					spi_main_fsm <= STATE_W_DONE;
				end
				else if(sync_spi_sclk[2:1] == 2'b01) begin
					spi_wdata_buf[MM_DATA_WIDTH-1-spi_wdata_cnt] <= sync_spi_mosi[1];
					spi_wdata_cnt <= spi_wdata_cnt + 1'b1;
				end
			end
			
			STATE_READ:
			begin
				if(spi_rdata_cnt == MM_DATA_WIDTH) begin
					spi_main_fsm <= STATE_R_DONE;
				end
				else if(sync_spi_sclk[2:1] == 2'b01) begin				
					spi_rdata_cnt <= spi_rdata_cnt + 1'b1;
				end
			end
			
			STATE_W_DONE:
			begin
				spi_main_fsm <= STATE_IDLE;
			end
			
			STATE_R_DONE:
			begin
				spi_main_fsm <= STATE_IDLE;
			end
			default:
			begin
				spi_main_fsm <= STATE_IDLE;
			end
		endcase
	end
end


// SPI MISO Control
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		spi_miso <= 0;
	end
	else if(spi_main_fsm == STATE_IDLE) begin
		spi_miso <= 0;
	end
	else begin
		if(spi_main_fsm == STATE_READ && sync_spi_sclk[2:1] == 2'b10 && spi_rdata_cnt < MM_DATA_WIDTH)			
			spi_miso <= mm_m_rdata_i[MM_DATA_WIDTH-1-spi_rdata_cnt];					
		else
			spi_miso <= spi_miso;
	end
end

// SPI MM Interface Write Enable Control
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		mm_m_we_o <= 0;
	end
	else begin
		if(spi_main_fsm == STATE_W_DONE)
			mm_m_we_o <= 1;
		else
			mm_m_we_o <= 0;
	end
end

// Blank Function
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		
	end
	else begin
		
	end
end



/***************************************************
 Wire and Tri-State Connections
***************************************************/
// MM Interfaces
assign mm_m_addr_o[MM_ADDR_WIDTH-1:0] = {1'b0, spi_addr_buf[MM_ADDR_WIDTH-2:0]};
assign mm_m_wdata_o[MM_DATA_WIDTH-1:0] = spi_wdata_buf[MM_DATA_WIDTH-1:0];
assign spi_miso_o = spi_cs_n_i ? 1'bz : spi_miso;

endmodule
