/*
 *  System Power Controller
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

module pwr_ctrl(
// For Simulation Only
//pwr_ctrl_fsm,

// Global Clock and Reset
clk_sys_i,
rst_n_i,

// Memory Mapped Bus Slave Interface
mm_s_addr_i,
mm_s_wdata_i,
mm_s_rdata_o,
mm_s_we_i,

// Power Control Signals
psu_prsnt_i,
pwr_good_i,
pwr_en_o,
pwr_err_o,
pwr_allgood_o
);

/***************************************************
 Power Controll Parameters
***************************************************/
parameter MM_ADDR_WIDTH = 8;
parameter MM_DATA_WIDTH = 16;
// S4: Power Controller
parameter REG_ADDR_PWR_CTRL = 'h10;
parameter REG_ADDR_PSU_STA  = 'h12;
parameter REG_ADDR_PWR_STA  = 'h14;
parameter REG_ADDR_PWR_ERR  = 'h16;

// Power Controll FSM State Definition
parameter STATE_OFF	= 4'b0001;
parameter STATE_UP	= 4'b0010;
parameter STATE_ON	= 4'b0100;
parameter STATE_ERR	= 4'b1000;

// System clock frequency for WDT auto calulation
parameter FREQ_SYSCLK = 25_000_000;
parameter TIME_SYSCLK = 1_000_000_000 / FREQ_SYSCLK;

// Power failure watchdog timeout: 100ms
parameter TIME_PWR_WDT = 1_000_000_000;
//parameter TIME_PWR_WDT = 100_000; // 100us for simulation

//---------- Global Clock and Reset -----------
input clk_sys_i, rst_n_i;

//---------- Master Interface Signals -----------
// Address and Data Bus
input[MM_ADDR_WIDTH-1:0] mm_s_addr_i;
input[MM_DATA_WIDTH-1:0] mm_s_wdata_i;
output[MM_DATA_WIDTH-1:0] mm_s_rdata_o;
// Bus Control
input mm_s_we_i;

//---------- Power Controll Signals -----------
input[1:0] psu_prsnt_i;
input[14:0] pwr_good_i;
output[11:0] pwr_en_o;
output pwr_err_o;
output pwr_allgood_o;

//For simulation only
//output[3:0] pwr_ctrl_fsm;

reg[11:0] pwr_en_o;
reg[7:0] pwr_good_stage;
// Power Enable Singal Definition
// BIT	Name			Assert Level
// 11	EN_PSU1			0
// 10	EN_PSU0			0
// 9	EN_P1V8_A		1
// 8	EN_P1V2_A1		1
// 7	EN_P1V2_A0		1
// 6	EN_P1V8			1
// 5	EN_P0V9_A1		1
// 4	EN_P0V9_A0		1
// 3	EN_P0V85_1		1
// 2	EN_P0V85_0		1
// 1	EN_P3V7			1
// 0	EN_P3V3			1

// Power Good Singal Definition
// BIT	Name			Assert Level
// 14	PGOOD_PSU1		1
// 13	PGOOD_PSU0		1
// 12	PGOOD_P1V8_A	1
// 11	PGOOD_P1V2_A1	1
// 10	PGOOD_P1V2_A0	1
// 9	PGOOD_P1V8		1
// 8	PGOOD_P0V9_A1	1
// 7	PGOOD_P0V9_A0	1
// 6	PGOOD_P0V85_1	1
// 5	PGOOD_P0V85_0	1
// 4	PGOOD_P3V7		1
// 3	PGOOD_P3V3_VCC	1
// 2	PGOOD_P3V3		1
// 1	PGOOD_P1V8_EA	1
// 0	PGOOD_P3V3_EA	1

// PSU Present Singal Definition
// BIT	Name			Assert Level
// 1	PRSNT_PSU1		0
// 0	PRSNT_PSU0		0
	

//---------- Internal Regs -----------
reg[1:0] pwr_ctrl_reg;
reg[14:0] pwr_err_reg;
reg[MM_DATA_WIDTH-1:0] mm_s_rdata_o; 

reg[31:0] pwr_delay_cnt;
reg[31:0] pwr_wdt_cnt;
reg pwr_delay_en, pwr_wdt_en, pwr_delay_ot, pwr_wdt_ot;

reg pwr_auto_start;
reg pwr_err_o;
reg pwr_allgood_o;

reg[3:0] pwr_ctrl_fsm;


/***************************************************
 Power Controll Functions
***************************************************/
// MM Write Function
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_ctrl_reg <= 0;
	end
	else begin
		if(mm_s_we_i) begin
			case(mm_s_addr_i)
				REG_ADDR_PWR_CTRL:
				begin
					pwr_ctrl_reg <= mm_s_wdata_i[1:0];
				end
				
				default:
				begin
					pwr_ctrl_reg <= 0;
				end
			endcase
		end
		else begin
			pwr_ctrl_reg <= 0;
		end
	end
end

// MM Read Function
always @(mm_s_addr_i, pwr_ctrl_reg, pwr_good_i, psu_prsnt_i, pwr_allgood_o, pwr_err_reg, rst_n_i)
begin
	if(!rst_n_i) begin
		mm_s_rdata_o <= 0;
	end
	else begin
		case(mm_s_addr_i)
			REG_ADDR_PWR_CTRL:
				mm_s_rdata_o <= {14'h0, pwr_ctrl_reg[1:0]};
				
			REG_ADDR_PSU_STA:
				mm_s_rdata_o <= {12'h0, pwr_good_i[14:13], ~psu_prsnt_i[1:0]};
			
			REG_ADDR_PWR_STA:
				mm_s_rdata_o <= {pwr_allgood_o, 2'h0, pwr_good_i[12:0]};
				
			REG_ADDR_PWR_ERR:
				mm_s_rdata_o <= {1'h0, pwr_err_reg};
		
			default:
				mm_s_rdata_o <= 0;
		endcase
	end
end

// Power Controll FSM
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_ctrl_fsm <= STATE_OFF;
	end
	else begin
		case(pwr_ctrl_fsm)
			STATE_OFF:
			begin
				// TBD: power button
					// if(pwr_ctrl_reg == 2'b01 | pwr_btn_on == 1'b1 | pwr_auto_start == 1'b1) begin
				if(pwr_ctrl_reg == 2'b01 | pwr_auto_start == 1'b1) begin
					pwr_ctrl_fsm <= STATE_UP;
				end
			end
			
			STATE_UP:
			begin
				if(pwr_allgood_o == 1'b1) begin
					pwr_ctrl_fsm <= STATE_ON;
				end
				else if(pwr_wdt_ot == 1'b1) begin
					pwr_ctrl_fsm <= STATE_ERR;
				end
			end
			
			STATE_ON:
			begin				
				if(pwr_allgood_o == 1'b0) begin
					pwr_ctrl_fsm <= STATE_ERR;
				end
				// TBD: power button
					// if(pwr_ctrl_reg == 2'b10 | pwr_btn_off == 1'b1) begin
				else if(pwr_ctrl_reg == 2'b10) begin
					pwr_ctrl_fsm <= STATE_OFF;
				end
			end
			
			STATE_ERR:
			begin
				pwr_ctrl_fsm <= STATE_OFF;
			end

			default:
			begin
				pwr_ctrl_fsm <= STATE_OFF;
			end
		endcase
	end
end

// Power Good Stage Control
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_good_stage <= 0;
	end
	else begin
		// Stage 0: PSU inserted, to triger PSU enable
			// TBD: Ignore PSU present singals due to Delta PSU PG/PRNST issue
			// pwr_good_stage[0] <= ~(psu_prsnt_i[1] | psu_prsnt_i[0]);
		pwr_good_stage[0] <= 1'b1;
		
		// Stage 1: (PGOOD_PSU0 or PGOOD_PSU1) and PGOOD_P3V3_EA and PGOOD_P1V8_EA active, to triger on board DCDC power rails
			// TBD: Ignore PSU power good singals due to Delta PSU PG/PRNST issue
			// pwr_good_stage[1] <= pwr_good_i[1] & pwr_good_i[0];
		pwr_good_stage[1] <= (pwr_good_i[14] | pwr_good_i[13]) & pwr_good_i[1] & pwr_good_i[0];
				
		// Stage 2: PGOOD_P3V7 and PGOOD_P3V3_VCC and PGOOD_P3V3 active
		pwr_good_stage[2] <= pwr_good_i[4] & pwr_good_i[3] & pwr_good_i[2];
		
		// Stage 3: PGOOD_P0V85_1 and PGOOD_P0V85_0 active
		pwr_good_stage[3] <= pwr_good_i[6] & pwr_good_i[5];
		
		// Stage 4: PGOOD_P1V8 and PGOOD_P0V9_A1 and PGOOD_P0V9_A0 active
		pwr_good_stage[4] <= pwr_good_i[9] & pwr_good_i[8] & pwr_good_i[7];
		
		// Stage 5: PGOOD_P1V2_A1 and PGOOD_P1V2_A0 active
		pwr_good_stage[5] <= pwr_good_i[11] & pwr_good_i[10];

		// Stage 6: PGOOD_P1V8_A active
		pwr_good_stage[6] <= pwr_good_i[12];
	end
end

// Power Good Stage Control
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_allgood_o <= 1'b0;
	end
	else begin		
		if(pwr_good_stage == 7'b111_1111)
			pwr_allgood_o <= 1'b1;
		else
			pwr_allgood_o <= 1'b0;
	end
end

// Power Enable Control
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_en_o <= 12'b1100_0000_0000;
		
	end
	else begin
		if(pwr_ctrl_fsm == STATE_UP) begin
			// EN_PSU1 and EN_PSU0
			pwr_en_o[11] <= ~pwr_good_stage[0];
			pwr_en_o[10] <= ~pwr_good_stage[0];
			
			// EN_P3V7 and EN_P3V3
			pwr_en_o[1] <= pwr_good_stage[1];
			pwr_en_o[0] <= pwr_good_stage[1];
			
			// EN_P0V85_1 and EN_P0V85_0
			pwr_en_o[3] <= pwr_good_stage[2];
			pwr_en_o[2] <= pwr_good_stage[2];
			
			// EN_P1V8 & EN_P0V9_A1 & EN_P0V9_A0
			pwr_en_o[6] <= pwr_good_stage[3];
			pwr_en_o[5] <= pwr_good_stage[3];
			pwr_en_o[4] <= pwr_good_stage[3];
			
			// EN_P1V2_A1 and EN_P1V2_A0
			pwr_en_o[8] <= pwr_good_stage[4];
			pwr_en_o[7] <= pwr_good_stage[4];
			
			// EN_P1V8_A
			pwr_en_o[9] <= pwr_good_stage[5];
		end
		
		else if(pwr_ctrl_fsm == STATE_OFF) begin
			pwr_en_o <= 12'b1100_0000_0000;
		end
	end
end

// Power Error Handler
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_err_reg <= 0;
		pwr_err_o <= 0;
	end
	else begin
		if(pwr_ctrl_fsm == STATE_ERR) begin		
			pwr_err_reg <= pwr_good_i;
			pwr_err_o <= 1'b1;
		end
	end
end

// Auto-Start at First Power Up
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_auto_start <= 1'b1;
	end
	else begin
		if(pwr_ctrl_fsm == STATE_UP)
			pwr_auto_start <= 1'b0;
	end
end

// Power Up Failure Detector WDT
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin
		pwr_wdt_cnt <= 0;
		pwr_wdt_ot <= 0;
	end
	else begin
		if(pwr_ctrl_fsm == STATE_OFF) begin
			pwr_wdt_cnt <= 0;
			pwr_wdt_ot <= 0;
		end		
		else if(pwr_ctrl_fsm == STATE_UP) begin
			pwr_wdt_cnt <= pwr_wdt_cnt + 1'b1;
			
			if(pwr_wdt_cnt >= TIME_PWR_WDT / (1_000_000_000 / FREQ_SYSCLK))
				pwr_wdt_ot <= 1;
		end
			
	end
end

// empty
always @(posedge clk_sys_i, negedge rst_n_i)
begin
	if(!rst_n_i) begin

	end
	else begin
		
	end
end


endmodule
