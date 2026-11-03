/*******************************MILIANKE*******************************
*Company : MiLianKe Electronic Technology Co., Ltd.
*WebSite:https://www.milianke.com
*TechWeb:https://www.uisrc.com
*tmall-shop:https://milianke.tmall.com
*jd-shop:https://milianke.jd.com
*taobao-shop1: https://milianke.taobao.com
*Create Date: 2023/03/23
*Module Name:
*File Name:
*Description: 
*The reference demo provided by Milianke is only used for learning. 
*We cannot ensure that the demo itself is free of bugs, so users 
*should be responsible for the technical problems and consequences
*caused by the use of their own products.
*Copyright: Copyright (c) MiLianKe
*All rights reserved.
*Revision: 1.0
*Signal description
*1) I_ input
*2) O_ output
*3) IO_ input output
*4) S_ system internal signal
*5) _n activ low
*6) _dg debug signal 
*7) _r delay or register
*8) _s state mechine
*********************************************************************/
/*********ae_set �ֶ��������AE ***********
--�汾��1.0
*********************************************************************/

module ae_set
(
input wire I_clk,
input wire I_rst,
input wire [3:0]I_btn,
input wire I_ae_cfg_done,
input wire I_cam_cfg_done,
output wire O_ae_req,
output wire [15:0] O_ae,
output wire [15:0] O_ag
);
	

reg 	[19:0] 	rc_ae_tick = 0; 
reg 			r_ae_tick = 0; 
reg 	[4:0] 	rc_ag_tick = 0; 
reg 			r_ag_tick = 0; 
	
always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		rc_ae_tick <= 0; 
		r_ae_tick <= 0; 
		rc_ag_tick <= 0; 
		r_ag_tick <= 0; 
	end else begin
	//	Trigger AE Inc / Dec per 1 ms. 
		rc_ae_tick <= rc_ae_tick + 1; 
		r_ae_tick <= 0; 
		if(rc_ae_tick >= 99999) begin
			rc_ae_tick <= 0; 
			r_ae_tick <= 1; 
		end 
	//	Trigger AG Inc / Dec per 32 ms. 
		rc_ag_tick <= rc_ag_tick + r_ae_tick; 
		r_ag_tick <= (&rc_ag_tick) && r_ae_tick; 
	end
end
	
//	Use button to add / sub idelay values. I_button[0]:IncDatDly, [1]DecDatDly, [2]IncI_clkDly, [3]DecI_clkDly
reg 	[3:0] 	r_button_0 = 0, r_button_1 = 0; 
// Use a moderate lift for distant targets while retaining headroom for bright
// lamps so the red detector still sees channel separation.
reg 	[15:0] 	r_ae_set = 1800, r_ag_set = 82;
always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		r_button_0 <= 0; 
		r_button_1 <= 0; 
		r_ae_set <= 1800;
		r_ag_set <= 82;
	end else begin
	//	When button pressed, r_button_1 bit is 1. 
		r_button_0 <= ~I_btn; 
		r_button_1 <= r_button_0; 
	
		if(r_button_1[0]) begin
			r_ae_set <= (r_ae_set > 2) ? r_ae_set - r_ae_tick : r_ae_set; 
		end else if(r_button_1[1]) begin
			r_ae_set <= (r_ae_set < 1990) ? r_ae_set + r_ae_tick : r_ae_set; 
		end 
		if(r_button_1[2]) begin
				r_ag_set <= (r_ag_set > 16) ? r_ag_set - r_ag_tick : r_ag_set; 
		end else if(r_button_1[3]) begin
				r_ag_set <= (r_ag_set < 89) ? r_ag_set + r_ag_tick : r_ag_set; 
		end 
		end
	end
	
//	Update AG / AE when changed. 
reg 	[15:0] 	r_ae ; 
reg 	[15:0] 	r_ag ; 
reg 			r_ae_req = 0; //synthesis keep
reg initial_pending;
reg [1:0] update_state;
localparam UPDATE_IDLE=0, WAIT_ACCEPT=1, WAIT_COMPLETE=2;
assign O_ae = r_ae;
assign O_ag = r_ag;
assign O_ae_req = r_ae_req;

always @(posedge I_clk or posedge I_rst) begin
	if(I_rst) begin
		r_ae_req <= 0; 
		r_ae     <= 1800;
		r_ag     <= 82;
		initial_pending <= 1;
        update_state <= UPDATE_IDLE;
    end else begin
        r_ae_req <= 0;
        if(!I_cam_cfg_done) begin
            initial_pending <= 1;
            update_state <= UPDATE_IDLE;
        end else case(update_state)
            UPDATE_IDLE: if(I_ae_cfg_done && (initial_pending ||
                            r_ae != r_ae_set || r_ag != r_ag_set)) begin
                r_ae <= r_ae_set;
                r_ag <= r_ag_set;
                r_ae_req <= 1;
                update_state <= WAIT_ACCEPT;
            end
            // Keep the requested pair stable until the controller finishes.
            WAIT_ACCEPT: if(!I_ae_cfg_done) update_state <= WAIT_COMPLETE;
            WAIT_COMPLETE: if(I_ae_cfg_done) begin
                initial_pending <= 0;
                update_state <= UPDATE_IDLE;
            end
            default: begin initial_pending <= 1; update_state <= UPDATE_IDLE; end
        endcase
	end
end


endmodule


	
