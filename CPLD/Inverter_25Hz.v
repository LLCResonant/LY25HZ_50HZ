module Inverter_25Hz(CLK_50M,DEV_CLRn,
                     R_PWM_LH_D,R_PWM_RH_D,
							I_PWM1_LL_D,I_PWM1_LH_D,I_PWM1_RL_D,I_PWM1_RH_D,
							I_PWM2_LL_D,I_PWM2_LH_D,I_PWM2_RL_D,I_PWM2_RH_D,
							BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2,
							Reset_D,CPLD1,
							OP_Rly1_D,OP_Rly2_D,OP_Scr1_D,OP_Scr2_D,
							COM1_IPF_C,COM2_IPF_C,COM1_OUT_D,COM2_OUT_D,
							R_PWM_LH_C,R_PWM_RH_C,
							I_PWM1_LL_C,I_PWM1_LH_C,I_PWM1_RL_C,I_PWM1_RH_C,
							I_PWM2_LL_C,I_PWM2_LH_C,I_PWM2_RL_C,I_PWM2_RH_C,
							BusOvp_D,IP_Ocp_D,InvOcp1_D,OP_Ovp1_D,InvOcp2_D,OP_Ovp2_D,
							CPLD2,CPLD_Data,
							EN1_4245,EN2_4245,
							OP_Rly1_C,OP_Rly2_C,OP_Scr1_C,OP_Scr2_C,
							COM1_IPF_D,COM2_IPF_D,COM1_OUT_C,COM2_OUT_C,
							LED1,LED2,LED3,LED4,LED5,LED6);
input CLK_50M,DEV_CLRn;
input R_PWM_LH_D,R_PWM_RH_D;
input I_PWM1_LL_D,I_PWM1_LH_D,I_PWM1_RL_D,I_PWM1_RH_D;
input I_PWM2_LL_D,I_PWM2_LH_D,I_PWM2_RL_D,I_PWM2_RH_D;
input BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2;
input Reset_D;
input CPLD1;
input OP_Rly1_D,OP_Rly2_D,OP_Scr1_D,OP_Scr2_D;
input COM1_IPF_C,COM2_IPF_C,COM1_OUT_D,COM2_OUT_D;

output R_PWM_LH_C,R_PWM_RH_C;
output I_PWM1_LL_C,I_PWM1_LH_C,I_PWM1_RL_C,I_PWM1_RH_C;
output I_PWM2_LL_C,I_PWM2_LH_C,I_PWM2_RL_C,I_PWM2_RH_C;
output BusOvp_D,IP_Ocp_D,InvOcp1_D,OP_Ovp1_D,InvOcp2_D,OP_Ovp2_D;
output CPLD2,CPLD_Data;
output EN1_4245,EN2_4245;
output OP_Rly1_C,OP_Rly2_C,OP_Scr1_C,OP_Scr2_C;
output COM1_IPF_D;
output reg COM2_IPF_D;
output COM1_OUT_C,COM2_OUT_C;
output LED1,LED2,LED3,LED4,LED5,LED6;

wire BusOvp_F,IP_Ocp_F,InvOcp1_F,OP_Ovp1_F,InvOcp2_F,OP_Ovp2_F;
wire R_PWM_LH_w,R_PWM_RH_w;
wire I_PWM1_LL_w,I_PWM1_LH_w,I_PWM1_RL_w,I_PWM1_RH_w;
wire I_PWM2_LL_w,I_PWM2_LH_w,I_PWM2_RL_w,I_PWM2_RH_w;
wire CPLD1_F;

Protect_Filter PF1(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),
                   .BusOvp(BusOvp),.IP_Ocp(IP_Ocp),.InvOcp1(InvOcp1),.OP_Ovp1(OP_Ovp1),.InvOcp2(InvOcp2),.OP_Ovp2(OP_Ovp2),
                   .BusOvp_F(BusOvp_F),.IP_Ocp_F(IP_Ocp_F),.InvOcp1_F(InvOcp1_F),.OP_Ovp1_F(OP_Ovp1_F),.InvOcp2_F(InvOcp2_F),.OP_Ovp2_F(OP_Ovp2_F));

LED_drive LD1(.BusOvp(BusOvp_F),.IP_Ocp(IP_Ocp_F),.InvOcp1(InvOcp1_F),.OP_Ovp1(OP_Ovp1_F),.InvOcp2(InvOcp2_F),.OP_Ovp2(OP_Ovp2_F),
              .LED1(LED1),.LED2(LED2),.LED3(LED3),.LED4(LED4),.LED5(LED5),.LED6(LED6));

PWM_drive PD1(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),
              .R_PWM_LH_D(R_PWM_LH_D),.R_PWM_RH_D(R_PWM_RH_D),
				  .I_PWM1_LL_D(I_PWM1_LL_D),.I_PWM1_LH_D(I_PWM1_LH_D),.I_PWM1_RL_D(I_PWM1_RL_D),.I_PWM1_RH_D(I_PWM1_RH_D),
				  .I_PWM2_LL_D(I_PWM2_LL_D),.I_PWM2_LH_D(I_PWM2_LH_D),.I_PWM2_RL_D(I_PWM2_RL_D),.I_PWM2_RH_D(I_PWM2_RH_D),
				  .BusOvp(BusOvp_F),.IP_Ocp(IP_Ocp_F),.InvOcp1(InvOcp1_F),.OP_Ovp1(OP_Ovp1_F),.InvOcp2(InvOcp2_F),.OP_Ovp2(OP_Ovp2_F),
				  .Reset_D(Reset_D),
				  .R_PWM_LH(R_PWM_LH_w),.R_PWM_RH(R_PWM_RH_w),
				  .I_PWM1_LL(I_PWM1_LL_w),.I_PWM1_LH(I_PWM1_LH_w),.I_PWM1_RL(I_PWM1_RL_w),.I_PWM1_RH(I_PWM1_RH_w),
				  .I_PWM2_LL(I_PWM2_LL_w),.I_PWM2_LH(I_PWM2_LH_w),.I_PWM2_RL(I_PWM2_RL_w),.I_PWM2_RH(I_PWM2_RH_w)
				  );
				  
//CPLD1_Filter CF1(.CLK_50M(CLK_50M),.RST_n(DEV_CLRn),.CPLD1(CPLD1),.CPLD1_F(CPLD1_F));

PWM_4245En PE1(.clk(CLK_50M),.Rst_n(DEV_CLRn),.OP_Rly1_D(OP_Rly1_D),.OP_Rly2_D(OP_Rly2_D),.OP_Scr1_D(OP_Scr1_D),
               .OP_Scr2_D(OP_Scr2_D),.COM1_OUT_D(COM1_OUT_D),.COM2_OUT_D(COM2_OUT_D),
               .En1(EN1_4245),.En2(EN2_4245),
               .OP_Rly1_C(OP_Rly1_C),.OP_Rly2_C(OP_Rly2_C),.OP_Scr1_C(OP_Scr1_C),
					.OP_Scr2_C(OP_Scr2_C),.COM1_OUT_C(COM1_OUT_C),.COM2_OUT_C(COM2_OUT_C));
					
Protect_CountCur PC1(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),.ResetD(Reset_D),.ProTect(InvOcp1_F),.PWMEN(InvOcp1_D));

Protect_CountCur PC2(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),.ResetD(Reset_D),.ProTect(InvOcp2_F),.PWMEN(InvOcp2_D));

Protect_CountVol PCV1(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),.ResetD(Reset_D),.ProTect(OP_Ovp1_F),.PWMEN(OP_Ovp1_D));

Protect_CountVol PCV2(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),.ResetD(Reset_D),.ProTect(OP_Ovp2_F),.PWMEN(OP_Ovp2_D));

//Protect_CountBus PCB1(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),.ResetD(Reset_D),.ProTect(BusOvp_F),.PWMEN(BusOvp_D));
					
 assign R_PWM_LH_C=(~EN1_4245) ? R_PWM_LH_w : 1'b0;//去掉！
 assign R_PWM_RH_C=(~EN1_4245) ? R_PWM_RH_w : 1'b0;
 
 assign I_PWM1_LL_C=(InvOcp1_D & OP_Ovp1_D) ? I_PWM1_LL_w : 1'b0;
 assign I_PWM1_LH_C=(InvOcp1_D & OP_Ovp1_D) ? I_PWM1_LH_w : 1'b0; 
 assign I_PWM1_RL_C=(InvOcp1_D & OP_Ovp1_D) ? I_PWM1_RL_w : 1'b0;
 assign I_PWM1_RH_C=(InvOcp1_D & OP_Ovp1_D) ? I_PWM1_RH_w : 1'b0; 
 
 assign I_PWM2_LL_C=(InvOcp2_D & OP_Ovp2_D) ? I_PWM2_LL_w : 1'b0;
 assign I_PWM2_LH_C=(InvOcp2_D & OP_Ovp2_D) ? I_PWM2_LH_w : 1'b0; 
 assign I_PWM2_RL_C=(InvOcp2_D & OP_Ovp2_D) ? I_PWM2_RL_w : 1'b0;
 assign I_PWM2_RH_C=(InvOcp2_D & OP_Ovp2_D) ? I_PWM2_RH_w : 1'b0; 
//	

COM_Filter CF1(.CLK_50M(CLK_50M),.Rst_n(DEV_CLRn),.COM1_IPF_C(COM1_IPF_C),.COM1_IPF_D(COM1_IPF_D));

always @(posedge CLK_50M)
 begin
   COM2_IPF_D<=COM2_IPF_C; 
 end
  
//assign OP_Rly1_C=OP_Rly1_D; assign OP_Rly2_C=OP_Rly2_D; assign OP_Scr1_C=OP_Scr1_D; assign OP_Scr2_C=OP_Scr2_D;
//assign COM1_IPF_D=COM1_IPF_C; assign COM2_IPF_D=COM2_IPF_C; assign COM1_OUT_C=COM1_OUT_D; assign COM2_OUT_C=COM2_OUT_D;


assign CPLD_Data=I_PWM1_LL_D;
				  
assign BusOvp_D=BusOvp_F; assign IP_Ocp_D=IP_Ocp_F; 
//assign InvOcp1_D=InvOcp1_F;assign OP_Ovp1_D=OP_Ovp1_F; 
//assign InvOcp2_D=InvOcp2_F; assign OP_Ovp2_D=OP_Ovp2_F;
assign CPLD2= I_PWM2_LL_D | I_PWM2_LL_C;

endmodule
