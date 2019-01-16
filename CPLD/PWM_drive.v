module PWM_drive(CLK_50M,Rst_n,
                 R_PWM_LH_D,R_PWM_RH_D,
					  I_PWM1_LL_D,I_PWM1_LH_D,I_PWM1_RL_D,I_PWM1_RH_D,
					  I_PWM2_LL_D,I_PWM2_LH_D,I_PWM2_RL_D,I_PWM2_RH_D,
					  BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2,
					  Reset_D,
					  R_PWM_LH,R_PWM_RH,
					  I_PWM1_LL,I_PWM1_LH,I_PWM1_RL,I_PWM1_RH,
					  I_PWM2_LL,I_PWM2_LH,I_PWM2_RL,I_PWM2_RH
					  );
input CLK_50M,Rst_n;
input R_PWM_LH_D,R_PWM_RH_D;
input I_PWM1_LL_D,I_PWM1_LH_D,I_PWM1_RL_D,I_PWM1_RH_D;
input I_PWM2_LL_D,I_PWM2_LH_D,I_PWM2_RL_D,I_PWM2_RH_D;
input BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2,Reset_D;

output R_PWM_LH,R_PWM_RH;
output I_PWM1_LL,I_PWM1_LH,I_PWM1_RL,I_PWM1_RH;
output I_PWM2_LL,I_PWM2_LH,I_PWM2_RL,I_PWM2_RH;

reg PFC_EN1,PFC_EN2;
reg INV1_ENE,INV2_ENE;
reg INV1_ENI,INV2_ENI;
reg [7:0] count1,count2;
parameter [1:0] NormalState_PFC=2'b01,ProtectState_PFC=2'b10;
parameter [1:0] NormalState_INV2=2'b01,ProtectState_INV2=2'b10;
parameter [1:0] NormalState_INV1=2'b01,ProtectState_INV1=2'b10;
reg [1:0] State_PFC1,State_PFC2,State_INV2,State_INV1;

wire INV1_ENE_w,INV2_ENE_w;

assign INV1_ENE_w= (InvOcp1 & OP_Ovp1); assign INV2_ENE_w=(InvOcp2 & OP_Ovp2);

reg r_R_PWM_LH1,r_R_PWM_LH2;
reg r_R_PWM_RH1,r_R_PWM_RH2;

always @(posedge CLK_50M)
begin
 if(!Rst_n)
  begin r_R_PWM_LH1<=1'b0; r_R_PWM_LH2<=1'b0; end
 else
  begin r_R_PWM_LH1<=R_PWM_LH_D; r_R_PWM_LH2<=r_R_PWM_LH1; end
end

always @(posedge CLK_50M)
begin
 if(!Rst_n)
  begin r_R_PWM_RH1<=1'b0; r_R_PWM_RH2<=1'b0; end
 else
  begin r_R_PWM_RH1<=R_PWM_RH_D; r_R_PWM_RH2<=r_R_PWM_RH1; end
end

wire Rec_pos1,Rec_pos2;
wire Rec_pos;
assign Rec_pos1=(~r_R_PWM_LH2)&(r_R_PWM_LH1);
assign Rec_pos2=(~r_R_PWM_RH2)&(r_R_PWM_RH1);
assign Rec_pos=Rec_pos1 | Rec_pos2;



reg r_I_PWM1_LH1,r_I_PWM1_RH1,r_I_PWM2_LH1,r_I_PWM2_RH1;
reg r_I_PWM1_LH2,r_I_PWM1_RH2,r_I_PWM2_LH2,r_I_PWM2_RH2;

always @(posedge CLK_50M)
begin
 if(!Rst_n)
  begin r_I_PWM1_LH1<=1'b0; r_I_PWM1_LH2<=1'b0; end
 else
  begin r_I_PWM1_LH1<=I_PWM1_LH_D; r_I_PWM1_LH2<=r_I_PWM1_LH1; end
end

always @(posedge CLK_50M)
begin
 if(!Rst_n)
  begin r_I_PWM1_RH1<=1'b0; r_I_PWM1_RH2<=1'b0; end
 else
  begin r_I_PWM1_RH1<=I_PWM1_RH_D; r_I_PWM1_RH2<=r_I_PWM1_RH1; end
end

always @(posedge CLK_50M)
begin
 if(!Rst_n)
  begin r_I_PWM2_LH1<=1'b0; r_I_PWM2_LH2<=1'b0; end
 else
  begin r_I_PWM2_LH1<=I_PWM2_LH_D; r_I_PWM2_LH2<=r_I_PWM2_LH1; end
end

always @(posedge CLK_50M)
begin
 if(!Rst_n)
  begin r_I_PWM2_RH1<=1'b0; r_I_PWM2_RH2<=1'b0; end
 else
  begin r_I_PWM2_RH1<=I_PWM2_RH_D; r_I_PWM2_RH2<=r_I_PWM2_RH1; end
end


wire Inv1_pos1,Inv1_pos4,Inv2_pos1,Inv2_pos4;
wire Inv1_pos,Inv2_pos;
assign Inv1_pos1=(~r_I_PWM1_LH2)&(r_I_PWM1_LH1);
assign Inv1_pos4=(~r_I_PWM1_RH2)&(r_I_PWM1_RH1);
assign Inv2_pos1=(~r_I_PWM2_LH2)&(r_I_PWM2_LH1);
assign Inv2_pos4=(~r_I_PWM2_RH2)&(r_I_PWM2_RH1);
assign Inv1_pos= Inv1_pos1 | Inv1_pos4;
assign Inv2_pos= Inv2_pos1 | Inv2_pos4;

always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin State_PFC1<=NormalState_PFC; PFC_EN1<=1'b1; end
  else
   begin
    case(State_PFC1)
	  NormalState_PFC:
	   begin
	    if(!IP_Ocp)
		  begin State_PFC1<=ProtectState_PFC; PFC_EN1<=1'b0; end
		 else
		  begin State_PFC1<=NormalState_PFC; PFC_EN1<=1'b1; end
	   end
	  ProtectState_PFC:
	   begin
	    if(Reset_D)
		  begin State_PFC1<=NormalState_PFC; PFC_EN1<=1'b1; end
		 else
		  begin State_PFC1<=ProtectState_PFC; PFC_EN1<=1'b0; end
	   end
	  default:
	   begin State_PFC1<=NormalState_PFC;PFC_EN1<=1'b1; end
	 endcase
	end
 end
 
 always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin State_PFC2<=NormalState_PFC; PFC_EN2<=1'b1; end
  else
   begin
    case(State_PFC2)
	  NormalState_PFC:
	   begin
	    if(!BusOvp)
		  begin State_PFC2<=ProtectState_PFC; PFC_EN2<=1'b0; end
		 else
		  begin State_PFC2<=NormalState_PFC; PFC_EN2<=1'b1; end
	   end
	  ProtectState_PFC:
	   begin
	    if((Rec_pos)&(BusOvp))
		  begin State_PFC2<=NormalState_PFC; PFC_EN2<=1'b1; end
		 else
		  begin State_PFC2<=ProtectState_PFC; PFC_EN2<=1'b0; end
	   end
	  default:
	   begin State_PFC2<=NormalState_PFC;PFC_EN2<=1'b1; end
	 endcase
	end
 end
 
 
 always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin State_INV1<=NormalState_INV1; INV1_ENE<=1'b1; end
  else
   begin
    case(State_INV1)
	  NormalState_INV1:
	   begin
	    if(!INV1_ENE_w)
		  begin State_INV1<=ProtectState_INV1; INV1_ENE<=1'b0; end
		 else
		  begin State_INV1<=NormalState_INV1; INV1_ENE<=1'b1; end
	   end
	  ProtectState_INV1:
	   begin
	    if((Inv1_pos)&(INV1_ENE_w))
		  begin State_INV1<=NormalState_INV1; INV1_ENE<=1'b1; end
		 else
		  begin State_INV1<=ProtectState_INV1; INV1_ENE<=1'b0; end
	   end
	  default:
	   begin State_INV1<=NormalState_INV1;INV1_ENE<=1'b1; end
	 endcase
	end
 end
 
 always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin State_INV2<=NormalState_INV2; INV2_ENE<=1'b1; end
  else
   begin
    case(State_INV2)
	  NormalState_INV2:
	   begin
	    if(!INV2_ENE_w)
		  begin State_INV2<=ProtectState_INV2; INV2_ENE<=1'b0; end
		 else
		  begin State_INV2<=NormalState_INV2; INV2_ENE<=1'b1; end
	   end
	  ProtectState_INV2:
	   begin
	    if((Inv2_pos)&(INV2_ENE_w))
		  begin State_INV2<=NormalState_INV2; INV2_ENE<=1'b1; end
		 else
		  begin State_INV2<=ProtectState_INV2; INV2_ENE<=1'b0; end
	   end
	  default:
	   begin State_INV2<=NormalState_INV2;INV2_ENE<=1'b1; end
	 endcase
	end
 end
	
always @(posedge CLK_50M)
 begin 
  if(!Rst_n)
   begin count1<=8'd0; end
  else
   begin
	 if(!INV1_ENE)
	  begin
	   if(count1<8'd150)
		 begin count1<=count1+1'b1; end
		else
		 begin count1<=8'd150; end
	  end
	 else
	  begin count1<=8'd0; end
	end
 end
 
always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin INV1_ENI<=1'b0; end
  else
   begin
	 if(count1!=8'd150)
	  begin INV1_ENI<=1'b1; end
	 else
	  begin INV1_ENI<=1'b0; end
	end
 end
 
 always @(posedge CLK_50M)
 begin 
  if(!Rst_n)
   begin count2<=8'd0; end
  else
   begin
	 if(!INV2_ENE)
	  begin
	   if(count2<8'd150)
		 begin count2<=count2+1'b1; end
		else
		 begin count2<=8'd150; end
	  end
	 else
	  begin count2<=8'd0; end
	end
 end
 
always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin INV2_ENI<=1'b0; end
  else
   begin
	 if(count2!=8'd150)
	  begin INV2_ENI<=1'b1; end
	 else
	  begin INV2_ENI<=1'b0; end
	end
 end
 
 wire PWM_Pre1,PWM_Pre2;
 assign PWM_Pre1=~((I_PWM1_LL_D & I_PWM1_LH_D) | (I_PWM1_RL_D & I_PWM1_RH_D));
 assign PWM_Pre2=~((I_PWM2_LL_D & I_PWM2_LH_D) | (I_PWM2_RL_D & I_PWM2_RH_D));
 
 assign R_PWM_LH=(PFC_EN1 & PFC_EN2) ? R_PWM_LH_D : 1'b0;//去掉！
 assign R_PWM_RH=(PFC_EN1 & PFC_EN2) ? R_PWM_RH_D : 1'b0;
 
 assign I_PWM1_LL=(INV1_ENI & PWM_Pre1) ? I_PWM1_LL_D : 1'b0;
 assign I_PWM1_LH=(INV1_ENE & PWM_Pre1) ? I_PWM1_LH_D : 1'b0; 
 assign I_PWM1_RL=(INV1_ENI & PWM_Pre1) ? I_PWM1_RL_D : 1'b0;
 assign I_PWM1_RH=(INV1_ENE & PWM_Pre1) ? I_PWM1_RH_D : 1'b0; 
 
 assign I_PWM2_LL=(INV2_ENI & PWM_Pre2) ? I_PWM2_LL_D : 1'b0;
 assign I_PWM2_LH=(INV2_ENE & PWM_Pre2) ? I_PWM2_LH_D : 1'b0; 
 assign I_PWM2_RL=(INV2_ENI & PWM_Pre2) ? I_PWM2_RL_D : 1'b0;
 assign I_PWM2_RH=(INV2_ENE & PWM_Pre2) ? I_PWM2_RH_D : 1'b0; 
 
	 
endmodule
 
  
 
 
 
 