module PWM_delay(CLK_50M,Rst_n,R_LH,R_RH,I1_RH,I1_RL,I1_LH,I1_LL,I2_RH,I2_RL,I2_LH,I2_LL,
                 R_LH_Ot,R_RH_Ot,I1_RH_Ot,I1_RL_Ot,I1_LH_Ot,I1_LL_Ot,I2_RH_Ot,I2_RL_Ot,I2_LH_Ot,I2_LL_Ot);
input CLK_50M,Rst_n;
input R_LH,R_RH,I1_RH,I1_RL,I1_LH,I1_LL,I2_RH,I2_RL,I2_LH,I2_LL;
output R_LH_Ot,R_RH_Ot,I1_RH_Ot,I1_RL_Ot,I1_LH_Ot,I1_LL_Ot,I2_RH_Ot,I2_RL_Ot,I2_LH_Ot,I2_LL_Ot;

reg [23:0] count;
reg en;

always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin
	 count<=24'd0;
	 en<=1'b0;
	end
  else
   begin
	 if(count<24'd15000000)
	  begin
	   count<=count+1'b1;
		en<=1'b0;
	  end
	 else
	  begin
	   count<=24'd15000000;
		en<=1'b1;
	  end
	end
 end
 
 assign R_LH_Ot=(en) ? R_LH : 1'b0; assign R_RH_Ot=(en) ? R_RH : 1'b0;
 assign I1_RH_Ot=(en) ? I1_RH : 1'b0; assign I1_RL_Ot=(en) ? I1_RL : 1'b0;
 assign I1_LH_Ot=(en) ? I1_LH : 1'b0; assign I1_LL_Ot=(en) ? I1_LL : 1'b0;
 assign I2_RH_Ot=(en) ? I2_RH : 1'b0; assign I2_RL_Ot=(en) ? I2_RL : 1'b0;
 assign I2_LH_Ot=(en) ? I2_LH : 1'b0; assign I2_LL_Ot=(en) ? I2_LL : 1'b0;
 
 endmodule
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 