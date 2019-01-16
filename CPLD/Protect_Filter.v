module Protect_Filter(CLK_50M,Rst_n,
                      BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2,
                      BusOvp_F,IP_Ocp_F,InvOcp1_F,OP_Ovp1_F,InvOcp2_F,OP_Ovp2_F);
input CLK_50M,Rst_n;
input BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2;
output reg BusOvp_F,IP_Ocp_F,InvOcp1_F,OP_Ovp1_F,InvOcp2_F,OP_Ovp2_F;

reg [5:0] count1,count4;
reg [5:0] count2,count5;
reg [5:0] count3,count6;


always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count1<=6'd0; end
  else
   begin
	 if(!IP_Ocp)//!IP_Ocp
	  begin 
	   if(count1<6'd50)
		 begin count1<=count1+1'b1; end
		else
		 begin count1<=6'd50; end
	  end
	 else
	  begin count1<=6'd0; end
	end
 end	  
 
 always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count2<=6'd0; end
  else
   begin
	 if(!InvOcp1)//!InvOcp1
	  begin 
	   if(count2<6'd50)
		 begin count2<=count2+1'b1; end
		else
		 begin count2<=6'd50; end
	  end
	 else
	  begin count2<=6'd0; end
	end
 end
 
 always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count3<=6'd0; end
  else
   begin
	 if(!InvOcp2)//!InvOcp1
	  begin 
	   if(count3<6'd50)
		 begin count3<=count3+1'b1; end
		else
		 begin count3<=6'd50; end
	  end
	 else
	  begin count3<=6'd0; end
	end
 end

always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count4<=6'd0; end
  else
   begin
	 if(!BusOvp)
	  begin 
	   if(count4<6'd50)
		 begin count4<=count4+1'b1; end
		else
		 begin count4<=6'd50; end
	  end
	 else
	  begin count4<=6'd0; end
	end
 end	  
 
 always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count5<=6'd0; end
  else
   begin
	 if(!OP_Ovp1)//!IP_Ocp
	  begin 
	   if(count5<6'd50)
		 begin count5<=count5+1'b1; end
		else
		 begin count5<=6'd50; end
	  end
	 else
	  begin count5<=6'd0; end
	end
 end	  
 
 always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count6<=6'd0; end
  else
   begin
	 if(!OP_Ovp2)//!IP_Ocp
	  begin 
	   if(count6<6'd50)
		 begin count6<=count6+1'b1; end
		else
		 begin count6<=6'd50; end
	  end
	 else
	  begin count6<=6'd0; end
	end
 end	  
 

 
 always @(posedge CLK_50M)
 begin
   if(!Rst_n)
	 begin IP_Ocp_F<=1'b1; end//InvOcp2_F<=1'b1
	else
	 begin
	  if(count1!=6'd50)
		begin IP_Ocp_F<=1'b1; end//InvOcp2_F<=1'b1
	  else 
		begin IP_Ocp_F<=1'b0; end//InvOcp2_F<=1'b0
	 end
 end
 

 
 always @(posedge CLK_50M)
 begin
   if(!Rst_n)
	 begin InvOcp1_F<=1'b1; end//InvOcp2_F<=1'b1
	else
	 begin
	  if(count2!=6'd50)
		begin InvOcp1_F<=1'b1; end//InvOcp2_F<=1'b1
	  else 
		begin InvOcp1_F<=1'b0; end//InvOcp2_F<=1'b0
	 end
 end


 

 always @(posedge CLK_50M)
 begin
   if(!Rst_n)
	 begin InvOcp2_F<=1'b1; end//InvOcp2_F<=1'b1
	else
	 begin
	  if(count3!=6'd50)
		begin InvOcp2_F<=1'b1; end//InvOcp2_F<=1'b1
	  else 
		begin InvOcp2_F<=1'b0; end//InvOcp2_F<=1'b0
	 end
 end 
 
 
 always @(posedge CLK_50M)
 begin
   if(!Rst_n)
	 begin BusOvp_F<=1'b1; end//InvOcp2_F<=1'b1
	else
	 begin
	  if(count4!=6'd50)
		begin BusOvp_F<=1'b1; end//InvOcp2_F<=1'b1
	  else 
		begin BusOvp_F<=1'b0; end//InvOcp2_F<=1'b0
	 end
 end
 

 
 always @(posedge CLK_50M)
 begin
   if(!Rst_n)
	 begin OP_Ovp1_F<=1'b1; end//InvOcp2_F<=1'b1
	else
	 begin
	  if(count5!=6'd50)
		begin OP_Ovp1_F<=1'b1; end//InvOcp2_F<=1'b1
	  else 
		begin OP_Ovp1_F<=1'b0; end//InvOcp2_F<=1'b0
	 end
 end


 always @(posedge CLK_50M)
 begin
   if(!Rst_n)
	 begin OP_Ovp2_F<=1'b1; end//InvOcp2_F<=1'b1
	else
	 begin
	  if(count6!=6'd50)
		begin OP_Ovp2_F<=1'b1; end//InvOcp2_F<=1'b1
	  else 
		begin OP_Ovp2_F<=1'b0; end//InvOcp2_F<=1'b0
	 end
 end
 
endmodule  
		 