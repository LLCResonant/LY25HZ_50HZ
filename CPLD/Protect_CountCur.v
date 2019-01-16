module Protect_CountCur(CLK_50M,Rst_n,ResetD,ProTect,PWMEN);
input CLK_50M,Rst_n,ResetD,ProTect;
output reg PWMEN;

reg [9:0] count1=10'b0;
reg [21:0] count2=22'b0;
reg [1:0] State_INV,State_Circuit;
parameter [1:0] NormalState_INV=2'b01,CountState_INV=2'b10;
parameter [1:0] NormalState_Circuit=2'b01,ShortState_Circuit=2'b10;
reg r_ProTect1,r_ProTect2;
wire ProTect_neg;
reg count_en=1'b0;
reg Circuit_short=1'b0;
reg CountFull=1'b0;

always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin
	 r_ProTect1<=1'b0; r_ProTect2<=1'b0;
	end
  else
   begin
	 r_ProTect1<=ProTect; r_ProTect2<=r_ProTect1;
	end
 end
 
assign ProTect_neg=(r_ProTect2)&(~r_ProTect1);

always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin
	 State_INV<=NormalState_INV; count_en<=1'b0;
	end
  else
   begin
	 case(State_INV)
	  NormalState_INV:
	   begin
		 if(!ProTect)
		  begin State_INV<=CountState_INV; count_en<=1'b1; end
		 else
		  begin State_INV<=NormalState_INV; count_en<=1'b0; end
		end
	  CountState_INV:
	   begin
		 if(CountFull)
		  begin State_INV<=NormalState_INV; count_en<=1'b0; end
		 else
		  begin State_INV<=CountState_INV; count_en<=1'b1; end
		end
	  default:
	   begin State_INV<=NormalState_INV; count_en<=1'b0; end
	 endcase
	end
 end
 
always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count2<=22'd0; CountFull<=1'b0; end
  else
   begin
	 if(count_en)
	  begin 
	   if(count2<22'd3999999)
		 begin count2<=count2+1'b1; CountFull<=1'b0; end
	   else
	    begin count2<=22'd0; CountFull<=1'b1; end
	  end
	 else
	  begin 
	   count2<=22'd0; CountFull<=1'b0;
	  end
	end
 end
		 
always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin count1<=10'd0; Circuit_short<=1'b0; end
  else
   begin
	 if(count_en)
	  begin
	   if(!CountFull)
		 begin
	     if(ProTect_neg)
		   begin
		    if(count1<10'd399)
		     begin count1<=count1+1'b1; Circuit_short<=1'b0; end
		    else
		     begin count1<=10'd0; Circuit_short<=1'b1; end
	      end
	     else
	      begin count1<=count1; Circuit_short<=Circuit_short; end
	    end
	   else
	    begin count1<=10'd0; Circuit_short<=1'b0; end
	  end
    else
	  begin count1<=10'd0; Circuit_short<=1'b0; end
	end
 end
 
always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin 
	 State_Circuit<=NormalState_INV; PWMEN<=1'b1; 
	end
  else
   begin
	 case(State_Circuit)
	  NormalState_Circuit:
	   begin
		 if(!Circuit_short)
		  begin State_Circuit<=NormalState_Circuit; PWMEN<=1'b1; end
		 else
		  begin State_Circuit<=ShortState_Circuit;PWMEN<=1'b0; end
		end
	  ShortState_Circuit:
	   begin
		 if(ResetD)
		  begin State_Circuit<=NormalState_Circuit; PWMEN<=1'b1; end
		 else
		  begin State_Circuit<=ShortState_Circuit;PWMEN<=1'b0; end
		end
	  default:
	   begin State_Circuit<=NormalState_Circuit; PWMEN<=1'b1; end
	 endcase
	end
 end
  
  
endmodule  