module Protect_delay(clk,Rst_n,Prt,Prt_dly);
input clk,Rst_n,Prt;
output reg Prt_dly;

reg [7:0] count1;
 
always @(posedge clk)
 begin 
  if(!Rst_n)
   begin count1<=8'd0; end
  else
   begin
	 if(!Prt)
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
 
always @(posedge clk)
 begin
  if(!Rst_n)
   begin Prt_dly<=1'b0; end
  else
   begin
	 if(count1!=8'd150)
	  begin Prt_dly<=1'b1; end
	 else
	  begin Prt_dly<=1'b0; end
	end
 end
 
 endmodule
 