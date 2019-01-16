module COM_Filter(CLK_50M,Rst_n,COM1_IPF_C,COM1_IPF_D);
input CLK_50M,Rst_n,COM1_IPF_C;
output reg COM1_IPF_D;

reg  [9:0] COMFT;

always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin
	 COMFT<=10'b0;
	end
  else
   begin
    COMFT[0]<=COM1_IPF_C;COMFT[1]<=COMFT[0];COMFT[2]<=COMFT[1];COMFT[3]<=COMFT[2];COMFT[4]<=COMFT[3];
    COMFT[5]<=COMFT[4];COMFT[6]<=COMFT[5];COMFT[7]<=COMFT[6];COMFT[8]<=COMFT[7];COMFT[9]<=COMFT[8];
	end
 end
 
wire COMFT_w1,COMFT_w2;
wire [1:0] COMFT_w;

assign COMFT_w1= COMFT[0] & COMFT[1] & COMFT[2] & COMFT[3] & COMFT[4] & COMFT[5] & COMFT[6] & COMFT[7] & COMFT[8] & COMFT[9];
assign COMFT_w2= (~COMFT[0]) & (~COMFT[1]) & (~COMFT[2]) & (~COMFT[3]) & (~COMFT[4]) & (~COMFT[5]) & (~COMFT[6]) & (~COMFT[7]) & (~COMFT[8]) & (~COMFT[9]);
assign COMFT_w={COMFT_w1,COMFT_w2} ;
 
always @(posedge CLK_50M)
 begin
  if(!Rst_n)
   begin
	 COM1_IPF_D<=1'b0;
	end
  else
   begin
	 case(COMFT_w)
	  2'b10:
	   begin COM1_IPF_D<=1'b1; end
	  2'b01:
	   begin COM1_IPF_D<=1'b0; end
	  2'b00,2'b11:
	   begin COM1_IPF_D<=COM1_IPF_D; end
	  default:
	   begin COM1_IPF_D<=COM1_IPF_D; end
	 endcase
	end
 end
 
 
 endmodule
 
	  
	   

