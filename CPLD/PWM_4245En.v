module PWM_4245En(clk,Rst_n,OP_Rly1_D,OP_Rly2_D,OP_Scr1_D,OP_Scr2_D,COM1_OUT_D,COM2_OUT_D,En1,En2,OP_Rly1_C,OP_Rly2_C,OP_Scr1_C,OP_Scr2_C,COM1_OUT_C,COM2_OUT_C);
input clk,Rst_n;
input OP_Rly1_D,OP_Rly2_D,OP_Scr1_D,OP_Scr2_D;
input COM1_OUT_D,COM2_OUT_D;
output reg En1,En2;
output reg OP_Rly1_C,OP_Rly2_C,OP_Scr1_C,OP_Scr2_C;
output reg COM1_OUT_C,COM2_OUT_C;

reg r_COM1_OUT_D1,r_COM1_OUT_D11;


//always @(posedge clk)
//begin
// if(!Rst_n)
//  begin r_COM1_OUT_D1<=1'b0; r_COM1_OUT_D11<=1'b0; end
// else
//  begin r_COM1_OUT_D11<=r_COM1_OUT_D1; r_COM1_OUT_D1<=COM1_OUT_D; end
//end
//
//
//wire COM1_pos;
//assign COM1_pos=(~r_COM1_OUT_D11)&(r_COM1_OUT_D1);
//
//reg [2:0] count1=3'b0;
//always @(posedge clk)
// begin
//  if(!Rst_n)
//   begin count1<=3'b0; end
//  else
//   begin
//	 if(COM1_pos!=1'b0)
//	  begin
//	   if(count1<3'd2)
//	    begin count1<=count1+1'b1; end
//		else
//		 begin count1<=3'd2; end
//	  end
//	 else
//	  begin 
//	   count1<=count1;
//	  end
//	end
// end
//	   
 
reg [24:0] count=25'b0;

always @(posedge clk)
 begin
  if(!Rst_n)
   begin count<=25'b0; end
  else
   begin
	   if(count<25'd20000000)
	    begin 
	     count<=count+1'b1; En1<=1'b1; En2<=1'b0; 
		  OP_Rly1_C<=1'b0; OP_Rly2_C<=1'b0; OP_Scr1_C<=1'b1; OP_Scr2_C<=1'b1;
        COM1_OUT_C<=1'b1; COM2_OUT_C<=1'b1;
	    end
	   else
	    begin
	      count<=25'd20000000; En1<=1'b0; En2<=1'b0; 
	      OP_Rly1_C<=OP_Rly1_D; OP_Rly2_C<=OP_Rly2_D; OP_Scr1_C<=OP_Scr1_D; OP_Scr2_C<=OP_Scr2_D;
         COM1_OUT_C<=COM1_OUT_D; COM2_OUT_C<=COM2_OUT_D;
	    end
   end
 end


//always @(posedge clk)
// begin
//  if(!Rst_n)
//   begin count<=26'b0; end
//  else
//   begin
//	   if(count<26'd10000000)
//	    begin 
//	     count<=count+1'b1; En1<=1'b1; En2<=1'b0; 
//		  OP_Rly1_C<=1'b0; OP_Rly2_C<=1'b0; OP_Scr1_C<=1'b1; OP_Scr2_C<=1'b1;
//        COM1_OUT_C<=1'b1; COM2_OUT_C<=1'b0;
//	    end
//	   else
//		 if(count<26'd40000000)
//	     begin
//		   if(count1<3'd2) 
//		    begin 
//	        count<=count+1'b1; En1<=1'b1; En2<=1'b0; 
//		     OP_Rly1_C<=1'b0; OP_Rly2_C<=1'b0; OP_Scr1_C<=1'b1; OP_Scr2_C<=1'b1;
//           COM1_OUT_C<=1'b1; COM2_OUT_C<=1'b0;
//	       end
//		   else
//		    begin
//	        count<=count+1'b1; En1<=1'b0; En2<=1'b0; 
//	        OP_Rly1_C<=OP_Rly1_D; OP_Rly2_C<=OP_Rly2_D; OP_Scr1_C<=OP_Scr1_D; OP_Scr2_C<=OP_Scr2_D;
//           COM1_OUT_C<=COM1_OUT_D; COM2_OUT_C<=COM2_OUT_D;
//	       end
//	     end
//	    else
//	     begin
//	      count<=26'd40000000; En1<=1'b0; En2<=1'b0; 
//	      OP_Rly1_C<=OP_Rly1_D; OP_Rly2_C<=OP_Rly2_D; OP_Scr1_C<=OP_Scr1_D; OP_Scr2_C<=OP_Scr2_D;
//         COM1_OUT_C<=COM1_OUT_D; COM2_OUT_C<=COM2_OUT_D;
//	    end
//   end
// end
 
 
 endmodule
	