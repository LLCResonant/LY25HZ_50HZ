module LED_drive(BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2,
                 LED1,LED2,LED3,LED4,LED5,LED6);
input BusOvp,IP_Ocp,InvOcp1,OP_Ovp1,InvOcp2,OP_Ovp2;
output LED1,LED2,LED3,LED4,LED5,LED6;

assign LED1=BusOvp; assign LED2=IP_Ocp; assign LED3=InvOcp1;
assign LED4=OP_Ovp1; assign LED5=InvOcp2; assign LED6=OP_Ovp2;

//assign LED1=1'b1; assign LED2=1'b1; assign LED3=1'b1;
//assign LED4=1'b1; assign LED5=1'b1; assign LED6=1'b1;

endmodule
