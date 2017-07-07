module decide_direction(clk,reset,ball_x, baffle_x,situation);
input [9:0] ball_x;
input [9:0] baffle_x;
input clk;
input reset;
output [3:0] situation;
reg [3:0] situation;
always@(posedge clk)
if(reset)
begin situation<=1;end
else 
begin
	if( (ball_x<baffle_x-46&&ball_x>=baffle_x-50) || (ball_x>baffle_x+46&&ball_x<=baffle_x+50) ) situation<=9;
	else if( (ball_x<baffle_x-42&&ball_x>=baffle_x-46) || (ball_x>baffle_x+42&&ball_x<=baffle_x+46) ) situation<=8;
	else if( (ball_x<baffle_x-35&&ball_x>=baffle_x-42) || (ball_x>baffle_x+35&&ball_x<=baffle_x+46) ) situation<=7;
	else if( (ball_x<baffle_x-15&&ball_x>=baffle_x-35) || (ball_x>baffle_x+15&&ball_x<=baffle_x+35) ) situation<=1;
	else if( (ball_x<baffle_x-8&&ball_x>=baffle_x-15) || (ball_x>baffle_x+8&&ball_x<=baffle_x+15) ) situation<=2;
	else if( (ball_x<baffle_x-4&&ball_x>=baffle_x-8) || (ball_x>baffle_x+4&&ball_x<=baffle_x+8) ) situation<=3;
	else if( (ball_x<baffle_x-0&&ball_x>=baffle_x-4) || (ball_x>baffle_x+0&&ball_x<=baffle_x+4) ) situation<=4;
	else situation<=1;
end
endmodule
	