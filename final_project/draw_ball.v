module draw_ball(x,y, x_ball, y_ball, flag);
input [9:0] x,y;
input [9:0] x_ball, y_ball;
output flag;
reg flag;
always@(x or y or x_ball or y_ball)
begin
	if((x-x_ball)*(y-y_ball)< 10 )
	flag<=1;
	else flag<=0;
end

endmodule