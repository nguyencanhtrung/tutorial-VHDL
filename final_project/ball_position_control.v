module ball_position_control(reset, clk_update, state, ball_x, ball_y, dx,dy, paddle_x,paddle_y);
input reset,clk_update;
input [3:0]state;
input dx,dy;
input [9:0]paddle_x,paddle_y;
output [9:0]ball_x, ball_y;
reg [9:0]ball_x,  ball_y;

always @(posedge clk_update or posedge reset)
begin
	if(reset)
		begin
			ball_x<=paddle_x;
			ball_y<=paddle_y-10;
		end
	else
			case(state)
			0:
			begin
				ball_x<=paddle_x;
				ball_y<=paddle_y-10;
			end
			1:
			begin
				ball_x<=ball_x-dx;
				ball_y<=ball_y-dy;
			end
			2:
			begin
				ball_x<=ball_x+dx;
				ball_y<=ball_y-dy;
			end
			3:
			begin
				ball_x<=ball_x+dx;
				ball_y<=ball_y+dy;
			end
			4:
			begin
				ball_x<=ball_x-dx;
				ball_y<=ball_y+dy;
			end
			endcase
end
endmodule
