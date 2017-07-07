module match_the_brick(clk, x,y, i,j);
input clk;
input [9:0]x,y;
output[2:0]i,j;
reg [2:0] i,j;
always@(posedge clk)
begin
	if(y>30&&y<70) i<=0;
	else if(y>70&&y<110) i<=1;
	else if(y>110&&y<150)i<=2;
	else if(y>150&&y<190)i<=3;
	else i<=0;
	
	if(x>25&&x<125) j<=0;
	else if(x>125&&x<225) j<=1;
	else if(x>225&&x<325) j<=2;
	else if(x>325&&x<425) j<=3;
	else if(x>425&&x<525) j<=4;
	else if(x>525&&x<625) j<=5;
	else j<=0;
end

endmodule
