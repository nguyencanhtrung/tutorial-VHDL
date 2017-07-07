module dxdy_control(clk, reset, situation, dx,dy);
input clk;
input reset;
input [3:0] situation;
output dx,dy;
reg dx,dy;
integer i;
reg [11:0] a1,a2,a3,a4,a6,a12,b;
always @( posedge clk or posedge reset)  
begin
	if(reset)
		begin
		  a12<=12'b000000100000;
			a6<=12'b001000001000;
			a4<=12'b010001000100;
			a3<=12'b010010010010;
			a2<=12'b011001100110;
			a1<=12'b111111111111;			
		   b<=12'b111111111111;		
			i<=0;
		end
	else
		begin
			case(situation)
			1:              
			begin
				dx<=a12[i]; dy<=b[i];
			end
			2:               
			begin
				dx<=a6[i]; dy<=b[i];
			end
			3:              
			begin
				dx<=a4[i]; dy<=b[i];
			end
			4:              
			begin
				dx<=a3[i]; dy<=b[i];
			end
			5:               
			begin
				dx<=a2[i]; dy<=b[i];
			end
		
			6:            // 45 angle      
			begin
				dx<=a1[i]; dy<=b[i];
			end
			/////////////////////
			7:               
			begin
				dy<=a2[i]; dx<=b[i];
			end
			8:              
			begin
				dy<=a3[i]; dx<=b[i];
			end
			9:               
			begin
				dy<=a4[i]; dx<=b[i];
			end
			10:              
			begin
				dy<=a6[i]; dx<=b[i];
			end
			11:               
			begin
				dy<=a12[i]; dx<=b[i];
			end
			default:
			begin
				dy<=1;
				dx<=1;
			end
			
			endcase
			
			if(i==11) i<=0;
			else i<=i+1;
		end
		
end
endmodule			
			
			
			
			
			
			
			
			
			
			
			
			
			
			
			