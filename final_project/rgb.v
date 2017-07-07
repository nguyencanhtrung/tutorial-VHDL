module rgb(reset,clk,x,y,baffle_x,baffle_y,baffle_l,ball_x,ball_y,vga_r,vga_g,vga_b,
				color00,color01,color02,color03,color04,color05,
				color10,color11,color12,color13,color14,color15,
				color20,color21,color22,color23,color24,color25,
				color30,color31,color32,color33,color34,color35);
input reset,clk;
input [9:0] color00,color01,color02,color03,color04,color05,
				color10,color11,color12,color13,color14,color15,
				color20,color21,color22,color23,color24,color25,
				color30,color31,color32,color33,color34,color35;
input[9:0] x,y,baffle_x, baffle_y, baffle_l;
input[9:0] ball_x,ball_y;

output [9:0]vga_r,vga_g,vga_b;
reg [9:0]vga_r,vga_g,vga_b;
parameter radius=3;

wire i;
wire [1:0] j;

assign i=(y>=35&& y<=85)? 0:1;
assign j= (x>=25&& x<=135)? 0: ((x>=185&& x<=295)? 1: ((x>=345&& x<=455)?2:3 ));
always@(posedge clk or posedge reset)
begin
	if(reset==1)
		begin
			vga_r<=0;vga_g<=0;vga_b<=0;
		end
	else 
		begin
		   // show the ball, and the ball will forever be in the front layer
			if ((x>ball_x-radius)&&(x<ball_x+radius)&&(y>ball_y-radius)&&(y<ball_y+radius) )
			begin
				vga_r<=10'h3ff;vga_g<=0;vga_b<=0;
			end
			// show bricks
			else if((y>=40)&&(y<=60))
			begin		
				if( x>=50&&x<=100)
					begin
						vga_r<=0;vga_g<=color00;vga_b<=0;
					end
				else if((x>=150&&x<=200))
					begin
						vga_r<=0;vga_g<=color01;vga_b<=0;
					end
				else if((x>=250&&x<=300))
					begin
						vga_r<=0;vga_g<=color02;vga_b<=0;
					end
				else if((x>=350&&x<=400))
					begin
						vga_r<=0;vga_g<=color03;vga_b<=0;
					end
				else if((x>=450&&x<=500))
					begin
						vga_r<=0;vga_g<=color04;vga_b<=0;
					end
				else if((x>=550&&x<=600))
					begin
						vga_r<=0;vga_g<=color05;vga_b<=0;
					end
				else 
					begin
						vga_r<=0;vga_g<=0;vga_b<=0;
					end
			end
			
			else if((y>=80)&&(y<=100) )
			begin		
				if( x>=50&&x<=100)
					begin
					   vga_r<=color10;vga_g<=0;vga_b<=0;
					end
				else if((x>=150&&x<=200))
					begin
						vga_r<=color11;vga_g<=0;vga_b<=0;
					end
				else if((x>=250&&x<=300))
					begin
						vga_r<=color12;vga_g<=0;vga_b<=0;
					end
				else if((x>=350&&x<=400))
					begin
						vga_r<=color13;vga_g<=0;vga_b<=0;
					end
				else if((x>=450&&x<=500))
					begin
						vga_r<=color14;vga_g<=0;vga_b<=0;
					end
				else if((x>=550&&x<=600))
					begin
						vga_r<=color15;vga_g<=0;vga_b<=0;
					end
				else 
					begin
						vga_r<=0;vga_g<=0;vga_b<=0;
					end
			end
			 
			 else if((y>=120)&&(y<=140) )
			begin		
				if( x>=50&&x<=100)
					begin
						vga_r<=0;vga_g<=0;vga_b<=color20;
					end
				else if((x>=150&&x<=200))
					begin
						vga_r<=0;vga_g<=0;vga_b<=color21;
					end
				else if((x>=250&&x<=300))
					begin
						vga_r<=0;vga_g<=0;vga_b<=color22;
					end
				else if((x>=350&&x<=400))
					begin
						vga_r<=0;vga_g<=0;vga_b<=color23;
					end
				else if((x>=450&&x<=500))
					begin
						vga_r<=0;vga_g<=0;vga_b<=color24;
					end
				else if((x>=550&&x<=600))
					begin
						vga_r<=0;vga_g<=0;vga_b<=color25;
					end
				else 
					begin
						vga_r<=0;vga_g<=0;vga_b<=0;
					end
			end
			 
			else if((y>=160)&&(y<=180) )
			begin		
				if( x>=50&&x<=100)
					begin
						vga_r<=color30;vga_g<=color30;vga_b<=0;
					end
				else if((x>=150&&x<=200))
					begin
						vga_r<=color31;vga_g<=color31;vga_b<=0;
					end
				else if((x>=250&&x<=300))
					begin
						vga_r<=color32;vga_g<=color32;vga_b<=0;
					end
				else if((x>=350&&x<=400))
					begin
						vga_r<=color33;vga_g<=color33;vga_b<=0;
					end
				else if((x>=450&&x<=500))
					begin
						vga_r<=color34;vga_g<=color34;vga_b<=0;
					end
				else if((x>=550&&x<=600))
					begin
						vga_r<=color35;vga_g<=color35;vga_b<=0;
					end
				else 
					begin
						vga_r<=0;vga_g<=0;vga_b<=0;
					end
			end
			 
//			 end
			 // show baffle
			 else if((y>baffle_y-10)&&(y<baffle_y))
			 begin
			 	if((x>baffle_x-baffle_l+6)&&(x<baffle_x+baffle_l) )
					begin
						vga_r<=10'h1f;vga_g<=10'h3f;vga_b<=10'h3a0;
					end
				else if(y==baffle_y-5)
					begin
						if(x[1:0]==3) begin vga_r<=10'h3ff;vga_g<=10'h3ff;vga_b<=10'h3ff; end
						else begin vga_r<=0;vga_g<=0;vga_b<=0; end
					end
				else 
					begin
					   vga_r<=0;vga_g<=0;vga_b<=0;
					end
			 end
			 
			 else
			 begin
			   vga_r<=0;vga_g<=0;vga_b<=0;
			 end					  
		 end
end
endmodule 