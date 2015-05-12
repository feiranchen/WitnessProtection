  //#############################################################################
  // MODIFIED BY ANNIE (WEI) DAI TO DRAW A CONTINUOUS LINE ON A DISCRETE GRID
  // BY SPECIFYING THE BEGINNING AND ENDING COORDINATES OF THE LINE
  // SPRING 2013
  //#############################################################################
  ///////////////////////////////////////////////////////////////////////
  //// HARDWARE IMPLEMENTATION OF BRESENHAN LINE DRAWING ALGORITHM///////
  //////////////////////////////////////////////////////////////////////
  module line (
			// inputs:
			iX1,
			iX2,
			iY1,
			iY2,
			iclk,
			ireset,
			iFLAG,
			ipixeldone,
			iHS,
			iVS,
			icolor,


			// outputs:
			oaddr_reg,
			odata_reg,
			opixelflag,
			odoneflag		
             );


input	[9:0]	iX1,iX2;
input [8:0] iY1,iY2;
input	iFLAG, iclk, ipixeldone, ireset, iHS, iVS;
input	[15:0] 	icolor;
output	[18:0]	oaddr_reg;
output	[15:0]	odata_reg;
output	odoneflag, opixelflag;

reg	[18:0]	addr_reg;
reg	[15:0]	data_reg;
//wire	flag;
(* preserve *)reg	[3:0]	state;

assign	oaddr_reg = addr_reg;
assign	odata_reg = data_reg;

reg 	[18:0]	count;
wire	[10:0] 	diffx;
wire  [9:0]    diffy;

wire	[10:0]	dx;
wire  [9:0]    dy;
reg	[9:0] 	temp,  x;
reg   [8:0]    y;
reg [10:0]	tempdx;
reg [9:0]   tempdy;
reg	xchange;
reg	[10:0]	half;
reg	lock;
reg done;
reg	pixelflag;

assign odoneflag = done;
assign opixelflag = pixelflag;

assign	diffx = iX2-iX1;
assign dx = (diffx[10])? -diffx:diffx;
assign	diffy = iY2-iY1;
assign dy = (diffy[9])? -diffy:diffy;

//state names
parameter	init		= 4'd0,
		drawline1	= 4'd1,
		drawline2	= 4'd2,
		drawline3	= 4'd3,
		drawline4	= 4'd4,
		putpixel1	= 4'd5,
		finish		= 4'd7;


always @ (posedge iclk)
begin
	if (ireset)
	begin
		x <= iX1;
		y <= iY1;
		tempdx <= dx;
		temp <= dx;
		tempdy <= dy;
		xchange <= 1'd0;
		count <= 11'd0;
		pixelflag <= 1'd0;
		half <= 11'd0;

		done <= 1'd0;
		state <= init;
	end
	else if ((~iHS | ~iVS))
	begin
		if (iFLAG)
		begin
		case(state)
			init:
			begin
				x <= iX1;
				y <= iY1;
				tempdx <= dx;
				temp <= dx;
				tempdy <= dy;
				xchange <= 1'd0;
				count <= 11'd0;
				pixelflag <= 1'd0;
				half <= 11'd0;

				done <= 1'd0;
				state <= drawline1;
			end

			drawline1:
			begin
				if(tempdy > tempdx)
				begin
					tempdx <= dy;
					tempdy <= temp;
					xchange <= 1'd1;
				end
				state <= drawline2;
			end
			
			drawline2:
			begin
				half <= (tempdy << 1) - tempdx;
				state <= drawline3;
			end

			drawline3:
			begin
				if (count <= (tempdx+tempdy))
				begin
					state <= putpixel1;
				end
				else
					state <= finish;
			end
			
			drawline4:
			begin
				if (~half[10])
				begin
					if (xchange)
					begin
						if (diffx[10])
							x <= x-1'd1;
						else
							x <= x+1'd1;
					end
					else	
					begin
						if (diffy[9])
							y <= y-1'd1;
						else
							y <= y+1'd1;
					end
					half <= half-(tempdx << 1);
				end

				else
				begin
					if (xchange) 
					begin
						if (diffy[9])
							y <= y-1'd1;
						else
							y <= y+1'd1;
					end
					else	
					begin
						if (diffx[10])
							x <= x-1'd1;
						else
							x <= x+1'd1;
					end
					half <= half+(tempdy << 1);
				end
				count <= count+1'd1;
				state <= drawline3;
			end
			
			putpixel1:
			begin
				if (ipixeldone)
				begin
					pixelflag <= 1'd0;
					state <= drawline4;
				end
				else
				begin
					pixelflag <= 1'd1;
					addr_reg <= {x,y};
					data_reg <= icolor;
				end
			end				
			finish:
			begin
				done <= 1'd1;
			end
		endcase
		end
		else
		begin
			done <= 1'd0;
			state <= init;
		end
	end
end

endmodule