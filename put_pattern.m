function pat = put_pattern( pose )
% pat = four points of the pattern	
	
O = pose(1:3,4);
x = pose(1:3,1);
y = pose(1:3,2);
z = pose(1:3,3);

d = 0.2;
px = O + d*x;
py = O + d*y;
pxy = O + d*x + d*y;

poo = O - 0.5*d*x - 0.5*d*y;
pxx = O + 1.5*d*x - 0.5*d*y;
pyy = O + 1.5*d*y - 0.5*d*x;
pxyxy = pxy + 0.5*d*x + 0.5*d*y;

pat = [ O, px, pxy, py,  poo, pxx, pxyxy, pyy ]; 
