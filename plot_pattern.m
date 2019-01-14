function plot_pattern( pat )
	
p1 = pat(:,1);
p2 = pat(:,2);
p3 = pat(:,3);
p4 = pat(:,4);


plot3( p1(1),  p1(2),  p1(3),  'ro',  'MarkerSize', 8, 'MarkerFaceColor', 'r' );
plot3( p2(1),  p2(2),  p2(3),  'go',  'MarkerSize', 8, 'MarkerFaceColor', 'g' );
plot3( p3(1),  p3(2),  p3(3),  'bo',  'MarkerSize', 8, 'MarkerFaceColor', 'b' );
plot3( p4(1),  p4(2),  p4(3),  'ko',  'MarkerSize', 8, 'MarkerFaceColor', 'k' );


c1 = pat(:,5);
c2 = pat(:,6);
c3 = pat(:,7);
c4 = pat(:,8);

plot3( [ c1(1)  c2(1)  c3(1)  c4(1) c1(1)  ],   [ c1(2)  c2(2)  c3(2)  c4(2) c1(2)  ],  [ c1(3)  c2(3)  c3(3)  c4(3) c1(3) ],  'k-' , 'LineWidth', 2  );