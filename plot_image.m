function plot_image( img_points, fill )

if(fill == 1)
    plot( img_points(1,1), img_points(2,1), 'ro' , 'MarkerSize', 10, 'MarkerFaceColor', 'r' );
    plot( img_points(1,2), img_points(2,2), 'go' , 'MarkerSize', 10, 'MarkerFaceColor', 'g' );
    plot( img_points(1,3), img_points(2,3), 'bo' , 'MarkerSize', 10, 'MarkerFaceColor', 'b' );
    plot( img_points(1,4), img_points(2,4), 'ko' , 'MarkerSize', 10, 'MarkerFaceColor', 'k' );
else
    plot( img_points(1,1), img_points(2,1), 'ro' , 'MarkerSize', 12, 'LineWidth', 2 );
    plot( img_points(1,2), img_points(2,2), 'go' , 'MarkerSize', 12, 'LineWidth', 2 );
    plot( img_points(1,3), img_points(2,3), 'bo' , 'MarkerSize', 12, 'LineWidth', 2 );
    plot( img_points(1,4), img_points(2,4), 'ko' , 'MarkerSize', 12, 'LineWidth', 2 );
end    
    