function draw_epipolar_lines(F, pts_l, pts_r, im_l, im_r)

    l_l = epipolarLine(F',pts_r);
    line_pts_l = lineToBorderPoints(l_l,size(im_l));

    l_r = epipolarLine(F,pts_l);
    line_pts_r = lineToBorderPoints(l_r,size(im_r));

    % draw
    figure(1), title('left & right');
    subplot(1,2,1), imshow(im_l), title('left'), hold on;
    plot(pts_l(:,1), pts_l(:,2), 'g+');
    line(line_pts_l(:,[1,3])',line_pts_l(:,[2,4])');
    hold off
    
    subplot(1,2,2), imshow(im_r), title('right'), hold on;
    plot(pts_r(:,1), pts_r(:,2), 'g+');
    line(line_pts_r(:,[1,3])',line_pts_r(:,[2,4])');
    hold off

end