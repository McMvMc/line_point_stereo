function show_detection(im_l, im_r, pts_l, pts_r, valid_pts_l, valid_pts_r)
    
    figure(1), title('left');
    imshow(im_l), hold on;
    plot(pts_l(:,1), pts_l(:,2), 'b+');
    plot(valid_pts_l(:,1), valid_pts_l(:,2), 'r+');
    hold off

    figure(2), title('right');
    imshow(im_r), hold on;
    plot(pts_r(:,1), pts_r(:,2), 'b+');
    plot(valid_pts_r(:,1), valid_pts_r(:,2), 'r+');
    hold off
end