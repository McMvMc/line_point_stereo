function show_match(im_l, im_r, pts_l, pts_r, valid_pts_l, valid_pts_r)
    
    figure(1), title('left match');
    imshow(im_l), hold on;
    plot(pts_l.Location(:,1), pts_l.Location(:,2), 'b+');
    plot(valid_pts_l.Location(:,1), valid_pts_l.Location(:,2), 'r+'), hold on;

    figure(2), title('right match');
    imshow(im_r), hold on;
    plot(pts_r.Location(:,1), pts_r.Location(:,2), 'b+');
    plot(valid_pts_r.Location(:,1), valid_pts_r.Location(:,2), 'r+'), hold on;
   
end