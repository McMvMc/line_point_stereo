function compare_opti(prev_im_l, prev_im_r, im_l, im_r, ...
                            prev_pts_l, prev_pts_r, prev_valid_pts_l, prev_valid_pts_r,...
                                pts_l, pts_r, valid_pts_l, valid_pts_r)
    
    figure(2);
    subplot(2, 2, 1), imshow(prev_im_l), hold on
    plot(prev_pts_l(:,1), prev_pts_l(:,2), 'b+');
    plot(prev_valid_pts_l(:,1), prev_valid_pts_l(:,2), 'r+');
    hold off
    
    subplot(2, 2, 2), imshow(prev_im_r), hold on
    plot(prev_pts_r(:,1), prev_pts_r(:,2), 'b+');
    plot(prev_valid_pts_r(:,1), prev_valid_pts_r(:,2), 'r+');
    hold off
    
    
    subplot(2, 2, 3), imshow(im_l), hold on
    plot(pts_l(:,1), pts_l(:,2), 'b+');
    plot(valid_pts_l(:,1), valid_pts_l(:,2), 'r+');
    hold off
    
    subplot(2, 2, 4), imshow(im_r), hold on
    plot(pts_r(:,1), pts_r(:,2), 'b+');
    plot(valid_pts_r(:,1), valid_pts_r(:,2), 'r+');
    hold off
   
end