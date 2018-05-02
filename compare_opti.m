function compare_opti(cur_3D_pts, cam_l_param, cam_r_param, prev_im_l, prev_im_r,...
                            im_l, im_r, prev_O_l_g, prev_P_l_g, ...
                            O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel,...
                            prev_matched_pts_l, prev_matched_pts_r,...
                            matched_pts_l, matched_pts_r)

    [prev_R_l_g, prev_t_l_g] = switch_coord_sys(prev_O_l_g, prev_P_l_g);
    [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
    [prev_O_r_g, prev_P_r_g] = right_from_left_cam(prev_O_l_g, prev_P_l_g, ...
                                                R_r_gt_rel, t_r_gt_rel);
    [prev_R_r_g, prev_t_r_g] = switch_coord_sys(prev_O_r_g, prev_P_r_g);
    [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
    [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);
    proj_l = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                         cam_l_param.IntrinsicMatrix', R_l_g, t_l_g)';
    proj_r = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                         cam_r_param.IntrinsicMatrix', R_r_g, t_r_g)';
    proj_prev_l = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                         cam_l_param.IntrinsicMatrix', prev_R_l_g, prev_t_l_g)';
    proj_prev_r = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                         cam_r_param.IntrinsicMatrix', prev_R_r_g, prev_t_r_g)';

    figure(2);
    clf(2);
    figure(2);
    subplot(2, 2, 1), imshow(prev_im_l), hold on
    plot(prev_matched_pts_l(:,1), prev_matched_pts_l(:,2), 'b+');
    plot(proj_prev_l(:,1), proj_prev_l(:,2), 'r+');
    hold off
    
    subplot(2, 2, 2), imshow(prev_im_r), hold on
    plot(prev_matched_pts_r(:,1), prev_matched_pts_r(:,2), 'b+');
    plot(proj_prev_r(:,1), proj_prev_r(:,2), 'r+');
    hold off
    
    
    subplot(2, 2, 3), imshow(im_l), hold on
    plot(matched_pts_l(:,1), matched_pts_l(:,2), 'b+');
    plot(proj_l(:,1), proj_l(:,2), 'r+');
    hold off
    
    subplot(2, 2, 4), imshow(im_r), hold on
    plot(matched_pts_r(:,1), matched_pts_r(:,2), 'b+');
    plot(proj_r(:,1), proj_r(:,2), 'r+');
    hold off
   
end