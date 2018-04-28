function gt_stereo_Rt(R_l_gt, t_l_gt, R_r_gt, t_r_gt, cam_l, cam_r, im_l, im_r,...
                        matched_pts_l, matched_pts_r)

    K_l = cam_l.IntrinsicMatrix';
    K_r = cam_r.IntrinsicMatrix';

    [R_r_gt_rel, t_r_gt_rel] = get_rel_Rt(R_l_gt, t_l_gt, R_r_gt, t_r_gt);
    
    E = toSkew(t_r_gt_rel)*R_r_gt_rel;
    F = inv(K_r')*E*inv(K_l);
    
    draw_epipolar_lines(F, matched_pts_l, matched_pts_r, im_l, im_r);
    
end