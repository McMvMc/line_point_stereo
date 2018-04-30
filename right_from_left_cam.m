function [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel)
    R_r_g = R_r_gt_rel;
    t_r_g = t_r_gt_rel;
    [to_r_cam_O, to_r_cam_P] = switch_coord_sys(R_r_g, t_r_g);
    
    O_r_g = to_r_cam_O*O_l_g;
    P_r_g = P_l_g + to_r_cam_P;
end