function [R_r_gt_rel, t_r_gt_rel] = get_rel_Rt(R_l_gt, t_l_gt, R_r_gt, t_r_gt)

    [O_l_gt, P_l_gt] = switch_coord_sys(R_l_gt, t_l_gt);
    [O_r_gt, P_r_gt] = switch_coord_sys(R_r_gt, t_r_gt);
    
    O_r_gt_rel = O_l_gt'*O_r_gt;
    P_r_gt_rel = P_r_gt-P_l_gt;
    
    [R_r_gt_rel, t_r_gt_rel] = switch_coord_sys(O_r_gt_rel, P_r_gt_rel);
end