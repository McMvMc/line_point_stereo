% reads out body frame O P, and convert to R t (left to right)
% T_BS is from sensor to world, in Direct cosine matrix and P.
% https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets#ground-truth
function [R_r_gt_rel, t_r_gt_rel, cam_l_param, cam_r_param] = ...
                                    read_gt(img_path_l, img_path_r)
    [O_l_b_gt, P_l_b_gt, cam_l_param] = load_cam(strcat(img_path_l, '../sensor.yaml'));
    [O_r_b_gt, P_r_b_gt, cam_r_param] = load_cam(strcat(img_path_r, '../sensor.yaml'));
    [R_l_b_gt, t_l_b_gt] = switch_coord_sys(O_l_b_gt, P_l_b_gt);
    [R_r_b_gt, t_r_b_gt] = switch_coord_sys(O_r_b_gt, P_r_b_gt);
    [R_r_b_gt_rel, t_r_b_gt_rel] = get_rel_Rt(R_l_b_gt, t_l_b_gt, R_r_b_gt, t_r_b_gt);
    [O_r_b_gt_rel, P_r_b_gt_rel] = switch_coord_sys(R_r_b_gt_rel, t_r_b_gt_rel);

    % O and P in body frame,   ^ x    camera frame x <- |
    %                       y<-|                      y v
    
    % body to sensor
    T_b2s = O_l_b_gt';
%     O_r_s_gt_rel = T_b2s*O_r_b_gt_rel;
    P_r_s_gt_rel = T_b2s*P_r_b_gt_rel;
    
    [R_r_gt_rel, t_r_gt_rel] = switch_coord_sys(O_r_b_gt_rel, P_r_s_gt_rel);
end