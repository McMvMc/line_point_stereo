% pts_3d -- N x 3
% *_pts -- N x 2
function [O_l, P_l, pts_3d, fval] = refineRt(prev_O_l_g, prev_P_l_g, O_l_g, P_l_g,...
                              prev_l_pts, prev_r_pts, l_pts, r_pts,...
                              cam_l, cam_r, ...
                              pts_3d, R_r_gt_rel, t_r_gt_rel)
pts_3d = double(pts_3d');
prev_l_pts = prev_l_pts'; 
prev_r_pts = prev_r_pts';
l_pts = l_pts';
r_pts = r_pts';

K_l = cam_l.IntrinsicMatrix';
K_r = cam_r.IntrinsicMatrix';

[prev_R_l_g, prev_t_l_g] = switch_coord_sys(prev_O_l_g, prev_P_l_g);
[R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);

% use quaternion to avoid gimbal locking
q_l_g = rotm2quat(R_l_g);
q_t = [q_l_g, t_l_g'];

%do the minimization
for i = 1:1
    display('optimizing camera pose')
    [q_t,fval,~,~] = fminsearch (@optimize_camera, q_t, ...
                       optimset ('MaxFunEvals', 10000, ...
                                 'MaxIter', 100000, ....
                                 'Algorithm', 'levenberg-marquardt',...
                                 'Display', 'iter'...
                                ), ...
                       prev_R_l_g, prev_t_l_g, K_l, K_r, R_r_gt_rel, t_r_gt_rel, ...
                       pts_3d, prev_l_pts, prev_r_pts, l_pts, r_pts);

%     display('optimizing 3D points')
%     [pts_3d,fval,~,~] = fminsearch (@optimize_points, pts_3d, ...
%                        optimset ('MaxFunEvals', 10000, ...
%                                  'MaxIter', 100000, ....
%                                  'Algorithm', 'levenberg-marquardt',...
%                                  'Display', 'iter'...
%                                 ), ...
%                        prev_R_l_g, prev_t_l_g, K_l, K_r, R_r_gt_rel, t_r_gt_rel, ...
%                        q_t, prev_l_pts, prev_r_pts, l_pts, r_pts);
end
               
q_l = q_t(1:4);
R_l = quat2rotm(q_l);
t_l = q_t(5:7)';

[O_l, P_l] = switch_coord_sys(R_l, t_l);
pts_3d = single(pts_3d');
end


% pts_3d -- 3 x N
% *_pts -- 2 x N
function d = optimize_camera (q_t, prev_R_l, prev_t_l, K_l, K_r, ...
                R_r_gt_rel, t_r_gt_rel, pts_3d, prev_l_pts, prev_r_pts, l_pts, r_pts)
    q_l = q_t(1:4);
    t_l = q_t(5:7)';
    R_l = quat2rotm(q_l);
    
    pts_3d = [pts_3d; ones(1, size(pts_3d, 2))];

    % get previous R and ts for left and right cam
    [prev_O_l, prev_P_l] = switch_coord_sys(prev_R_l, prev_t_l);
    [prev_O_r, prev_P_r] = right_from_left_cam(prev_O_l, prev_P_l, ...
                                                        R_r_gt_rel, t_r_gt_rel);
    [prev_R_r, prev_t_r] = switch_coord_sys(prev_O_r, prev_P_r);

    % get current R and ts for left and right cam
    [O_l, P_l] = switch_coord_sys(R_l, t_l);
    [O_r, P_r] = right_from_left_cam(O_l, P_l, R_r_gt_rel, t_r_gt_rel);
    [R_r, t_r] = switch_coord_sys(O_r, P_r);

    prev_l_proj = project_to_cam(pts_3d, K_l, prev_R_l, prev_t_l);
    prev_r_proj = project_to_cam(pts_3d, K_r, prev_R_r, prev_t_r);
    l_proj = project_to_cam(pts_3d, K_l, R_l, t_l);
    r_proj = project_to_cam(pts_3d, K_r, R_r, t_r);

    dist_prev_l = vecnorm(prev_l_pts - prev_l_proj, 2, 1);
    dist_prev_r = vecnorm(prev_r_pts - prev_r_proj, 2, 1);
    dist_l = vecnorm(l_pts - l_proj, 2, 1);
    dist_r = vecnorm(r_pts - r_proj, 2, 1);


    d = sum(dist_prev_l + dist_prev_r + dist_l + dist_r);
end

% pts_3d -- 3 x N
% *_pts -- 2 x N
function d = optimize_points (pts_3d, prev_R_l, prev_t_l, K_l, K_r, ...
                R_r_gt_rel, t_r_gt_rel, q_t, prev_l_pts, prev_r_pts, l_pts, r_pts)
    q_l = q_t(1:4);
    t_l = q_t(5:7)';
    R_l = quat2rotm(q_l);
    
    pts_3d = [pts_3d; ones(1, size(pts_3d, 2))];

    % get previous R and ts for left and right cam
    [prev_O_l, prev_P_l] = switch_coord_sys(prev_R_l, prev_t_l);
    [prev_O_r, prev_P_r] = right_from_left_cam(prev_O_l, prev_P_l, ...
                                                        R_r_gt_rel, t_r_gt_rel);
    [prev_R_r, prev_t_r] = switch_coord_sys(prev_O_r, prev_P_r);

    % get current R and ts for left and right cam
    [O_l, P_l] = switch_coord_sys(R_l, t_l);
    [O_r, P_r] = right_from_left_cam(O_l, P_l, R_r_gt_rel, t_r_gt_rel);
    [R_r, t_r] = switch_coord_sys(O_r, P_r);
    
    prev_l_proj = project_to_cam(pts_3d, K_l, prev_R_l, prev_t_l);
    prev_r_proj = project_to_cam(pts_3d, K_r, prev_R_r, prev_t_r);
    l_proj = project_to_cam(pts_3d, K_l, R_l, t_l);
    r_proj = project_to_cam(pts_3d, K_r, R_r, t_r);

    dist_prev_l = vecnorm(prev_l_pts - prev_l_proj, 2, 1);
    dist_prev_r = vecnorm(prev_r_pts - prev_r_proj, 2, 1);
    dist_l = vecnorm(l_pts - l_proj, 2, 1);
    dist_r = vecnorm(r_pts - r_proj, 2, 1);

    d = sum(dist_prev_l + dist_prev_r + dist_l + dist_r);
end
