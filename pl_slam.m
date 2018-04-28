addpath('~/YAMLMatlab_0.4.3');

img_path_l = '~/Desktop/833/project/mav0/cam0/data/';
img_path_r = '~/Desktop/833/project/mav0/cam1/data/';

% reads out body frame O P, and convert to R t (left to right)
[R_r_gt_rel, t_r_gt_rel, cam_l_param, cam_r_param] = ...
                            read_gt(img_path_l, img_path_r);

img_list_l = dir(strcat(img_path_l, '*.png'));
img_list_r = dir(strcat(img_path_r, '*.png'));

camera_list = {camera_obj()};

O_l_g = eye(3);
P_l_g = zeros(3,1);

for idx=1:size(img_list_l)
    tic()
    
% ---- tracking ---- %
    [im_l, im_r] = get_images(strcat(img_path_l, img_list_l(idx).name), ...
                              strcat(img_path_r, img_list_r(idx).name), ...
                              cam_l_param, cam_r_param);

    [desc_l, valid_pts_l] = detect_points(im_l);
%     [desc_r, valid_pts_r] = detect_points(im_r);

    % matching
    % match_points()

    % tracking
    [matched_pts_l, matched_pts_r] = track_points(valid_pts_l, im_l, im_r);
    size(matched_pts_l)

    % show_detection(im_l, im_r, matched_pts_l(1,:), matched_pts_r(1,:), matched_pts_l, matched_pts_r);
    
    
% ---- calculate camera pose ---- %    
    % output R t matlab convention -- transposed relative to cv convention
    [F, matched_pts_l, matched_pts_r, R_r_g, t_r_g]...
                = calc_cam_poses(matched_pts_l, matched_pts_r, cam_l_param, cam_r_param);
    size(matched_pts_l)

    % visualization and comparison
    draw_epipolar_lines(F, matched_pts_l, matched_pts_r, im_l, im_r);
    gt_stereo_Rt(eye(3), zeros(3,1), R_r_gt_rel, t_r_gt_rel, ...
        cam_l_param, cam_r_param, im_l, im_r, matched_pts_l, matched_pts_r);
    

% ---- triangulate ---- %
    % gt is in cv convention
    R_r_g = R_r_gt_rel;
    t_r_g = t_r_gt_rel;
    prev_cam = camera_list{end};
    [to_r_cam_O, to_r_cam_P] = switch_coord_sys(R_r_g, t_r_g);
    
    O_l_g = O_l_g * prev_cam.get_O();
    P_l_g = P_l_g + prev_cam.get_P();
    
    O_r_g = to_r_cam_O*O_l_g;
    P_r_g = P_l_g + to_r_cam_P;
    
    % world to cur cam coord
    [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
    [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);
    
    camMatrix_l = cameraMatrix(cam_l_param,R_l_g,t_l_g);
    camMatrix_r = cameraMatrix(cam_r_param,R_r_g,t_r_g);
    
    worldPoints = triangulate(matched_pts_l,matched_pts_r,camMatrix_l,camMatrix_r);
    
    draw_3D_pts(worldPoints, O_l_g, P_l_g, O_r_g, P_r_g)


    % [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints,pointTracks,cameraPoses,cam_l_param);

    toc()
    
end
% pointTracker = vision.PointTracker('MaxBidirectionalError', 3, 'BlockSize', 15, 'MaxIterations', 15);




