YAML_LIB_PATH = '~/YAMLMatlab_0.4.3';
addpath(YAML_LIB_PATH);

img_path_l = '~/Desktop/833/project/mav0/cam0/data/';
img_path_r = '~/Desktop/833/project/mav0/cam1/data/';
step_size = 5;

% reads out body frame O P, and convert to R t (left to right)
[R_r_gt_rel, t_r_gt_rel, cam_l_param, cam_r_param] = ...
                            read_gt(img_path_l, img_path_r);

img_list_l = dir(strcat(img_path_l, '*.png'));
img_list_r = dir(strcat(img_path_r, '*.png'));

% ---- database ---- %
global_3D_pts = zeros(0,3);
global_descriptor = [];

cur_3D_pts = zeros(0,3);
cur_pt_ids = zeros(0,1);

camera_list = {camera_obj()};
% ---- database ---- %


need_redetect = false;
need_traingulate = false;

O_l_g = eye(3);
P_l_g = zeros(3,1);
prev_O_l_g = O_l_g;
prev_P_l_g = P_l_g;
% [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
% prev_O_r_g = O_r_g;
% prev_P_r_g = P_r_g;
                      
for idx=1:step_size:size(img_list_l)
    tic()
    
% ---- track features ---- %
    [im_l, im_r] = get_images(strcat(img_path_l, img_list_l(idx).name), ...
                              strcat(img_path_r, img_list_r(idx).name), ...
                              cam_l_param, cam_r_param);
    if need_redetect || idx==1
        [desc_l, matched_pts_l] = detect_points(im_l);
        matched_pts_l = matched_pts_l.Location;
        [matched_pts_l, matched_pts_r, validity] = track_points(matched_pts_l, ...
                                                        im_l, im_r);        
        
        need_traingulate = true;
    else
        % track between frames and between stereo
        % filters points
        [prev_matched_pts_l_filt, matched_pts_l_filt, valid_l] = ...
                                    track_points(prev_matched_pts_l, prev_im_l, im_l);
        [matched_pts_l_filt, matched_pts_r, valid_r] = track_points(matched_pts_l_filt, ...
                                                        im_l, im_r);
        prev_matched_pts_l_filt = prev_matched_pts_l_filt(valid_r, :);    

        avg_disparity = mean(sqrt(sum((prev_matched_pts_l_filt - ...
                                                matched_pts_l_filt).^2,2)));
        if avg_disparity>5
            prev_matched_pts_l = prev_matched_pts_l_filt;
            matched_pts_l = matched_pts_l_filt;
            cur_3D_pts = cur_3D_pts(valid_l,:);
            cur_3D_pts = cur_3D_pts(valid_r,:);
        else
            continue
        end    
    end
% ---- track features ---- %

% ---- track camera pose ---- %
    if idx ~= 1
        [O_l_g, P_l_g, inlier_idx] = estimateWorldCameraPose(matched_pts_l,...
                                                            cur_3D_pts,cam_l_param);
        P_l_g = P_l_g';
        matched_pts_l = matched_pts_l(inlier_idx, :);
        matched_pts_r = matched_pts_r(inlier_idx, :);
        cur_3D_pts = cur_3D_pts(inlier_idx, :);
        prev_matched_pts_l = prev_matched_pts_l(inlier_idx, :);
        prev_matched_pts_r = prev_matched_pts_r(inlier_idx, :);
        
        [O_l_g, P_l_g, cur_3D_pts, fval] = refineRt_n_Pts(prev_O_l_g, prev_P_l_g,...
                              O_l_g, P_l_g,...
                              prev_matched_pts_l, prev_matched_pts_r,...
                              matched_pts_l, matched_pts_r,...
                              cam_l_param, cam_r_param, ...
                              cur_3D_pts, R_r_gt_rel, t_r_gt_rel);
    
        % output R t in matlab convention -- transposed relative to cv convention
        % [F, prev_matched_pts_l_filt, matched_pts_l_filt, R_l_local, t_l_local]...
        %       = calc_cam_poses(prev_matched_pts_l_filt, matched_pts_l_filt, ...
        %                                           cam_l_param, cam_l_param);

        % R_l_local = R_l_local'; % to cv convention
        % t_l_local = t_l_local';

        % gt is in cv convention, 
        % [O_l_g, P_l_g] = local2globalOP(prev_O_l_g, prev_P_l_g, R_l_local, t_l_local);

        % visualization and comparison
        [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
        [prev_R_l_g, prev_t_l_g] = switch_coord_sys(prev_O_l_g, prev_P_l_g);
        [prev_R_r_g, prev_t_r_g] = switch_coord_sys(prev_O_r_g, prev_P_r_g);
        [R_r_2prev_rel, t_r_2prev_rel] = get_rel_Rt(prev_R_l_g, prev_t_l_g, R_l_g, t_l_g);
        F = FfromRTK(R_r_2prev_rel, t_r_2prev_rel, cam_l_param.IntrinsicMatrix');
        draw_epipolar_lines(F, prev_matched_pts_l, matched_pts_l, prev_im_l, im_l);

        % plot stereo epipolar lines from groundtruth
        % gt_stereo_Rt(eye(3), zeros(3,1), R_r_gt_rel, t_r_gt_rel, ...
        % cam_l_param, cam_r_param, im_l, im_r, matched_pts_l, matched_pts_r);
        
        % compare before and after optimization
        [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
        [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);
        proj_l = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                             cam_l_param.IntrinsicMatrix', R_l_g, t_l_g);
        proj_r = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                             cam_r_param.IntrinsicMatrix', R_r_g, t_r_g);
        proj_prev_l = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                             cam_l_param.IntrinsicMatrix', prev_R_l_g, prev_t_l_g);
        proj_prev_r = project_to_cam([cur_3D_pts'; ones(1,size(cur_3D_pts,1))], ...
                                             cam_r_param.IntrinsicMatrix', prev_R_r_g, prev_t_r_g);                                         
        compare_opti(prev_im_l, prev_im_r, im_l, im_r,...
                    prev_matched_pts_l, prev_matched_pts_r, proj_prev_l', proj_prev_r',...
                                    matched_pts_l, matched_pts_r, proj_l', proj_r');
    end
% ---- track camera pose ---- %


% ---- triangulate ---- %
    [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
    
    size(matched_pts_l, 1)
    if size(matched_pts_l, 1) < 80
        need_traingulate = true;
    end
    
    if need_traingulate
        % world to cur cam coord
        [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
        [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);

        camMatrix_l = cameraMatrix(cam_l_param,R_l_g,t_l_g);
        camMatrix_r = cameraMatrix(cam_r_param,R_r_g,t_r_g);

        cur_3D_pts = triangulate(matched_pts_l,matched_pts_r,camMatrix_l,camMatrix_r);
    %     point_dicts = store_pts(worldPoints, point_dicts);
    
        need_traingulate = false;
    end
    
    toc()    
% ---- triangulate ---- %

% ---- visualize ---- %
    draw_map(cur_3D_pts, O_l_g, P_l_g, O_r_g, P_r_g);
% ---- visualize ---- %

% ---- bundle adjustment ---- %    
    % [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints,pointTracks,cameraPoses,cam_l_param);
% ---- bundle adjustment ---- %    

% ---- update variables ---- %    
    prev_O_l_g = O_l_g;
    prev_P_l_g = P_l_g;
    prev_O_r_g = O_r_g;
    prev_P_r_g = P_r_g;    
    prev_im_l = im_l;
    prev_im_r = im_r;
    prev_desc_l = desc_l; 
    prev_matched_pts_l = matched_pts_l;
    prev_matched_pts_r = matched_pts_r;
% ---- update variables ---- %    

end
% pointTracker = vision.PointTracker('MaxBidirectionalError', 3, 'BlockSize', 15, 'MaxIterations', 15);




