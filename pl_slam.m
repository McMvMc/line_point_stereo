YAML_LIB_PATH = '~/YAMLMatlab_0.4.3';
addpath(YAML_LIB_PATH);

img_path_l = '~/Desktop/833/project/mav0/cam0/data/';
img_path_r = '~/Desktop/833/project/mav0/cam1/data/';
step_size = 3;
start_idx = 700;
down_scale = 500/700;

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
[O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
prev_O_r_g = O_r_g;
prev_P_r_g = P_r_g;
                      
for idx=start_idx:step_size:size(img_list_l)
    tic()
    
% ---- track features ---- %
    [im_l, im_r] = get_images(strcat(img_path_l, img_list_l(idx).name), ...
                              strcat(img_path_r, img_list_r(idx).name), ...
                              cam_l_param, cam_r_param);
                          
    im_l = imresize(im_l,down_scale);
    im_r = imresize(im_r,down_scale);
    
    % redetect: detect and match with previous points.                      
    if idx==start_idx
        [desc_l, matched_pts_l] = detect_points(im_l);
        matched_pts_l = matched_pts_l.Location;
        [matched_pts_l, matched_pts_r, validity] = track_points(matched_pts_l, ...
                                                        im_l, im_r);
        cur_pt_ids = 1:size(matched_pts_l, 1);
        global_descriptor = binaryFeatures(desc_l.Features(validity,:));
        
        % prev_matched_pts_l = matched_pts_l;
        % prev_matched_pts_r = matched_pts_r;
        
        % world to cur cam coord
        [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
        [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);

        camMatrix_l = cameraMatrix(cam_l_param,R_l_g,t_l_g);
        camMatrix_r = cameraMatrix(cam_r_param,R_r_g,t_r_g);

        [cur_3D_pts, err] = triangulate(matched_pts_l, matched_pts_r,...
                                                camMatrix_l,camMatrix_r);
    else
        % track between frames and between stereo
        % filters points
        [prev_matched_pts_l_filt, matched_pts_l_filt, valid_l] = ...
                                    track_points(prev_matched_pts_l, prev_im_l, im_l);
        [matched_pts_l_filt, matched_pts_r, valid_r] = track_points(matched_pts_l_filt, ...
                                                        im_l, im_r);
        prev_matched_pts_l_filt = prev_matched_pts_l_filt(valid_r, :);    

        size(matched_pts_l, 1)
        if size(matched_pts_l, 1) < 150
            need_redetect = true;
            need_traingulate = true;
        else
            need_redetect = false;
            need_traingulate = false;
        end
        
        avg_disparity = mean(sqrt(sum((prev_matched_pts_l_filt - ...
                                                matched_pts_l_filt).^2,2)));
        if avg_disparity>5
            prev_matched_pts_l = prev_matched_pts_l_filt;
            matched_pts_l = matched_pts_l_filt;
            cur_3D_pts = cur_3D_pts(valid_l,:);
            cur_3D_pts = cur_3D_pts(valid_r,:);
            cur_pt_ids = cur_pt_ids(valid_l);
            cur_pt_ids = cur_pt_ids(valid_r);
        else
            continue
        end
        
        % ---- redetection ---- %
        if need_redetect
            [prev_matched_pts_l, prev_matched_pts_r, new_prev_matched_pts_l, ...
            new_prev_matched_pts_r, new_matched_pts_l, new_matched_pts_r, ...
            cur_3D_pts, cur_pt_ids, global_descriptor, matched_pts_l, matched_pts_r]...
             = redetect_points(prev_im_l, prev_im_r, im_l, im_r, prev_matched_pts_l, ...
                matched_pts_l, matched_pts_r, cur_3D_pts, cur_pt_ids, global_descriptor);
        end
        
    end
% ---- track features ---- %


% ---- track camera pose ---- %
    if idx ~= start_idx
        [O_l_g, P_l_g, inlier_idx] = estimateWorldCameraPose(matched_pts_l,...
                                                            cur_3D_pts,cam_l_param);
        P_l_g = P_l_g';
        cur_3D_pts = cur_3D_pts(inlier_idx, :);
        matched_pts_l = matched_pts_l(inlier_idx, :);
        matched_pts_r = matched_pts_r(inlier_idx, :);
        prev_matched_pts_l = prev_matched_pts_l(inlier_idx, :);
        prev_matched_pts_r = prev_matched_pts_r(inlier_idx, :);
        
        [O_l_g, P_l_g, cur_3D_pts, fval] = refineRt_n_Pts(prev_O_l_g, prev_P_l_g,...
                              O_l_g, P_l_g,...
                              prev_matched_pts_l, prev_matched_pts_r,...
                              matched_pts_l, matched_pts_r,...
                              cam_l_param, cam_r_param, ...
                              cur_3D_pts, R_r_gt_rel, t_r_gt_rel);
        [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
                          
        % output R t in matlab convention -- transposed relative to cv convention
        % [F, prev_matched_pts_l_filt, matched_pts_l_filt, R_l_local, t_l_local]...
        %       = calc_cam_poses(prev_matched_pts_l_filt, matched_pts_l_filt, ...
        %                                           cam_l_param, cam_l_param);

        % R_l_local = R_l_local'; % to cv convention
        % t_l_local = t_l_local';

        % gt is in cv convention, 
        % [O_l_g, P_l_g] = local2globalOP(prev_O_l_g, prev_P_l_g, R_l_local, t_l_local);

        % visualization and comparison
        % compare before and after optimization
        if need_redetect
            tmp_matched_pts_l = [matched_pts_l; new_matched_pts_l];
            tmp_matched_pts_r = [matched_pts_r; new_matched_pts_r];
            tmp_prev_matched_pts_l = [prev_matched_pts_l; new_prev_matched_pts_l];
            tmp_prev_matched_pts_r = [prev_matched_pts_r; new_prev_matched_pts_r];
        else
            tmp_matched_pts_l = matched_pts_l;
            tmp_matched_pts_r = matched_pts_r;
            tmp_prev_matched_pts_l = prev_matched_pts_l;
            tmp_prev_matched_pts_r = prev_matched_pts_r;
        end
        [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
        [prev_R_l_g, prev_t_l_g] = switch_coord_sys(prev_O_l_g, prev_P_l_g);
        [prev_R_r_g, prev_t_r_g] = switch_coord_sys(prev_O_r_g, prev_P_r_g);
        [R_l_2prev_rel, t_l_2prev_rel] = get_rel_Rt(prev_R_l_g, prev_t_l_g, R_l_g, t_l_g);
        F = FfromRTK(R_l_2prev_rel, t_l_2prev_rel, cam_l_param.IntrinsicMatrix');
        draw_epipolar_lines(F, tmp_prev_matched_pts_l, tmp_matched_pts_l, prev_im_l, im_l);
        
        compare_opti(cur_3D_pts, cam_l_param, cam_r_param, prev_im_l, prev_im_r, ...
                                    im_l, im_r, prev_O_l_g, prev_P_l_g, ...
                                    O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel,...
                                    tmp_prev_matched_pts_l, tmp_prev_matched_pts_r,...
                                    tmp_matched_pts_l, tmp_matched_pts_r)
        
    end
% ---- track camera pose ---- %


% ---- triangulate ---- %
    % [prev_O_r_g, prev_P_r_g] = right_from_left_cam(prev_O_l_g, prev_P_l_g, ...
    %                                                     R_r_gt_rel, t_r_gt_rel);
    
    if need_traingulate
        % world to cur cam coord
        [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
        [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);

        camMatrix_l = cameraMatrix(cam_l_param,R_l_g',t_l_g');
        camMatrix_r = cameraMatrix(cam_r_param,R_r_g',t_r_g');

        [new_cur_3D_pts, err] = triangulate(new_matched_pts_l, new_matched_pts_r,...
                                                camMatrix_l,camMatrix_r);
        cur_3D_pts = [cur_3D_pts; new_cur_3D_pts];
        matched_pts_l = [matched_pts_l; new_matched_pts_l];
        matched_pts_r = [matched_pts_r; new_matched_pts_r];
        prev_matched_pts_l = [prev_matched_pts_l; new_prev_matched_pts_l];
        prev_matched_pts_r = [prev_matched_pts_r; new_prev_matched_pts_r];
        
        need_traingulate = false;
        
        compare_opti(cur_3D_pts, cam_l_param, cam_r_param, prev_im_l, prev_im_r, ...
                            im_l, im_r, prev_O_l_g, prev_P_l_g, ...
                            O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel,...
                            tmp_prev_matched_pts_l, tmp_prev_matched_pts_r,...
                            tmp_matched_pts_l, tmp_matched_pts_r)
    end
    
% ---- triangulate ---- %



    toc()

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




