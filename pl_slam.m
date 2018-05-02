YAML_LIB_PATH = '3rdParty/YAMLMatlab_0.4.3';
addpath(YAML_LIB_PATH);

img_path_l = '../mav0/cam0/data/';
img_path_r = '../mav0/cam1/data/';
step_size = 3;
start_idx = 2000;
redetect_thresh = 600;
down_scale = 1;

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
point_database= containers.Map('KeyType','double','ValueType','any');
frame_cell=containers.Map('KeyType','double','ValueType','any');
key_frame_index=[];
updated_pts_id=[];
key_frame_count=1;
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
                                im_l, im_r, false, [], []);
        
        % triangulate
        [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
        [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);

        camMatrix_l = cameraMatrix(cam_l_param,R_l_g,t_l_g);
        camMatrix_r = cameraMatrix(cam_r_param,R_r_g,t_r_g);

        [cur_3D_pts, err] = triangulate(matched_pts_l, matched_pts_r,...
                                                camMatrix_l,camMatrix_r);
                                            
        % store database
        cur_pt_ids = 1:size(matched_pts_l, 1);
%         global_descriptor = binaryFeatures(desc_l.Features(validity,:));
        global_descriptor = desc_l(validity,:);
        global_3D_pts = cur_3D_pts;
    else
        % track between frames and between stereo
        % filters points
        
        use_matching = true;
        cur_pt_ids_filt = cur_pt_ids;
        [prev_matched_pts_l_filt, matched_pts_l, valid_l] = ...
                                    track_points(prev_matched_pts_l, prev_im_l, im_l,...
                                     use_matching, cur_pt_ids_filt, global_descriptor);
        prev_matched_pts_r_filt = prev_matched_pts_r(valid_l, :);
        
        cur_pt_ids_filt = cur_pt_ids_filt(valid_l);
        [matched_pts_l, matched_pts_r, valid_r] = track_points(matched_pts_l, ...
                         im_l, im_r,  use_matching, cur_pt_ids_filt, global_descriptor);
        prev_matched_pts_l_filt = prev_matched_pts_l_filt(valid_r, :);
        prev_matched_pts_r_filt = prev_matched_pts_r_filt(valid_r, :);
        
        cur_pt_ids_filt = cur_pt_ids_filt(valid_r);

        size(matched_pts_l, 1)
        if size(matched_pts_l, 1) < redetect_thresh
            need_redetect = true;
            need_traingulate = true;          
        else
            need_redetect = false;
            need_traingulate = false;
        end
        
        % H_reproj_err = test_homography(prev_matched_pts_l_filt, matched_pts_l_filt);
        % 'homo error'
        % H_reproj_err = median(H_reproj_err)
        
        avg_disparity = mean(sqrt(sum((prev_matched_pts_l_filt - ...
                                                matched_pts_l).^2,2)));
        if avg_disparity>5
            'camera moving'
            prev_matched_pts_l = prev_matched_pts_l_filt;
            prev_matched_pts_r = prev_matched_pts_r_filt;
            cur_3D_pts = cur_3D_pts(valid_l, :);
            cur_3D_pts = cur_3D_pts(valid_r, :);
            cur_pt_ids = cur_pt_ids_filt;  
        else
            continue
        end
        
        % ---- redetection ---- %
        if need_redetect

            [prev_matched_pts_l, prev_matched_pts_r, new_prev_matched_pts_l, ...
            new_prev_matched_pts_r, new_matched_pts_l, new_matched_pts_r, ...
            new_cur_pt_ids, cur_3D_pts, cur_pt_ids, global_descriptor, ...
            matched_pts_l, matched_pts_r, n_old_match]...
                 = redetect_points(prev_im_l, prev_im_r, im_l, im_r, ...
                    prev_matched_pts_l, matched_pts_l, matched_pts_r, ...
                    cur_3D_pts, cur_pt_ids, global_descriptor, global_3D_pts);
        end
        
    end
% ---- track features ---- %

    if idx ~=start_idx
        id_s = cur_pt_ids;
        pt_3d_s = cur_3D_pts;
        m_l_s = matched_pts_l;
        m_r_s = matched_pts_r;
        p_m_l_s = prev_matched_pts_l;
        p_m_r_s = prev_matched_pts_r;
    end
% ---- track camera pose ---- %
    if idx ~= start_idx
        
        [O_l_g, P_l_g, inlier_idx, status] = estimateWorldCameraPose(matched_pts_l,...
                                                    cur_3D_pts,cam_l_param);
        P_l_g = P_l_g';
        
        if status ~=0
            
        end
        
        cur_3D_pts = cur_3D_pts(inlier_idx, :);
        cur_pt_ids = cur_pt_ids(inlier_idx);
        matched_pts_l = matched_pts_l(inlier_idx, :);
        matched_pts_r = matched_pts_r(inlier_idx, :);
        prev_matched_pts_l = prev_matched_pts_l(inlier_idx, :);
        prev_matched_pts_r = prev_matched_pts_r(inlier_idx, :);
        
        O_l_g = prev_O_l_g;
        P_l_g = prev_P_l_g;
        [O_l_g, P_l_g, cur_3D_pts, fval] = refineRt_n_Pts(prev_O_l_g, prev_P_l_g,...
                              O_l_g, P_l_g,...
                              prev_matched_pts_l, prev_matched_pts_r,...
                              matched_pts_l, matched_pts_r,...
                              cam_l_param, cam_r_param, ...
                              cur_3D_pts, R_r_gt_rel, t_r_gt_rel);
        [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
        
        % update global database
        global_3D_pts(cur_pt_ids,:) = cur_3D_pts;
        
        
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
    
    if need_traingulate
        % world to cur cam coord
        [R_l_g, t_l_g] = switch_coord_sys(O_l_g, P_l_g);
        [R_r_g, t_r_g] = switch_coord_sys(O_r_g, P_r_g);

        camMatrix_l = cameraMatrix(cam_l_param,R_l_g',t_l_g');
        camMatrix_r = cameraMatrix(cam_r_param,R_r_g',t_r_g');

        [new_cur_3D_pts, err] = triangulate(new_matched_pts_l, new_matched_pts_r,...
                                                camMatrix_l,camMatrix_r);
        cur_3D_pts = [cur_3D_pts; new_cur_3D_pts];
        global_3D_pts = [global_3D_pts; new_cur_3D_pts];
        cur_pt_ids = [cur_pt_ids new_cur_pt_ids];
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
%     draw_map(global_3D_pts, O_l_g, P_l_g, O_r_g, P_r_g);
    draw_map(cur_3D_pts, O_l_g, P_l_g, O_r_g, P_r_g);
% ---- visualize ---- %


% ---- bundle adjustment ---- %    
    % [xyzRefinedPoints,refinedPoses] = bundleAdjustment(xyzPoints,pointTracks,cameraPoses,cam_l_param);
% ---- bundle adjustment ---- %    

    if idx ~=start_idx
        string(idx)+'--------------'
        id_e = cur_pt_ids(1:5)
        id_s_idx = [];
        for id = id_e
            id_s_idx = [id_s_idx find(id_s==id)];
        end
        pt_3d_s(id_s_idx,:)
        pt_3d_e = cur_3D_pts(1:5, :)
        m_l_s(id_s_idx,:)
        m_l_e = matched_pts_l(1:5, :)
        m_r_s(id_s_idx,:)
        m_r_e = matched_pts_r(1:5, :)
        p_m_l_s(id_s_idx,:)
        p_m_l_e = prev_matched_pts_l(1:5, :)
        p_m_r_s(id_s_idx,:)
        p_m_r_e = prev_matched_pts_r(1:5, :)
        string(idx)+'--------------'
    end



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




