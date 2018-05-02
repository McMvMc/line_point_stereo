addpath('../YAMLMatlab_0.4.3');
addpath('../..');
addpath('../Camera-Pose-Estimation-from-Lines-using-Plucker-Coordinates_supp/Matlab_code')
img_path_l = '../../../mav0/cam0/data/';
img_path_r = '../../../mav0/cam1/data/';

% reads out body frame O P, and convert to R t (left to right)
[R_cam, t_cam, cam_l_param, cam_r_param] = ...
                            read_gt(img_path_l, img_path_r);
             
F = (cam_r_param.IntrinsicMatrix)\(toSkew(t_cam)*R_cam)/(cam_l_param.IntrinsicMatrix');

stereoParams = stereoParameters(cam_l_param,cam_r_param,R_cam',t_cam');

img_list_l = dir(strcat(img_path_l, '*.png'));
img_list_r = dir(strcat(img_path_r, '*.png'));

frame_num = size(img_list_l, 1);

detection_l = cell(frame_num, 1);
detection_r = cell(frame_num, 1);
matching_l = cell(frame_num, 1);
matching_r = cell(frame_num, 1);
matching_s = cell(frame_num, 1);
keyframe = [];
world_l = {};
world_r = {};
cam_Matrix = {};

im_l = zeros(size(480, 752));
im_r = zeros(size(480, 752));

% for tracking
template_l = zeros(0, 4);
template_r = zeros(0, 4);
template_id_l = zeros(0, 1);
template_id_r = zeros(0, 1);
match_missed_l = zeros(0, 1);
match_missed_r = zeros(0, 1);
feature_tracked = 1000;

for idx=1:frame_num

    % read stereo image pair
    prev_im_l = im_l;
    prev_im_r = im_r;

    [im_l, im_r] = get_images(strcat(img_path_l, img_list_l(idx).name), ...
                              strcat(img_path_r, img_list_r(idx).name), ...
                              cam_l_param, cam_r_param);
    tic();

    % detection corner and line
    detection_l{idx} = detect_lines_full(im_l);
    detection_r{idx} = detect_lines_full(im_r);

    % stereo matching for initial 3D line generation
    matching_s{idx} = match_lines_stereo(detection_l{idx}, ...
                                        detection_r{idx}, ...
                                        im_l, im_r, F);
    
                                    
    if idx > 1
        % temporal tracking 
        [template_l, validity_l] = track_lines(template_l, prev_im_l, im_l);
        [template_r, validity_r] = track_lines(template_r, prev_im_r, im_r);

        % remove failed LK cases
        template_l = template_l(validity_l, :);
        template_r = template_r(validity_r, :);
        template_id_l = template_id_l(validity_l);
        template_id_r = template_id_r(validity_r);
        match_missed_l = match_missed_l(validity_l);
        match_missed_r = match_missed_r(validity_r);
        
        % update template with new detection
        match_l = match_lines(template_l, detection_l{idx});
        match_r = match_lines(template_r, detection_r{idx});
        
        template_l(match_l(:,1), :) = detection_l{idx}(match_l(:,2), :);
        template_r(match_r(:,1), :) = detection_r{idx}(match_r(:,2), :);
        matching_l{idx} = [template_id_l(match_l(:,1)), match_l(:,2)];
        matching_r{idx} = [template_id_r(match_r(:,1)), match_r(:,2)];
        
        missed_l = true(size(match_missed_l));
        missed_r = true(size(match_missed_r));
        missed_l(match_l(:,1)) = false;
        missed_r(match_r(:,1)) = false;
        
        match_missed_l(missed_l) = match_missed_l(missed_l) + 1;
        match_missed_r(missed_r) = match_missed_r(missed_r) + 1;
        
        missed_l = match_missed_l > 5;
        missed_r = match_missed_r > 5;
        
        template_l = template_l(~missed_l, :);
        template_r = template_r(~missed_r, :);
        template_id_l = template_id_l(~missed_l);
        template_id_r = template_id_r(~missed_r);
        match_missed_l = match_missed_l(~missed_l);
        match_missed_r = match_missed_r(~missed_r);
        
        % add in more stereo pairs
        match = matching_s{idx};
        current_pair = zeros(0, 2);
        for p = 1:size(match, 1)
            idx_l = find(matching_l{idx}(:,2)==match(p, 1));
            idx_r = find(matching_r{idx}(:,2)==match(p, 2));
            if numel(idx_l)==1 && numel(idx_r)==1
               % propagate new stereo pair
                current_pair(end+1, :) = [matching_l{idx}(idx_l,1),...
                                          matching_r{idx}(idx_r,1)];
            end
        end
        stereo_pair = unique([stereo_pair; current_pair], 'row');
    end
    
    if size(template_l, 1) + size(template_l, 2) <= min(200, feature_tracked*0.8)
        if idx > 1
            % generate 3D points with stereo pair
            
            matched_l = detection_l{keyframe(end)}(stereo_pair(:,1), :);
            matched_r = detection_r{keyframe(end)}(stereo_pair(:,2), :);
            pts_w_l = generate_3d_lines(matched_l, matched_r, stereoParams, F);
            pts_w_r = pts_w_l*R_cam' + t_cam';
            
            n = size(pts_w_l, 1)/2;
            lines_w_l = [pts_w_l(1:n, :), pts_w_l(n+1:end, :)];
            lines_w_r = [pts_w_r(1:n, :), pts_w_r(n+1:end, :)];
            
            % test pose estimation
            track_full_l = zeros(size(detection_l{keyframe(end)}, 1), 1);
            track_full_r = zeros(size(detection_r{keyframe(end)}, 1), 1);

            track_l = matching_l{idx};
            track_r = matching_r{idx};
            
            track_full_l(track_l(:,1)) = track_l(:,2);
            track_full_r(track_r(:,1)) = track_r(:,2);
            
            stereo_track_l = track_full_l(stereo_pair(:,1));
            stereo_track_r = track_full_r(stereo_pair(:,2));
            
            lines_w_l = lines_w_l(stereo_track_l~=0, :);
            lines_w_r = lines_w_r(stereo_track_r~=0, :);
            
            lines_l = detection_l{idx}(stereo_track_l(stereo_track_l~=0), :);
            lines_r = detection_r{idx}(stereo_track_r(stereo_track_r~=0), :);
            
            w_l = sqrt(sum((lines_l(:, 1:2) - lines_l(:,3:4)).^2, 2));
            w_r = sqrt(sum((lines_r(:, 1:2) - lines_r(:,3:4)).^2, 2));
            w = [w_l; w_r];
            w = sqrt(w);
            
            lines_w_l = reshape(lines_w_l', 3, []);
            lines_w_r = reshape(lines_w_r', 3, []);
            
            lines_w_l = [lines_w_l; ones(1, size(lines_w_l, 2))];
            lines_w_r = [lines_w_r; ones(1, size(lines_w_r, 2))];
  
            lines_l = reshape(lines_l', 2, []);
            lines_r = reshape(lines_r', 2, []);
            
            lines_l = cam_l_param.IntrinsicMatrix'\[lines_l; ones(1, size(lines_l, 2))];
            lines_r = cam_r_param.IntrinsicMatrix'\[lines_r; ones(1, size(lines_r, 2))];
            
            
            [ R, t] = esitmate_pose([lines_w_l, lines_w_r],...
                                    [lines_l, lines_r]);
            
            
            pts_l = (pts_w_l*R'+t')*cam_l_param.IntrinsicMatrix;
            pts_r = (pts_w_r*R'+t')*cam_r_param.IntrinsicMatrix;
            
            pts_l = pts_l(:,1:2)./pts_l(:,3);
            pts_r = pts_r(:,1:2)./pts_r(:,3);
            
            projected_l = [pts_l(1:n, :), pts_l(n+1:end, :)];
            projected_r = [pts_r(1:n, :), pts_r(n+1:end, :)];
%             
%             matched_l = matched_l(stereo_track_l~=0, :);
%             matched_r = matched_r(stereo_track_r~=0, :);


            vis_l = visualize_lines(zeros(size(im_l)), detection_l{idx});
            vis_r = visualize_lines(zeros(size(im_r)), detection_r{idx});
%             vis_l = visualize_lines(zeros(size(im_l)), matched_l);
%             vis_r = visualize_lines(zeros(size(im_r)), matched_r);
            vis_l = circshift(vis_l,1,3);
            vis_r = circshift(vis_r,1,3);
            vis_l = visualize_lines(vis_l, projected_l);
            vis_r = visualize_lines(vis_r, projected_r);
            
            vis_t = [vis_l, vis_r];
            

            figure(1);
            imshow(vis_t);

            % do 3D-2D registration to get the relative pose of the current 
            % frame againt last keyframe
            P = [R, t; zeros(1, 3), 1];
            P = cam_Matrix{end}/P;
            
            % add new 3d lines
            R = cam_Matrix{end}(1:3,1:3);
            t = cam_Matrix{end}(1:3,4);
            pts_w_l = pts_w_l*R' + t';
            n = size(pts_w_l, 1)/2;
%             world_l{end} = [pts_w_l(1:n, :), pts_w_l(n+1:end, :)];
            world_points = pts_w_l;
            figure(2);
            len = vecnorm(pts_w_l(1:n, :)- pts_w_l(n+1:end, :), 2, 2);
            valid = len < 2;
            world_points = world_points([valid; valid], :);
            hold on
            plot3(reshape(world_points(:,1), [], 2)', ...
                reshape(world_points(:,2), [], 2)', ...
                reshape(world_points(:,3), [], 2)');
            axis equal
            hold off
        
        else
            R = eye(3);
            t = ones(3, 1);
            P = [R, t; zeros(1, 3), 1];
        end
        % add new keyframe 
        keyframe(end+1) = idx;
        cam_Matrix{end+1} = P;  
        
        template_l = detection_l{idx};
        template_r = detection_r{idx};
        template_id_l = (1:size(template_l, 1))';
        template_id_r = (1:size(template_r, 1))';
        match_missed_l = zeros(size(template_id_l));
        match_missed_r = zeros(size(template_id_r));
        
        stereo_pair = matching_s{idx};
        

    end
    
    match = matching_s{idx};
    matched_l = detection_l{idx}(match(:,1), :);
    matched_r = detection_r{idx}(match(:,2), :);
    

    
    
    
    % if tracked line drops, initailize new keyframe
    

    
    
    run_time = toc();
    
    

    % visualization
%     disp(run_time);
    continue;
    
    vis_l = visualize_lines(zeros(size(im_l)), detection_l{idx});
    vis_r = visualize_lines(zeros(size(im_r)), detection_r{idx});
    vis_l = circshift(vis_l,1,3);
    vis_r = circshift(vis_r,1,3);
    vis_l = visualize_lines(vis_l, template_l);
    vis_r = visualize_lines(vis_r, template_r);

    vis_l = circshift(vis_l,1,3);
    vis_r = circshift(vis_r,1,3);
    vis_l = visualize_lines(vis_l, matched_l);
    vis_r = visualize_lines(vis_r, matched_r);
    vis_l = circshift(vis_l,-1,3);
    vis_r = circshift(vis_r,-1,3);
    
    vis_key_l = visualize_lines(zeros(size(im_l)), detection_l{keyframe(end)});
    vis_key_r = visualize_lines(zeros(size(im_r)), detection_r{keyframe(end)});
    vis_key_l = circshift(vis_key_l,1,3);
    vis_key_r = circshift(vis_key_r,1,3);
    vis_key_l = visualize_lines(vis_key_l, detection_l{keyframe(end)}(stereo_pair(:,1), :));
    vis_key_r = visualize_lines(vis_key_r, detection_r{keyframe(end)}(stereo_pair(:,2), :));
    

%     vis_p = circshift([vis_l, vis_r], 1, 3);
%     c_l = (matched_l(:,1:2) + matched_l(:,3:4))/2;
%     c_r = (matched_r(:,1:2) + matched_r(:,3:4))/2;
%     c_r(:,1) = c_r(:,1) + size(vis_l, 2);
%     matched_line = [c_l, c_r];
%     im_p = visualize_lines(im_p, matched_line);
%     vis_p = circshift(vis_p, -1, 3);
%     im_l = visualize_points(im_l, pts_l);
%     im_r = visualize_points(im_r, pts_r);
    
    figure(1);
    imshow([[vis_l, vis_r];[vis_key_l, vis_key_r]]);

    

end



