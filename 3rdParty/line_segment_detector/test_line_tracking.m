addpath('../YAMLMatlab_0.4.3');
addpath('../..');
img_path_l = '../../../mav0/cam0/data/';
img_path_r = '../../../mav0/cam1/data/';

% reads out body frame O P, and convert to R t (left to right)
[R, t, cam_l_param, cam_r_param] = ...
                            read_gt(img_path_l, img_path_r);
                        
% F = (cam_r_param.IntrinsicMatrix)\(toSkew(t)*R)/(cam_l_param.IntrinsicMatrix');

stereoParams = stereoParameters(cam_l_param,cam_r_param,R',t');

img_list_l = dir(strcat(img_path_l, '*.png'));
img_list_r = dir(strcat(img_path_r, '*.png'));

radius = 5;
feature_tracked = 1000;
last_detection_frame = 1;
for idx=1:size(img_list_l)
    
    if idx == 1
        prev_im_l = zeros(size(502, 937));
        prev_lines_l = zeros(0, 4);
        match_missed = zeros(0, 1);
    else
        prev_im_l = im_l;
        if size(matched, 1) <= min(200, feature_tracked*0.8)
            prev_lines_l = lines_l;
            feature_tracked = size(lines_l, 1);
            match_missed = zeros(feature_tracked, 1);
            
            disp("track lost");
            disp(["track length", num2str(idx-last_detection_frame)]);
            last_detection_frame = idx;
            
        else
            prev_lines_l = matched;
        end
    end
    
    [im_l, im_r] = get_images(strcat(img_path_l, img_list_l(idx).name), ...
                              strcat(img_path_r, img_list_r(idx).name), ...
                              cam_l_param, cam_r_param);
%     [im_l, im_r] = rectifyStereoImages(im_l, im_r,stereoParams);
    
    scale = 0.5;
    tic();     
    im_l_down = imresize(im_l, scale);
    [~, valid_pts_l] = detect_points(im_l_down);
    pts_l = valid_pts_l.Location/scale; 
    lines_l = detect_lines(im_l, pts_l);
    
    if idx > 1
        % match frame with previous 
        [tracked, validity] = track_lines(prev_lines_l, prev_im_l, im_l);
%         disp(sum(validity)/numel(validity));
        tracked = tracked(validity, :);
        match_missed = match_missed(validity, :);
        matched = tracked;
        match = match_lines(matched, lines_l);
        % update matched lines
        matched(match~=0, :) = lines_l(match(match~=0), :);
        match_missed(match==0) = match_missed(match==0) + 1;
        missed = match_missed > 3;
        matched = matched(~missed, :);
        match_missed = match_missed(~missed, :);
        
        
    else
        matched = zeros(0, 4);
        match_missed = zeros(0, 1);
    end
   
    
    run_time = toc();
%     disp(run_time);
    

    prev_vis_l = visualize_lines(zeros(size(im_l)), prev_lines_l);
    vis_l = visualize_lines(zeros(size(im_l)), lines_l);
    vis_l = circshift(vis_l, 1, 3);
    vis_l = visualize_lines(vis_l, matched);
    vis_l = circshift(vis_l, -1, 3);
    
    figure(1);
    imshow([prev_vis_l, vis_l]);
    

end



