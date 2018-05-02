addpath('../YAMLMatlab_0.4.3');
addpath('../..');
img_path_l = '../../../mav0/cam0/data/';
img_path_r = '../../../mav0/cam1/data/';

% reads out body frame O P, and convert to R t (left to right)
[R, t, cam_l_param, cam_r_param] = ...
                            read_gt(img_path_l, img_path_r);
             
F = (cam_r_param.IntrinsicMatrix)\(toSkew(t)*R)/(cam_l_param.IntrinsicMatrix');

stereoParams = stereoParameters(cam_l_param,cam_r_param,R',t');

img_list_l = dir(strcat(img_path_l, '*.png'));
img_list_r = dir(strcat(img_path_r, '*.png'));

frame_num = size(img_list_l);

detection_l = cell(frame_num, 1);
detection_r = cell(frame_num, 1);
tracking_l = cell(frame_num, 1);
tracking_r = cell(frame_num, 1);
temperal_match_l;


for idx=1:frame_num
    
    % read stereo image pair
    [im_l, im_r] = get_images(strcat(img_path_l, img_list_l(idx).name), ...
                              strcat(img_path_r, img_list_r(idx).name), ...
                              cam_l_param, cam_r_param);
    tic();

    % detection corner and line
    detection_l{idx} = detect_lines_full(im_l);
    detection_r{idx} = detect_lines_full(im_r);

    % stereo matching for initial 3D line generation
    match = match_lines_stereo2(lines_l, lines_r, im_l, im_r, F);
    matched_l = lines_l(match(:,1), :);
    matched_r = lines_r(match(:,2), :);
    
    pts_w_l = triangulate([matched_l(:,1:2);matched_l(:,3:4)],...
                               [matched_r(:,1:2);matched_r(:,3:4)],...
                               stereoParams);
    
    n = size(pts_w_l, 1)/2;
    lines_w_l = [pts_w_l(1:n, :), pts_w_l(n+1:end, :)];
    
    
    run_time = toc();
    

    % visualization
    disp(run_time);
    
    vis_l = visualize_lines(zeros(size(im_l)), lines_l);
    vis_r = visualize_lines(zeros(size(im_r)), lines_r);
    vis_l = circshift(vis_l,1,3);
    vis_r = circshift(vis_r,1,3);
    vis_l = visualize_lines(vis_l, matched_l);
    vis_r = visualize_lines(vis_r, matched_r);
    

    vis_p = circshift([vis_l, vis_r], 1, 3);
    c_l = (matched_l(:,1:2) + matched_l(:,3:4))/2;
    c_r = (matched_r(:,1:2) + matched_r(:,3:4))/2;
    c_r(:,1) = c_r(:,1) + size(vis_l, 2);
    matched_line = [c_l, c_r];
%     im_p = visualize_lines(im_p, matched_line);
    vis_p = circshift(vis_p, -1, 3);
%     im_l = visualize_points(im_l, pts_l);
%     im_r = visualize_points(im_r, pts_r);
    
    figure(1);
    imshow(vis_p);
%     figure(2);
%     hold on
%     plot3(reshape(world_points(:,1), [], 2)', ...
%           reshape(world_points(:,2), [], 2)', ...
%           reshape(world_points(:,3), [], 2)');
%     axis equal
%     hold off
    

end



