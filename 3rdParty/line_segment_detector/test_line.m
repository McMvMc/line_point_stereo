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

radius = 5;

for idx=1:size(img_list_l)
    [im_l, im_r] = get_images(strcat(img_path_l, img_list_l(idx).name), ...
                              strcat(img_path_r, img_list_r(idx).name), ...
                              cam_l_param, cam_r_param);
    tic();
%     [im_l, im_r] = rectifyStereoImages(im_l, im_r, stereoParams);
    
    scale = 0.5;
         
    im_l_down = imresize(im_l, scale);
    im_r_down = imresize(im_r, scale);
    
    
    [~, valid_pts_l] = detect_points(im_l_down);
    [~, valid_pts_r] = detect_points(im_r_down);
    
    pts_l = valid_pts_l.Location/scale;
    pts_r = valid_pts_r.Location/scale;
    
    lines_l = detect_lines(im_l, pts_l);
%     match = match_endpoints(pts_l, lines_l, radius);
%     lines_l = lines_l(sum(match~=0,2)~=0, :);
    
    lines_r = detect_lines(im_r, pts_r);
%     match = match_endpoints(pts_r, lines_r, radius);
%     lines_r = lines_r(sum(match~=0,2)~=0, :);

    match = match_lines_stereo2(lines_l, lines_r, im_l, im_r, F);
    matched_l = lines_l(match(:,1), :);
    matched_r = lines_r(match(:,2), :);
    
    
    world_points = triangulate([matched_l(:,1:2);matched_l(:,3:4)],...
                               [matched_r(:,1:2);matched_r(:,3:4)],...
                               stereoParams);
    
    
    run_time = toc();
    
%     figure(2);
%     plot3(reshape(world_points(:,1), [], 2)',...
%           reshape(world_points(:,2), [], 2)',...
%           reshape(world_points(:,3), [], 2)');
%     axis equal
    disp(run_time);
    
    im_l = visualize_lines(zeros(size(im_l)), lines_l);
    im_r = visualize_lines(zeros(size(im_r)), lines_r);
    
    im_l = circshift(im_l,1,3);
    im_r = circshift(im_r,1,3);
    im_l = visualize_lines(im_l, lines_l(match(:,1), :));
    im_r = visualize_lines(im_r, lines_r(match(:,2), :));
    

    im_p = circshift([im_l, im_r], 1, 3);
    c_l = (matched_l(:,1:2) + matched_l(:,3:4))/2;
    c_r = (matched_r(:,1:2) + matched_r(:,3:4))/2;
    c_r(:,1) = c_r(:,1) + size(im_l, 2);
    matched_line = [c_l, c_r];
%     im_p = visualize_lines(im_p, matched_line);
    im_p = circshift(im_p, -1, 3);
%     im_l = visualize_points(im_l, pts_l);
%     im_r = visualize_points(im_r, pts_r);
    
    figure(1);
    imshow(im_p);
    figure(2);
    hold on
    plot3(reshape(world_points(:,1), [], 2)', ...
          reshape(world_points(:,2), [], 2)', ...
          reshape(world_points(:,3), [], 2)');
    axis equal
    hold off
    

end



