function [im_l, im_r] = get_images(im_l_fn, im_r_fn, cam_l_param, cam_r_param)
    im_l_dis = imread(im_l_fn);
    im_r_dis = imread(im_r_fn);
    [im_l,~] = undistortImage(im_l_dis,cam_l_param);
    [im_r,~] = undistortImage(im_r_dis,cam_r_param);
    
%     figure(3), title('undistortion'), hold on;
%     subplot(2,2,1), imshow(im_l_dis);
%     subplot(2,2,2), imshow(im_r_dis);
%     subplot(2,2,3), imshow(im_l);
%     subplot(2,2,4), imshow(im_r);
%     close(3)
end