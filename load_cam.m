% T_BS is from sensor to 
% https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets#ground-truth
function [R_gt, t_gt, cam_param] = load_cam(path)
    cam_yaml = ReadYaml(path);

    % matlab reshape is transposed
    Rt_gt = cell2mat(reshape(cam_yaml.T_BS.data, 4, 4))';
    R_gt = Rt_gt(1:3,1:3);
    t_gt = Rt_gt(1:3,4);
    
    Kt = cell2mat([cam_yaml.intrinsics(1), 0, 0;
          0, cam_yaml.intrinsics(2), 0;
          cam_yaml.intrinsics(3), cam_yaml.intrinsics(4), 1]);
    cam_param = cameraParameters('IntrinsicMatrix', Kt,...
                                 'RadialDistortion', cell2mat(cam_yaml.distortion_coefficients(1:2)),...
                                 'TangentialDistortion', cell2mat(cam_yaml.distortion_coefficients(3:4)));
    
end