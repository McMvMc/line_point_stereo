% T_BS is from sensor to world, in Direct cosine matrix and P.
% https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets#ground-truth
function [O_gt_b, P_gt_b, cam_param] = load_cam(path)
    cam_yaml = ReadYaml(path);

    % matlab reshape is transposed
    DCM_P_gt = cell2mat(reshape(cam_yaml.T_BS.data, 4, 4))';
    DCM_b_gt = DCM_P_gt(1:3,1:3);
    b_P_bs_gt = DCM_P_gt(1:3,4);
    
    Kt = cell2mat([cam_yaml.intrinsics(1), 0, 0;
          0, cam_yaml.intrinsics(2), 0;
          cam_yaml.intrinsics(3), cam_yaml.intrinsics(4), 1]);
    cam_param = cameraParameters('IntrinsicMatrix', Kt,...
                                 'RadialDistortion', cell2mat(cam_yaml.distortion_coefficients(1:2)),...
                                 'TangentialDistortion', cell2mat(cam_yaml.distortion_coefficients(3:4)));
    
    % O and P in body frame,   ^ x    camera frame x <- |
    %                       y<-|                      y v
    O_gt_b = DCM_b_gt;
    P_gt_b = b_P_bs_gt;
end