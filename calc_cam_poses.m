function [F, matched_pts_l, matched_pts_r, R_r_g, t_r_g]...
            = calc_cam_poses(matched_pts_l, matched_pts_r, cam_l_param, cam_r_param)

    [F,inliers] = estimateFundamentalMatrix(matched_pts_l, matched_pts_r,...
                                    'NumTrials',1000);
%                                   'InlierPercentage', 80);
%                                 'Method','RANSAC',...
%                                 'NumTrials',1000,'DistanceThreshold',1e-4,...
                                

    matched_pts_l = matched_pts_l(inliers,:);
    matched_pts_r = matched_pts_r(inliers,:);

    % matlab here uses F in cv convention
    [F, err] = refineF(F,matched_pts_l,matched_pts_r);

    % these are transposed -- Matlab fashion.
    [Ori_r_g,Pos_r_g] = cameraPose(F,cam_l_param,cam_r_param,...
                                        matched_pts_l,matched_pts_r);
    
    % matlab convention -- [x y z] = [x y z]R + t                                
    R_r_g = Ori_r_g';
    t_r_g = -Pos_r_g*Ori_r_g';

end