% proj -- 2xN
function proj = project_to_cam(pts_3d, K, R, t)
    P = K*[R, t];    
    
    proj = P*pts_3d;
    proj = proj./proj(3,:);
    proj = proj(1:2,:);
end