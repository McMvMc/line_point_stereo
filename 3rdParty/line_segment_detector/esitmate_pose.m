function [bestR, bestt] = esitmate_pose(l3d, l2d)
%ESITMATE_POSE Summary of this function goes here
%   Detailed explanation goes here

NLINES = size(l3d, 2)/2;
sample_size = min(30, NLINES);

bestErr = Inf;
bestR = eye(3);
bestt = ones(3, 1);

pl2d = cross(l2d(:, 1:2:end), l2d(:, 2:2:end));
pl2d = pl2d./vecnorm(pl2d, 2, 1);
pl2d = reshape([pl2d;pl2d], 3, []);


for i = 1:200
    idx = randperm(NLINES, sample_size);
    idx = reshape([idx*2-1; idx*2], 1, []);

    [ R, t] = linePoseEstimAOR( l3d(:,idx), l2d(:,idx), 1, 460, ...
        ones(sample_size, 1), -1);
    pl3d = (R*l3d(1:3,idx)+t);
    pl3d = pl3d./pl3d(3,:);
    err = mean(abs(sum(pl3d.*pl2d(:, idx), 1)));
    if err < bestErr
        bestErr = err;
        bestR = R;
        bestt = t;
    end
end

% remove outlier
R = bestR;
t = bestt;
pl3d = (R*l3d(1:3,:)+t);
pl3d = pl3d./pl3d(3,:);
err = abs(sum(pl3d.*pl2d, 1));
err = sum(reshape(err, 2, []), 1);
err_mean = mean(err);
err_sig = sqrt(var(err));
inliers = err < err_mean + err_sig;
inliers = reshape([inliers; inliers], 1, []);
l3d = l3d(:, inliers);
pl2d = pl2d(:, inliers);


fun = @(x)pose_err(x,l3d, pl2d);
options.Algorithm = 'levenberg-marquardt';
options.FunctionTolerance = '1e-8';

x0 = [rotationMatrixToVector(bestR), bestt'];
[x, r] = lsqnonlin(fun,x0,[],[],options);

bestR = rotationVectorToMatrix(x(1:3));
bestt = x(4:6)';
r


end

function d = pose_err(x, l3d, pl2d)
    
    R = rotationVectorToMatrix(x(1:3));
    t = x(4:6)';
    pl3d = (R*l3d(1:3,:)+t);
    pl3d = pl3d./pl3d(3,:);
    d = sum(pl3d.*pl2d, 1)';

end

