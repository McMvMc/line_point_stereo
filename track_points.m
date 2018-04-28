function [matched_pts_l, matched_pts_r] = track_points(matched_pts_l, im_l, im_r)
    LKT = vision.PointTracker('MaxBidirectionalError',1,...
                              'BlockSize', [11, 11]);
    initialize(LKT,matched_pts_l.Location,im_l);
    [matched_pts_r,validity] = step(LKT, im_r);
    matched_pts_l = matched_pts_l.Location(validity,:);
    matched_pts_r = matched_pts_r(validity,:);
end