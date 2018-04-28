function [matched_pts_l, matched_pts_r, desc_l, desc_r] = ...
                        match_points(im_l, im_r, desc_l, valid_pts_l, ...
                                                desc_r, valid_pts_r)
    show_detection(im_l, im_r, pts_l, pts_r, valid_pts_l, valid_pts_r);
    indexPairs = matchFeatures(desc_l, desc_r, 'MaxRatio', 0.75);
    matched_pts_l = valid_pts_l(indexPairs(:,1));
    matched_pts_r = valid_pts_r(indexPairs(:,2));
end