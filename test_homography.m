function err = test_homography(prev_matched_pts_l, matched_pts_l)
    homography = estimateGeometricTransform(prev_matched_pts_l,matched_pts_l,...
                                                                'projective');
    [x, y] = transformPointsForward(homography,prev_matched_pts_l(:,1),...
                                                    prev_matched_pts_l(:,2));

    err = [x-prev_matched_pts_l(:,1) y-prev_matched_pts_l(:,2)];
    err = vecnorm(err, 2, 2);
end