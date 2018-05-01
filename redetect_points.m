function [prev_matched_pts_l, prev_matched_pts_r, new_prev_matched_pts_l, ...
            new_prev_matched_pts_r, new_matched_pts_l, new_matched_pts_r, ...
            cur_3D_pts, cur_pt_ids, global_descriptor, matched_pts_l, matched_pts_r]...
             = redetect_points(prev_im_l, prev_im_r, im_l, im_r, prev_matched_pts_l, ...
                matched_pts_l, matched_pts_r, cur_3D_pts, cur_pt_ids, global_descriptor)
    % track previous right points
    [prev_matched_pts_l, prev_matched_pts_r, prev_valid_r] = ...
                    track_points(prev_matched_pts_l, prev_im_l, prev_im_r);
    matched_pts_l = matched_pts_l(prev_valid_r, :);
    matched_pts_r = matched_pts_r(prev_valid_r, :);
    cur_3D_pts = cur_3D_pts(prev_valid_r, :);
    cur_pt_ids = cur_pt_ids(prev_valid_r);

    % detect and track new points
    [new_prev_desc_l, new_prev_matched_pts_l] = detect_points(prev_im_l);
    new_prev_matched_pts_l = new_prev_matched_pts_l.Location;

    [new_prev_matched_pts_l, new_prev_matched_pts_r, prev_valid_r] = ...
                    track_points(new_prev_matched_pts_l, prev_im_l, prev_im_r);
    [new_prev_matched_pts_l, new_matched_pts_l, valid_l] = ...
                        track_points(new_prev_matched_pts_l, prev_im_l, im_l);
    [new_matched_pts_l, new_matched_pts_r, valid_r] = ...
                        track_points(new_matched_pts_l, im_l, im_r);

    new_prev_matched_pts_l = new_prev_matched_pts_l(valid_r,:);
    new_prev_matched_pts_r = new_prev_matched_pts_r(valid_l,:);
    new_prev_matched_pts_r = new_prev_matched_pts_r(valid_r,:);
    new_prev_desc_l = new_prev_desc_l.Features(prev_valid_r,:);
    new_prev_desc_l = new_prev_desc_l(valid_l,:);
    new_prev_desc_l = new_prev_desc_l(valid_r,:);
    new_prev_desc_l = binaryFeatures(new_prev_desc_l);

    % find correspondence and new features
    [match_idx_pairs,~] = matchFeatures(new_prev_desc_l, global_descriptor,...
                                    'Method','Approximate');
    matched_idx = match_idx_pairs(:,1);
    tmp_new_idx = 1:size(new_prev_matched_pts_l, 1);
    new_feat_ids = setdiff(tmp_new_idx, matched_idx);

    global_descriptor = binaryFeatures([global_descriptor.Features;...
                                new_prev_desc_l.Features(new_feat_ids, :)]);
    new_matched_pts_l = new_matched_pts_l(new_feat_ids, :);
    new_matched_pts_r = new_matched_pts_r(new_feat_ids, :);
    new_prev_matched_pts_l = new_prev_matched_pts_l(new_feat_ids, :);
    new_prev_matched_pts_r = new_prev_matched_pts_r(new_feat_ids, :);
    
    last_id = size(cur_pt_ids, 2);
    cur_pt_ids = [cur_pt_ids last_id:(last_id+size(new_feat_ids, 2)-1)];
end