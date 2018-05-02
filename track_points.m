function [matched_pts_l, matched_pts_r, validity] = ...
                            track_points(matched_pts_l, im_l, im_r, use_matching, ...
                            cur_pt_ids, global_descriptor)
    
    LKT = vision.PointTracker('MaxBidirectionalError',1,...
                        'BlockSize', [31,31]);
    initialize(LKT,matched_pts_l,im_l);

    [matched_pts_r,validity] = step(LKT, im_r);
    matched_pts_l = matched_pts_l(validity,:);
    matched_pts_r = matched_pts_r(validity,:);
    
% 	if use_matching && size(cur_pt_ids, 2)~=0
%         
%         initialize(LKT,matched_pts_l,im_l);
%         [cur_tracked_pts_r,validity] = step(LKT, im_r);
%         cur_tracked_pts_l = matched_pts_l(validity,:);
%         
%         if size(cur_tracked_pts_l, 1)<size(matched_pts_l, 1)
%             not_tracked_idx = ~validity;
%             cur_pt_ids = cur_pt_ids(not_tracked_idx);
%             
%             desc_l = binaryFeatures(global_descriptor.Features(cur_pt_ids, :));
%             [desc_r, not_tracked_pts_r] = detect_points(im_r);
%             
%             [match_idx_pairs,~] = matchFeatures(desc_l, desc_r,...
%                                         'Method','Exhaustive', 'Unique', true);
%             
%             not_tracked_pts_r = not_tracked_pts_r.Location(match_idx_pairs(:,2),:);
%             
%             tmp_validity = validity(not_tracked_idx);
%             tmp_validity(match_idx_pairs(:,1)) = true;
%             
%             validity(not_tracked_idx) = tmp_validity;
%             
%             matched_pts_l = matched_pts_l(validity);
%             matched_pts_r = [cur_tracked_pts_r; not_tracked_pts_r];
%         else
%             matched_pts_l = cur_tracked_pts_l;
%             matched_pts_r = cur_tracked_pts_r(validity,:);
%         end
%         
%     else
%         initialize(LKT,matched_pts_l,im_l);
%         
%         [matched_pts_r,validity] = step(LKT, im_r);
%         matched_pts_l = matched_pts_l(validity,:);
%         matched_pts_r = matched_pts_r(validity,:);
%     end
    
end