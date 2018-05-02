function visualize_from_database(key_frame_index, frame_cell, point_database,R_r_gt_rel,t_r_gt_rel)

%figure;
frame_num=size(key_frame_index,1);
for i=1: frame_num
    this_frame_index=key_frame_index(i);
    this_frame=frame_cell(this_frame_index);
    O_l_g=get_O(this_frame);
    P_l_g=get_P(this_frame);
    [O_r_g, P_r_g] = right_from_left_cam(O_l_g, P_l_g, R_r_gt_rel, t_r_gt_rel);
    this_frame_point_indexes=get_point_index(this_frame);
    this_frame_point_indexes_size=size(this_frame_point_indexes,1);
    cur_3D_pts=zeros(this_frame_point_indexes_size,3);
    for j=1:this_frame_point_indexes_size
        this_point=point_database(this_frame_point_indexes(j));
        cur_3D_pts(j,:)=this_point.coor3.';
    end
    draw_map_database(cur_3D_pts, O_l_g, P_l_g, O_r_g, P_r_g);
end