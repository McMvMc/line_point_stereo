function loss=copmute_reprojection_loss_from_database(K,frame_index,frame_cell,point_database)
loss=0;
frame_num=size(frame_index,1);
for i=1:frame_num
    this_frame_index=frame_index(i);
    this_frame=frame_cell(this_frame_index);
    this_frame_point_indexes=this_frame.point_index;
    this_frame_point_size=size(this_frame_point_indexes,1);
    this_frame_points=ones(4,this_frame_point_size);
    this_frame_impoints2=ones(2,this_frame_point_size);
    %need to consider if it is R or O!!!
    this_frame_matrix=[this_frame.R,this_frame.t;zeros(1,4)];
    for j=1:this_frame_point_size
        this_frame_points(1:3,j)=point_database(this_frame_point_indexes(j)).coor3;
        this_frame_impoints2(:,j)=point_database(this_frame_point_indexes(j)).pos_in_frame(this_frame_index).';
    end
    this_frame_points3=this_frame_matrix*this_frame_points;
    this_frame_points3=this_frame_points3(1:3,:);
    this_frame_points3=K*this_frame_points3;
    this_frame_points2=this_frame_points3./this_frame_points3(3,:);
    this_frame_points2=this_frame_points2(1:2,:);
    this_frame_dis=this_frame_points2-this_frame_impoints2;
    this_frame_dis=this_frame_dis.^2;
    this_loss=sum(sqrt(sum(this_frame_dis)));
    %this_loss=sum((sum(this_frame_dis)));
    loss=loss+this_loss;
end
    
    
    