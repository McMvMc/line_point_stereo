function [frame_cell, point_database]=point_BA(K,frame_index,frame_cell,point_database)


import gtsam.*

options.triangle = false;
options.nrCameras = 10;
options.showImages = false;

%K=gtsam.Cal3_S2(500,500,0,320,240);
frame_num=size(frame_index,1);

data.K=gtsam.Cal3_S2(K(1),K(5),K(4),K(7),K(8));
data.Z={};
data.J={};
data.F={};
%generate data
for i=1:frame_num
    for j=1:size(frame_cell(frame_index(i)).point_index,1)
        frame_id=frame_cell(frame_index(i)).id;
        point_id=frame_cell(frame_index(i)).point_index(j);
%         disp(frame_id);
%         disp(point_id);
        point=point_database(point_id).pos_in_frame(frame_id);
        data.Z{i}{j}=gtsam.Point2(point(1),point(2));
        data.J{i}{j}=point_database(frame_cell(frame_index(i)).point_index(j)).id;
    end
    data.F{i}=frame_cell(frame_index(i)).id;
end
    


measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

%measurementNoiseSigma = 10;
%pointNoiseSigma = 0;
poseNoiseSigmas = [0 0 0 0 0 0]';

%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;
subgraph=NonlinearFactorGraph;



%Subgraph=subgraph(graph,1);

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
first_frame=frame_cell(frame_index(1));
first_frame_id=first_frame.id;
first_orientation=first_frame.O;
first_position=first_frame.P;
first_pose=Pose3(Rot3(first_orientation),Point3(first_position));
graph.add(PriorFactorPose3(symbol('x',first_frame_id), first_pose, posePriorNoise));


pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);
first_point_index=data.J{1}{1};
first_point_coor3=point_database(first_point_index).coor3;
first_point=Point3(first_point_coor3);      
graph.add(PriorFactorPoint3(symbol('p',first_point_index), first_point, pointPriorNoise));

%% Add factors for all measurements
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
covariance = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);

for i=1:length(data.Z)
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        graph.add(GenericProjectionFactorCal3_S2(data.Z{i}{k}, measurementNoise, symbol('x',data.F{i}), symbol('p',j), data.K));
    end
%    if i>1
%        %fg.add(NonlinearEqualityPose3(0, p0));
%         this_pos_sym=symbol('x',data.F{i});
%         pre_pos_sym=symbol('x',data.F{i-1});
%         this_frame=frame_cell(data.F{i});
%         pre_frame=frame_cell(data.F{i-1});
%         this_O=this_frame.O;
%         this_P=this_frame.P;
%         pre_O=pre_frame.O;
%         pre_P=pre_frame.P;
%         this_pos=Pose3(Rot3(this_O),Point3(this_P));
%         pre_pos=Pose3(Rot3(pre_O),Point3(pre_P));
%         delta = pre_pos.between(this_pos);
%         graph.add(BetweenFactorPose3(pre_pos_sym,this_pos_sym, delta, covariance));
%    end
end
%% Print the graph
%graph.print(sprintf('\nFactor graph:\n'));

%% Initialize cameras and points close to ground truth in this example
initialEstimate = Values;
point_index_all=[];
for i=1:frame_num
    this_frame=frame_cell(frame_index(i));
    this_orientation=this_frame.O;
    this_position=this_frame.P;
    this_pose=Pose3(Rot3(this_orientation),Point3(this_position)).retract(0.1*randn(6,1));
    this_pose=Pose3(Rot3(this_orientation),Point3(this_position));
    initialEstimate.insert(symbol('x',this_frame.id),this_pose);
    point_index_all=[point_index_all;this_frame.point_index];
end
point_index_all=unique(point_index_all);
for i=1:size(point_index_all,1)
    this_point_index=point_index_all(i);
    this_point_coor3=point_database(this_point_index).coor3;
    this_point=Point3(this_point_coor3);
    initialEstimate.insert(symbol('p',this_point_index),this_point);
end
%initialEstimate.print(sprintf('\nInitial estimate:\n  '));
old_loss=copmute_reprojection_loss_from_database(K,frame_index,frame_cell,point_database);

%frame_cell_old=frame_cell;

%% Fine grain optimization, allowing user to iterate step by step
parameters = LevenbergMarquardtParams;
parameters.setlambdaInitial(1.0);
parameters.setVerbosityLM('trylambda');

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);
%optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate);
for i=1:5
    optimizer.iterate();
end

%result = optimizer.optimizeSafely();


result = optimizer.values();
%result.print(sprintf('\nFinal result:\n  '));

%%Update the database
for i=1:size(point_index_all,1)
    this_index=point_index_all(i);
    this_point=result.atPoint3(symbol('p',this_index));
    this_coor3=this_point.vector();
    update_coor3( point_database(this_index),this_coor3);
    %point_database(this_index).coor3=this_coor3;
end
for i=1:size(frame_index,1)
    this_index=frame_index(i);
    this_pose=result.atPose3(symbol('x',this_index));
    this_matrix=this_pose.matrix();
    this_O=this_matrix(1:3,1:3);
    this_P=this_matrix(1:3,4);
    set_O(frame_cell(frame_index(i)),this_O);
    set_P(frame_cell(frame_index(i)),this_P);
%     frame_cell(frame_index(i)).O=this_O;
%     frame_cell(frame_index(i)).R=this_O.';
%     frame_cell(frame_index(i)).P=this_P;
%     frame_cell(frame_index(i)).t=-this_P;
end
    

%%Copmute the reprojection loss

new_loss=copmute_reprojection_loss_from_database(K,frame_index,frame_cell,point_database);
show_old_loss=sprintf('old_loss= %f',old_loss);
disp(show_old_loss);

show_new_loss=sprintf('new_loss= %f',new_loss);
disp(show_new_loss);


%% Plot results with covariance ellipses
% marginals = Marginals(graph, result);
% cla
% hold on;
% 
% plot3DPoints(result, [], marginals);
% plot3DTrajectory(result, '*', 1, 8, marginals);
% 
% axis([-40 40 -40 40 -10 20]);axis equal
% view(3)
% colormap('hot')
