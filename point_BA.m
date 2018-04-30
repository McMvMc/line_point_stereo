%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% GTSAM Copyright 2010, Georgia Tech Research Corporation, 
% Atlanta, Georgia 30332-0415
% All Rights Reserved
% Authors: Frank Dellaert, et al. (see THANKS for the full author list)
% 
% See LICENSE for the license information
%
% @brief A structure from motion example
% @author Duy-Nguyen Ta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

import gtsam.*

options.triangle = false;
options.nrCameras = 10;
options.showImages = false;

K=gtsam.Cal3_S2(500,500,0,320,240);
frame_num=size(frame_cell,1);
point_database
data.K=K;
data.Z={};
data.J={};
data.F={};
for i=1:frame_num
    for j=1:size(frame_cell{i}.point_index,1)
        point=point_database(frame_cell{i}.point_index(j)).pos_in_frame(frame_cell{i}.id);
        data.Z{i}{j}=gtsam.Point2(point(1),point(2));
        data.J{i}{j}=point_database(frame_cell{i}.point_index(j)).id;
    end
    data.F{i}=frame_cell{i}.id;
end
    


measurementNoiseSigma = 1.0;
pointNoiseSigma = 0.1;
poseNoiseSigmas = [0.001 0.001 0.001 0.1 0.1 0.1]';

%% Create the graph (defined in visualSLAM.h, derived from NonlinearFactorGraph)
graph = NonlinearFactorGraph;
subgraph=NonlinearFactorGraph;
%% Add factors for all measurements
measurementNoise = noiseModel.Isotropic.Sigma(2,measurementNoiseSigma);
covariance = noiseModel.Diagonal.Sigmas([5*pi/180; 5*pi/180; 5*pi/180; 0.05; 0.05; 0.05]);

for i=1:length(data.Z)
    for k=1:length(data.Z{i})
        j = data.J{i}{k};
        graph.add(GenericProjectionFactorCal3_S2(data.Z{i}{k}, measurementNoise, symbol('x',data.F{i}), symbol('p',j), data.K));
    end
    if i>1
        this_pos=symbol('x',data.F{i});
        pre_pos=symbol('x',data.F{i-1});
        delta = pre_pos.between(this_pos);
        graph.add(BetweenFactorPose3(pre_pos,this_pos, delta, covariance));
    end
end


%Subgraph=subgraph(graph,1);

%% Add Gaussian priors for a pose and a landmark to constrain the system
posePriorNoise  = noiseModel.Diagonal.Sigmas(poseNoiseSigmas);
first_frame=frame_cell{1};
first_orientation=first_frame.O;
first_position=first_frame.P;
first_pose=Pose3(Rot3(first_orientation),Point3(first_position));
graph.add(PriorFactorPose3(symbol('x',pre_pos), first_pose, posePriorNoise));
pointPriorNoise  = noiseModel.Isotropic.Sigma(3,pointNoiseSigma);

first_point_index=data.J{1}{1};
first_point_coor3=point_database(first_point_index).coor3;
first_point=Pose3(this_point_coor3);      
graph.add(PriorFactorPoint3(symbol('p',first_point_index), first_point, pointPriorNoise));


%% Print the graph
graph.print(sprintf('\nFactor graph:\n'));

%% Initialize cameras and points close to ground truth in this example
initialEstimate = Values;
point_index_all=[];
for i=1:frame_num
    this_frame=frame_cell{i};
    this_orientation=this_frame.O;
    this_position=this_frame.P;
    this_pose=Pose3(Rot3(this_orientation),Point3(this_position));
    initialEstimate.insert(symbol('x',this_frame.id),this_pose);
    point_index_all=[point_index_all;this_frame.point_index];
end
point_index_all=unique(point_index_all);
for i=1:size(point_index_all,1)
    this_point_index=point_index_all(i);
    this_point_coor3=point_database(this_point_index).coor3;
    this_point=Pose3(this_point_coor3);
    initialEstimate.insert(symbol('p',this_point_index),this_point);
end
initialEstimate.print(sprintf('\nInitial estimate:\n  '));

%% Fine grain optimization, allowing user to iterate step by step
parameters = LevenbergMarquardtParams;
parameters.setlambdaInitial(1.0);
parameters.setVerbosityLM('trylambda');

optimizer = LevenbergMarquardtOptimizer(graph, initialEstimate, parameters);

for i=1:5
    optimizer.iterate();
end
result = optimizer.values();
result.print(sprintf('\nFinal result:\n  '));

%% Plot results with covariance ellipses
marginals = Marginals(graph, result);
cla
hold on;

plot3DPoints(result, [], marginals);
plot3DTrajectory(result, '*', 1, 8, marginals);

axis([-40 40 -40 40 -10 20]);axis equal
view(3)
colormap('hot')
