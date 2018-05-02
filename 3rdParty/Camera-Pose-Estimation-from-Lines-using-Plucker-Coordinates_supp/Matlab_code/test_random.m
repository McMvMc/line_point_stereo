%%%
% Test of our method using randomly generated lines.
%%%

startup;

%% TEST PARAMETERS
NLINES   = 9; % how many lines to generate
CUBESIZE = 10.0; % size of one side of an imaginary cube inside of which the lines are generated
CAMDIST  = 25.0; % distance of the camera from the center of the imaginary cube

%% GROUND TRUTH camera pose (randomly generated)
% camera position
camX_GT = 2*rand()-1;
camY_GT = 2*rand()-1;
camZ_GT = 2*rand()-1;

normCoef = CAMDIST / norm([camX_GT camY_GT camZ_GT]);

camX_GT = normCoef * camX_GT;
camY_GT = normCoef * camY_GT;
camZ_GT = normCoef * camZ_GT;

t_GT = getTranslationVector(camX_GT, camY_GT, camZ_GT);

% camera orientation
if(rand < 0.5)
	camRotZ_GT = -atan2(camX_GT, camY_GT);
	camRotY_GT = 0;
	camRotX_GT = -acos((camZ_GT - 0) / CAMDIST);
else
	camRotZ_GT = atan2(camY_GT, camX_GT);
	camRotY_GT = acos((camZ_GT - 0) / CAMDIST);
	camRotX_GT = 0;
end

R_GT = getRotationMatrix(camRotX_GT, camRotY_GT, camRotZ_GT);

TM_GT = getTransformationMatrix(camRotX_GT, camRotY_GT, camRotZ_GT, camX_GT, camY_GT, camZ_GT);


%% 3D LINES (randomly generated)
line3DEndPts = [CUBESIZE * rand(3, 2*NLINES) - (CUBESIZE/2); ones(1, 2*NLINES)];

%% 2D LINES (3D lines projected using generated ground truth camera pose)
line2DEndPts = TM_GT * line3DEndPts;
for i = 1:3
	line2DEndPts(i,:)  = line2DEndPts(i,:)  ./ line2DEndPts(3,:);
end

%% CAMERA POSE ESTIMATION
[R_estim, t_estim] = linePoseEstim( line3DEndPts, line2DEndPts );

%% OUTPUT COMPARISON
disp('Ground truth [R|t]');
disp([R_GT    t_GT   ]);

disp('Estimated pose [R|t]');
disp([R_estim t_estim]);
