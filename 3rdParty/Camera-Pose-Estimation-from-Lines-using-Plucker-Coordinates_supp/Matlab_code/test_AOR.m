%%%
% Test of our method with AOR (Algebraic Outlier Rejection) using randomly
% generated lines.
%%%

startup;

%% TEST PARAMETERS
CUBESIZE = 10; % size of one side of the cube
CAMDIST  = 25.0; % distance of the camera from the center of the imaginary cube
NLINES = 40; % how many lines to generate
SIGMA   = 1;
NOUTLIERS = 3;
OUTL_SIGMA = 100;
IMG_SIZE = [480 640];
FOCAL = -800;
PLOT = true; % whether to plot into figures

%% Check param values
if (NOUTLIERS >= NLINES)
	error('The number of all lines NLINES must be smaller than the number of outliers NOUTLIERS.')
end

REPROJ_ERR_TH_AOR = SIGMA;

%% GROUND TRUTH camera pose (randomly generated)
% camera position
camX_GT = 2*rand()-1;
camY_GT = 2*rand()-1;
camZ_GT = 2*rand()-1;

normCoef = CAMDIST / norm([camX_GT camY_GT camZ_GT]);

camX_GT = normCoef * camX_GT;
camY_GT = normCoef * camY_GT;
camZ_GT = normCoef * camZ_GT;

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
t_GT = getTranslationVector(camX_GT, camY_GT, camZ_GT);

TM_GT = getTransformationMatrix(camRotX_GT, camRotY_GT, camRotZ_GT, camX_GT, camY_GT, camZ_GT);

principalX = (IMG_SIZE(2)-1) / 2;
principalY = (IMG_SIZE(1)-1) / 2;
C  = getCameraMatrix(FOCAL, principalX, principalY);

%% Generate lines
% 3D LINES (randomly generated)
line3DEndPts = [CUBESIZE * rand(3, 2*NLINES) - (CUBESIZE/2); ones(1, 2*NLINES)];

% 2D LINES (3D lines projected using generated ground truth camera pose)
line2DEndPts = TM_GT * line3DEndPts;
for i = 1:3
	line2DEndPts(i,:)  = line2DEndPts(i,:)  ./ line2DEndPts(3,:);
end

lineEndPtsImg =  C * line2DEndPts;
for i = 1:3
	lineEndPtsImg(i,:) = lineEndPtsImg(i,:) ./ lineEndPtsImg(3,:);
end

%% Add noise

% add gaussian noise to the endpoints of all lines to make the inliers
% worse distinguishable
lineEndPtsImgNoisy = lineEndPtsImg;
lineEndPtsImgNoisy(1:2, :) = lineEndPtsImgNoisy(1:2, :) + (SIGMA * randn(2, 2*NLINES));

% produce outliers by strongly perturbing endpoints of image lines
lineEndPtsImgNoisy(1:2, end-2*NOUTLIERS+1:end) = lineEndPtsImgNoisy(1:2, end-2*NOUTLIERS+1:end) + (OUTL_SIGMA * randn(2, 2*NOUTLIERS));


line2DEndPtsNoisy = inv(C) * lineEndPtsImgNoisy;

%% CAMERA POSE ESTIMATION
[ R_estim, t_estim ] = linePoseEstimAOR( line3DEndPts, line2DEndPtsNoisy, REPROJ_ERR_TH_AOR, FOCAL );

%% OUTPUT COMPARISON
disp('Ground truth [R|t]');
disp([R_GT    t_GT   ]);

disp('Estimated pose [R|t]');
disp([R_estim t_estim]);

%% PLOT
if (PLOT)
	% create figure
	if (exist('figLinePoseEstimOutliers') && ~isempty(figLinePoseEstimOutliers) && ishandle(figLinePoseEstimOutliers))
		figure(figLinePoseEstimOutliers);
	else
		figLinePoseEstimOutliers = figure();
	end;
	cla;
	
	% plot the images of line segments by connecting projected endpoints
	matlabImgTForm = [ ...
		1    0      1;    ...
		0   -1  IMG_SIZE(1); ...
		0    0      1     ...
	];

	lineEndPtsImgMatlab      = matlabImgTForm * lineEndPtsImg;
	lineEndPtsImgMatlabNoisy = matlabImgTForm * lineEndPtsImgNoisy;
	for i = 1:3
		lineEndPtsImgMatlab(i,:)      = lineEndPtsImgMatlab(i,:)      ./ lineEndPtsImgMatlab(3,:);
		lineEndPtsImgMatlabNoisy(i,:) = lineEndPtsImgMatlabNoisy(i,:) ./ lineEndPtsImgMatlabNoisy(3,:);
	end
	
	hold on;
	colorIn = [0.4 0.6 1.0];
	colorOut = 'red';
	plot( ... % plot original lines
		[lineEndPtsImgMatlab(1, 1:2:end); lineEndPtsImgMatlab(1, 2:2:end)], ...
		[lineEndPtsImgMatlab(2, 1:2:end); lineEndPtsImgMatlab(2, 2:2:end)], ...
		'Color', colorIn,    'LineWidth', 2.0 ...
	);
	plot( ... % plot noisy lines
		[lineEndPtsImgMatlabNoisy(1, 1:2:end-(2*NOUTLIERS)); lineEndPtsImgMatlabNoisy(1, 2:2:end-(2*NOUTLIERS))], ...
		[lineEndPtsImgMatlabNoisy(2, 1:2:end-(2*NOUTLIERS)); lineEndPtsImgMatlabNoisy(2, 2:2:end-(2*NOUTLIERS))], ...
		'Color', 'blue',    'LineWidth', 2.0 ...
	);
	plot( ... % plot outliers
		[lineEndPtsImgMatlabNoisy(1, end-(2*NOUTLIERS)+1:2:end); lineEndPtsImgMatlabNoisy(1, end-(2*NOUTLIERS)+2:2:end)], ...
		[lineEndPtsImgMatlabNoisy(2, end-(2*NOUTLIERS)+1:2:end); lineEndPtsImgMatlabNoisy(2, end-(2*NOUTLIERS)+2:2:end)], ...
		'Color', colorOut,    'LineWidth', 2.0 ...
	);
	plot( ... % plot noisy endpoints
		[lineEndPtsImgMatlabNoisy(1, 1:2:end-(2*NOUTLIERS)); lineEndPtsImgMatlabNoisy(1, 2:2:end-(2*NOUTLIERS))], ...
		[lineEndPtsImgMatlabNoisy(2, 1:2:end-(2*NOUTLIERS)); lineEndPtsImgMatlabNoisy(2, 2:2:end-(2*NOUTLIERS))], ...
		'+b' ...
	);
	plot( ... % plot outliers' endpoints
		[lineEndPtsImgMatlabNoisy(1, end-(2*NOUTLIERS)+1:2:end); lineEndPtsImgMatlabNoisy(1, end-(2*NOUTLIERS)+2:2:end)], ...
		[lineEndPtsImgMatlabNoisy(2, end-(2*NOUTLIERS)+1:2:end); lineEndPtsImgMatlabNoisy(2, end-(2*NOUTLIERS)+2:2:end)], ...
		'+', 'Color', colorOut  ...
	);
	hold off;
	
	[rotXAngle_estim, rotYAngle_estim, rotZAngle_estim] = rotMatrix2EulerAngles(R_estim);

	TM_estim = getTransformationMatrix( ...
		rotXAngle_estim, rotYAngle_estim, rotZAngle_estim, ...
		-t_estim(1), -t_estim(2), -t_estim(3)  ...
	);

	line2DEndPts_estim        = TM_estim             * line3DEndPts;
	lineEndPtsImgMatlab_estim = (matlabImgTForm * C) * line2DEndPts_estim;
	for i = 1:3
		lineEndPtsImgMatlab_estim(i,:) = lineEndPtsImgMatlab_estim(i,:) ./ lineEndPtsImgMatlab_estim(3,:);
	end

	hold on;
	plot( ... % plot outliers' endpoints
		[lineEndPtsImgMatlab_estim(1, 1:2:end); lineEndPtsImgMatlab_estim(1, 2:2:end)], ...
		[lineEndPtsImgMatlab_estim(2, 1:2:end); lineEndPtsImgMatlab_estim(2, 2:2:end)], ...
		'Color', [0 0.8 0], 'LineWidth', 2.0  ...
	);
	hold off;
	
	axis equal;
	xlim([1 IMG_SIZE(2)]);
	ylim([1 IMG_SIZE(1)]);
	set(gca,'YDir','reverse');
	set(gca, 'XAxisLocation', 'top');
	
	fprintf('PLOT LEGEND:\n');
	fprintf('Light blue - original 3D lines projected into image\n');
	fprintf('Dark blue - image lines perturbed with noise (inliers)\n');
	fprintf('Red - image lines perturbed with noise (outliers)\n');
	fprintf('Green - image lines projected using the ESTIMATED CAMERA POSE\n');
end
