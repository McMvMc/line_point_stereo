%%%
% Test of out method on the VGG multiview dataset.
%%%

% INSTRUCTIONS:
%
% 1) Choose a dataset by uncommenting it below.
% 2) Run this script.
% 3) Images overlaid with reprojected 3D lines will can be seen in the
%    respective dataset subdirectory VGG/[dataset].

%% Settings for individual datasets - please uncomment/comment as needed.

startup;

% Comment this and uncomment the desired dataset
% disp('Please comment this and uncomment a dataset of your choice in this script.'); return;

% % Merton College I
% folderPath = ['VGG' filesep 'Merton-College-I' filesep];
% l3DFileName = 'l3d';
% lineMatchFileName = 'nview-lines';
% img1Nr = 1;
% NIMGS = 3;
% imgPartName = '';
% imgExt = '.jpg';
% sceneSpaceZhalfAxis = -1;

% % Merton College II
% folderPath = ['VGG' filesep 'Merton-College-II' filesep];
% l3DFileName = 'l3d';
% lineMatchFileName = 'nview-lines';
% img1Nr = 1;
% NIMGS = 3;
% imgPartName = '';
% imgExt = '.jpg';
% sceneSpaceZhalfAxis = -1;

% % Merton College III
% folderPath = ['VGG' filesep 'Merton-College-III' filesep];
% l3DFileName = 'l3d';
% lineMatchFileName = 'nview-lines';
% img1Nr = 1;
% NIMGS = 3;
% imgPartName = '';
% imgExt = '.jpg';
% sceneSpaceZhalfAxis = -1;

% % University Library
% folderPath = ['VGG' filesep 'University-Library' filesep];
% l3DFileName = 'l3d';
% lineMatchFileName = 'nview-lines';
% img1Nr = 1;
% NIMGS = 3;
% imgPartName = '';
% imgExt = '.jpg';
% sceneSpaceZhalfAxis = -1;

% % Wadham College
folderPath = ['VGG' filesep 'Wadham-College' filesep];
l3DFileName = 'l3d';
lineMatchFileName = 'nview-lines';
img1Nr = 1;
NIMGS = 5;
imgPartName = '';
imgExt = '.jpg';
sceneSpaceZhalfAxis = +1;

% % Corridor
% folderPath = ['VGG' filesep 'Corridor' filesep];
% l3DFileName = 'bt.l3d';
% lineMatchFileName = 'bt.nview-lines';
% img1Nr = 0;
% NIMGS = 11;
% imgPartName = 'bt.';
% imgExt = '.pgm';
% sceneSpaceZhalfAxis = -1;

%% Run our estimation method on all images from a dataset
imgNrs = img1Nr:(NIMGS + img1Nr - 1);

for imgNr = imgNrs
	
	l2DFileName    = [imgPartName sprintf('%03d', imgNr) '.lines'];
	camMatFileName = [imgPartName sprintf('%03d', imgNr) '.P'];
	imgName        = [imgPartName sprintf('%03d', imgNr) imgExt];
	
	%% read 3D line endpoints
	endPts3D = dlmread([folderPath l3DFileName]);
	endPts3D = endPts3D';
	
	%% read 3D-2D line matches
	f = fopen([folderPath lineMatchFileName], 'r');
	matches = textscan(f, '%s', 'EndOfLine', '\n');
	fclose(f);
	matches = matches{1,1};

	for i=1:length(matches(:))
		num = str2num(matches{i});
		if(~isempty(num))
			matches{i} = num;
		else
			matches{i} = NaN;
		end
	end

	matches = cell2mat(matches);
	matches = reshape(matches, NIMGS, size(endPts3D,2));
	matches = matches';

	% keep just matches for current image
	matches = matches(:, (imgNr - img1Nr + 1));
	
	NLINES = sum(~isnan(matches));
	
	%% filter our nonmatched 3D lines
	endPts3D = endPts3D(:, ~isnan(matches));

	line3DEndPts = ones(4, 2*NLINES);
	line3DEndPts(1:3, 1:2:end) = endPts3D(1:3, :);
	line3DEndPts(1:3, 2:2:end) = endPts3D(4:6, :);
	
	%% read camera matrix
	P_GT = dlmread([folderPath camMatFileName]);
	[C_GT, ~, ~] = vgg_KR_from_P(P_GT);
	
	%% save images with reprojected lines (GROUND TRUTH)
	fig = figure();
	
	subplot(1, 2, 1);
	img = imread([folderPath imgName]);
	imshow(img);
	hold on;
	
	lineEndPtsImg_GT = P_GT * line3DEndPts;
	for i = 1:3
		lineEndPtsImg_GT(i,:) = lineEndPtsImg_GT(i,:) ./ lineEndPtsImg_GT(3,:);
	end
	
	plot([lineEndPtsImg_GT(1, 1:2:end); lineEndPtsImg_GT(1, 2:2:end)],  [lineEndPtsImg_GT(2, 1:2:end); lineEndPtsImg_GT(2, 2:2:end)], 'Color', 'red', 'LineWidth', 1.0);
	
	hold off;
	title('Ground-truth');
	
	%% read 2D lines
	endPtsImg = dlmread([folderPath l2DFileName]);
	endPtsImg = endPtsImg(:, 1:4);
	endPtsImg = endPtsImg';

	% filter out nonmatched 2D lines
	endPtsImg = endPtsImg(:, matches(~isnan(matches)) + 1);

	lineEndPtsImg_estim = ones(3, 2*NLINES);
	lineEndPtsImg_estim(1:2, 1:2:end) = endPtsImg(1:2,:);
	lineEndPtsImg_estim(1:2, 2:2:end) = endPtsImg(3:4,:);

	line2DEndPts_estim = inv(C_GT) * lineEndPtsImg_estim;
	
	%% camera pose estimation
	[R_estim, t_estim] = linePoseEstim( line3DEndPts, line2DEndPts_estim, true, ones(NLINES, 1), sceneSpaceZhalfAxis );
	
	%% save images with reprojected lines
	subplot(1, 2, 2);
	imshow(img);
	hold on;
	
	[camRotX_estim, camRotY_estim, camRotZ_estim] = rotMatrix2EulerAngles(R_estim);
	TM_estim = getTransformationMatrix( ...
		camRotX_estim, camRotY_estim, camRotZ_estim, ...
		-t_estim(1), -t_estim(2), -t_estim(3)  ...
	);

	line2DEndPts_estim  = TM_estim * line3DEndPts;
	lineEndPtsImg_estim =  C_GT * line2DEndPts_estim;
	for i = 1:3
		lineEndPtsImg_estim(i,:) = lineEndPtsImg_estim(i,:) ./ lineEndPtsImg_estim(3,:);
	end
	
	plot([lineEndPtsImg_estim(1, 1:2:end); lineEndPtsImg_estim(1, 2:2:end)],  [lineEndPtsImg_estim(2, 1:2:end); lineEndPtsImg_estim(2, 2:2:end)], 'Color', 'red', 'LineWidth', 1.0);
	
	hold off;
	title('Estimated');

% 	close(fig);

end

fprintf('\nSeveral figures have been drawn.\n');

