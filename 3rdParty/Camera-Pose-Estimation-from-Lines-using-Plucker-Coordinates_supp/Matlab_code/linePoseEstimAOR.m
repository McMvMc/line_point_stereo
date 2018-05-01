function [ R, t ] = linePoseEstimAOR( line_3D_end_pts, line_2D_end_pts, reproj_err_th, focal, varargin )
%LINEPOSEESTIMAOR Camera Pose Estimation from Lines using Plücker
%Coordinates with Algebraic Outlier Rejection
%
%		INPUT: 
%   line_3D_end_pts - 4x(2N) matrix of 3D line start- and end-points in homogeneous coordinates [x; y; z; w]
%   line_2D_end_pts - 3x(2N) matrix of 2D line start- and end-points in homogeneous coordinates [x; y; w] in the normalized image plane
%   w - Nx1 vector of weights of lines
%   reproj_err_th - threshold for maximal line reprojection error
%   focal - focal length of the camera [px] (used to compute error threshold which depends on focal)
%
%		OUTPUT:
%		R - 3x3 rotation matrix
%		t - 3x1 translation vector
%
% Line reprojection error is the one defined in: C.J.Taylor and D.J.Kriegman:
% "Structure and Motion from Line Segments in Multiple Images".
%

%% Input checks
	if (rem(size(line_3D_end_pts,2), 2))
		error('Number of 3D line endpoints has to be an even number.');
	end
	
	if (size(line_3D_end_pts,1) ~= 4)
		error('3D line endpoints have to be homogeneous coordinates - 4-tuples [x; y; z; w].');
	end
	
	if (size(line_3D_end_pts,2) ~= size(line_2D_end_pts,2))
		error('Number of 3D and 2D line endpoints has to be equal.');
    end
	
	if (size(line_2D_end_pts,1) ~= 3)
		error('2D line endpoints have to be homogeneous coordinates - 3-tuples [x; y; w].');
	end

	NLINES = size(line_3D_end_pts,2)/2;
	
	if (NLINES < 9)
		error('At least 9 lines has to be supplied.');
	end
	
	if (nargin < 5)
		w = ones(NLINES, 1);
	else
		w = varargin{1};
		if (~all(size(w) == [NLINES 1]))
			error('`w` has to be a NLINESx1 vector.');
		end
	end
	
	if (nargin < 6)
		zPM = +1;
	else
		zPM = varargin{2};
		if (~isscalar(zPM))
			error('zPM has to be either +1 or -1.');
		end
	end
	

	%% Create Plücker representation of 3D lines
	lines_3D = pluckerLines(line_3D_end_pts);
		
	%% Construct 2D line equations from projected endpoints
	lines_2D = cross(line_2D_end_pts(:, 1:2:end), line_2D_end_pts(:, 2:2:end));
		
	%% Reject outliers algebraicly
	w_inliers = filterOutliers(lines_3D, lines_2D, w, reproj_err_th, focal);
	
	%% Compute the final solution from inliers only
	[ R, t ] = linePoseEstim(line_3D_end_pts, line_2D_end_pts, true, w_inliers, zPM);
	return;
end

