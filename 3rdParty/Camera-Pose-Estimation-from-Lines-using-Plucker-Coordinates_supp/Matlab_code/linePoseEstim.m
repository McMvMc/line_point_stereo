function [ R, t ] = linePoseEstim( line_3D_end_pts, line_2D_end_pts, varargin )
%LINEPOSEESTIM Camera Pose Estimation from Lines using Plücker Coordinates.
%
%		INPUT: 
%   line_3D_end_pts - 4x(2N) matrix of 3D line start- and end-points in homogeneous coordinates [x; y; z; w]
%   line_2D_end_pts - 3x(2N) matrix of 2D line start- and end-points in homogeneous coordinates [x; y; w] in the normalized image plane
%		OPTIONAL INPUT:
%   prenormalize - true if prenormalization should be carried out; default: true
%   w - Nx1 vector of weights of lines; default: ones
%   zPM - +1 or -1 as the direction of the Z axis; default: +1
%
%		OUTPUT:
%		R - 3x3 rotation matrix
%		t - 3x1 translation vector
%
	
	% input checks
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
	
	% set default parameter values
	if (nargin < 3)
		prenormalize = true;
	else
		prenormalize = varargin{1};
		if (~islogical(prenormalize) || ~isscalar(prenormalize))
			error('`prenormalize` has to be either true or false.');
		end
	end
	
	if (nargin < 4)
		w = ones(NLINES, 1);
	else
		w = varargin{2};
		if (~all(size(w) == [NLINES 1]))
			error('`w` has to be a NLINESx1 vector.');
		end
	end
	
	if (nargin < 5)
		zPM = +1;
	else
		zPM = varargin{3};
		if (~isscalar(zPM))
			error('zPM has to be either +1 or -1.');
		end
	end
	
	
	%% Drop lines and weights which have w==0
	w_nzero = (w ~= 0);
	w = w(w_nzero);
	
	w_nzero_end_pts = [w_nzero w_nzero]';
	w_nzero_end_pts = w_nzero_end_pts(:);
	
	line_3D_end_pts = line_3D_end_pts(:, w_nzero_end_pts);
	line_2D_end_pts = line_2D_end_pts(:, w_nzero_end_pts);
	
	NLINES = size(line_3D_end_pts,2)/2;
	
	if (NLINES < 9)
		error('At least 9 lines with non-zero weights has to be supplied.');
	end
	
	
	%% Compute the centroid of 3D lines endpoints
	% to test whether camera does look at te 3D structure
	center_3D = mean([ ...
		line_3D_end_pts(1,:) ./ line_3D_end_pts(4,:); ...
		line_3D_end_pts(2,:) ./ line_3D_end_pts(4,:); ...
		line_3D_end_pts(3,:) ./ line_3D_end_pts(4,:) ...
	], 2);
	
	
	%% Pre-normalization of 3D Plucker lines
	if(prenormalize)
		% proper pre-normalization (translation and isotropic scaling) of 3D
		% Plucker lines cannot be done here as Plucker homogeneous 6-tuples can
		% have 0 coordinates (~ 5D homog points at infinity). Translation to
		% the Origin must suffice here.
	
		% translate lines so that the closest point to them is the Origin
		shift_3D = closestPoint2SetOfLines(line_3D_end_pts, w);
		pre_tform_3D_pts = [eye(3) -shift_3D(1:3); 0 0 0 1];
	else
		shift_3D = [0; 0; 0];
		pre_tform_3D_pts = eye(4);
	end
	
	line_3D_end_pts = pre_tform_3D_pts * line_3D_end_pts;
	
	
	%% Create Plücker representation of 3D lines
	lines_3D = pluckerLines(line_3D_end_pts);
	
	%% Construct 2D line equations from projected endpoints
	lines_2D = cross(line_2D_end_pts(:, 1:2:end), line_2D_end_pts(:, 2:2:end));
	
	
	%% Pre-normalization of 2D lines - treat them as 2D points
	
	if(prenormalize)
		% "translate" lines so that their centroid is at the Origin
		lines_2D_w = lines_2D * diag(w);
		shift_2D = mean([lines_2D_w(1,:) ./ lines_2D_w(3,:); lines_2D_w(2,:) ./ lines_2D_w(3,:)], 2);
		pre_shift_2D_lines = [eye(2) [-shift_2D(1); -shift_2D(2)]; 0 0 1];
		lines_2D_shift = pre_shift_2D_lines * lines_2D;

		% "scale" lines isotropically
		scale_2D = sqrt(2) / mean(sqrt((lines_2D_shift(1,:) ./ lines_2D_shift(3,:)) .^2 + (lines_2D_shift(2,:) ./ lines_2D_shift(3,:)) .^2));
		pre_scale_2D_lines = diag([scale_2D scale_2D 1]);

		% combine the transformations
		pre_tform_2D_lines = pre_scale_2D_lines * pre_shift_2D_lines;
	else
		pre_tform_2D_lines = eye(3);
	end
	
	lines_2D = pre_tform_2D_lines * lines_2D;
	
		
	%% Estimate the camera pose
		
	% estimate of the line projection matrix using Linear Least Squares
	[P_line_est, ~] = fitLineProjMatrix(lines_3D, lines_2D, w);
	
	% post-transformation un-doing the pre-normalizing transformations of 2D lines
	P_line_est = pre_tform_2D_lines \ P_line_est;
	
	% extract the camera pose parameters
	[cam_x0, cam_y0, cam_z0, rot_x0, rot_y0, rot_z0] = getProjParams(P_line_est);	

	
	%% Choose a physically plausible solution, i.e. scene is in front of camera
	
	if (iscell(rot_x0))
		R1 = getRotationMatrix(   rot_x0{1}, rot_y0{1}, rot_z0{1});
		t1 = getTranslationVector(cam_x0{1}, cam_y0{1}, cam_z0{1});
		
		R2 = getRotationMatrix(   rot_x0{2}, rot_y0{2}, rot_z0{2});
		t2 = getTranslationVector(cam_x0{2}, cam_y0{2}, cam_z0{2});
		
		% post-transformation un-doing the pre-normalizing transformations of
		% 3D lines is hidden in the "shift3D" variable
		
		test_1 = sign(zPM) * dot(R1(3,:), (center_3D - shift_3D - t1)/norm(center_3D-shift_3D-t1));
		test_2 = sign(zPM) * dot(R2(3,:), (center_3D - shift_3D - t2)/norm(center_3D-shift_3D-t2));

		if(test_1 > 0 && test_2 < 0)
			R = R1;
			t = t1 - shift_3D;
			return;
		else
			R = R2;
			t = t2 - shift_3D;
			return;
		end
				
	else
		% post-transformation un-doing the pre-normalizing transformations of 3D lines
		cam_x0 = cam_x0 + shift_3D(1);
		cam_y0 = cam_y0 + shift_3D(2);
		cam_z0 = cam_z0 + shift_3D(3);
		
		R   = getRotationMatrix(   rot_x0, rot_y0, rot_z0);
		t   = getTranslationVector(cam_x0, cam_y0, cam_z0);
		return;
	end

end
