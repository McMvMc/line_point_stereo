function [lines] = detect_lines(im, pts)
% detect lines around the given points
% im: MxN gray scale image
% pts: Kx2 K*(x, y) array 
% lines: Lx4 L*(x1, y1, x2, y2) array


% check input type
% convert im to double array
% convert pts to double array

if ~isa(im, 'double')
    im = im2double(im); 
end

if ~isa(pts, 'double')
    pts = double(pts);
end

pts = [fliplr(pts)';5*ones(1, size(pts, 1))];
lines = lsd_mex(im, pts, 1.0, 1, 0.02, 30, 00.0, 0.5, 1024);


lines = [lines(2,:)', lines(1,:)', lines(4,:)', lines(3,:)']+1;

end

