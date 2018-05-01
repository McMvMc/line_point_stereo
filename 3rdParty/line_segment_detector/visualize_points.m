function [overlay] = visualize_points(im, pts)
%SHOW_LINES Summary of this function goes here
%   im: MxN 
%   lines: K*4 K*(x1, y1, x2, y2)

if ~isa(im, 'double')
    im = im2double(im);
end

if size(im, 3) == 1
    im = repmat(im, 1, 1, 3);
end

overlay = insertShape(im, 'Circle', [pts, 5*ones(size(pts, 1), 1)],...
    'LineWidth', 1, 'Color', 'r');
overlay = insertShape(overlay, 'Circle', [pts, 1*ones(size(pts, 1), 1)],...
    'LineWidth', 1, 'Color', 'r');

end

