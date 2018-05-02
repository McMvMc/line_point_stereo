function [match] = match_lines_stereo(lines_l, lines_r, im_l, im_r, F)

pts_l = [lines_l(:,1:2); lines_l(:,3:4)];
pts_r = [lines_r(:,1:2); lines_r(:,3:4)];
epi_l = epipolarLine(F, pts_l);
epi_l = epi_l./sqrt(sum(epi_l(:,1:2).^2, 2));

num_l = size(lines_l, 1);
num_r = size(lines_r, 1);

dist = epi_l*[pts_r, ones(size(pts_r, 1), 1)]';
dist = abs(dist) < 1;
% limited disparity

dx = (pts_l(:, 1) - pts_r(:,1)');
dx = dx > 0 & dx < 100;
% dy_l = abs(pts_l(1:num_l, 2) - pts_l(num_l+1:end, 2)) < 2;
% dy_l = [dy_l; dy_l];
% dy_r = abs(pts_r(1:num_r, 2) - pts_r(num_r+1:end, 2)) < 2;
% dy_r = [dy_r; dy_r];

dist = dist & dx;
% dist(dy_l, :) = 0;
% dist(:, dy_r) = 0;

dist = (dist(1:num_l, 1:num_r) & dist(num_l+1:end, num_r+1:end)) | ...
    (dist(1:num_l, num_r+1:end) & dist(num_l+1:end, 1:num_r));

non_l = sum(dist, 2) ~= 1;
non_r = sum(dist, 1) ~= 1;

dist(non_l, non_r) = 0;
[row, col] = find(dist);
match = [row, col];

b = 11;

% remove lines with endpoints around the boundary

matched_l = lines_l(row, :);
matched_r = lines_r(col, :);

valid = matched_l(:, 1) > b & matched_l(:, 1) < size(im_l, 2) - b & ...
        matched_l(:, 3) > b & matched_l(:, 3) < size(im_l, 2) - b & ...
        matched_l(:, 2) > b & matched_l(:, 2) < size(im_l, 1) - b & ...
        matched_l(:, 4) > b & matched_l(:, 4) < size(im_l, 1) - b & ...
        matched_r(:, 1) > b & matched_r(:, 1) < size(im_r, 2) - b & ...
        matched_r(:, 3) > b & matched_r(:, 3) < size(im_r, 2) - b & ...
        matched_r(:, 2) > b & matched_r(:, 2) < size(im_r, 1) - b & ...
        matched_r(:, 4) > b & matched_r(:, 4) < size(im_r, 1) - b;

match = match(valid, :);
pts_l = [lines_l(match(:,1), 1:2);lines_l(match(:,1), 3:4)];
pts_r = [lines_r(match(:,2), 1:2);lines_r(match(:,2), 3:4)];
% use ssd to catch correspondence
[desc_l, ~] = extractFeatures(im_l, pts_l,...
    'Method', 'Block', 'BlockSize', b);
[desc_r, ~] = extractFeatures(im_r, pts_r,...
    'Method', 'Block', 'BlockSize', b);

n = size(match, 1);

dis1 = sum(abs(desc_l(1:n,:) - desc_r(1:n,:)), 2) + ...
       sum(abs(desc_l(n+1:end,:) - desc_r(n+1:end,:)), 2);
dis2 = sum(abs(desc_l(n+1:end,:) - desc_r(1:n,:)), 2) + ...
       sum(abs(desc_l(1:n,:) - desc_r(n+1:end,:)), 2);
dis = min(dis1, dis2);

dis_m = mean(dis);
dis_sig = sqrt(var(dis));

valid = dis < dis_m + dis_sig;
match = match(valid, :);

% filter out lines parallel to the epipolar lines

l = normr(epi_l(match(:,1),1:2));
v = normr(lines_r(match(:,2),1:2) - lines_r(match(:,2),3:4));
angle = sum(l.*v, 2);
valid = abs(angle) > 0.1;
match = match(valid, :);




end

