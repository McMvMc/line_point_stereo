function [matched, validity] = track_lines(prev_lines, prev_im, im)

LKT = vision.PointTracker('MaxBidirectionalError',1,'BlockSize', [15, 15]);
% generate points to track from lines

long = sum((prev_lines(:,1:2) - prev_lines(:,3:4)).^2, 2)>400;

long_lines = prev_lines(long, :);
short_lines = prev_lines(~long, :);

long_points = arrayfun(@(x) long_lines(:,1:2)*(1-x) + ...
                              long_lines(:,3:4)*x, (0:0.25:1)','UniformOutput',false);
long_points = cell2mat(long_points);
short_points = [short_lines(:,1:2); short_lines(:,3:4)];
sample_points = [long_points; short_points];

sample_validity = all(sample_points > 0, 2);
sample_points(~sample_validity, :) = 0.1;

initialize(LKT, sample_points, prev_im);
[tracked_points, validity, score] = step(LKT, im);
validity = validity & score > 0.95;
validity = validity & sample_validity;

long_n = sum(long);
long_tracked_points = tracked_points(1:long_n*5, :);
long_validity = validity(1:long_n*5, :);

short_n = sum(~long);
short_tracked_points = tracked_points(long_n*5+1:end, :);
short_validity = validity(long_n*5+1:end, :);

long_matched = zeros(long_n, 4);
short_matched = zeros(short_n, 4);
long_matched_validity = ones(long_n, 1);
short_matched_validity = ones(short_n, 1);

for i = 1:long_n
    pts = long_tracked_points(i:long_n:end, :);
    valid = long_validity(i:long_n:end);
    if sum(valid)<2
        long_matched(i,:) = 0;
        long_matched_validity(i) = 0;
    else
        pts = pts(valid, :);
        t = (0:0.25:1)';
        t = t(valid, :);
        px = polyfit(t, pts(:,1), 1);
        long_matched(i, 1:2:end) = polyval(px, [0, 1]);
        py = polyfit(t, pts(:,2), 1);
        long_matched(i, 2:2:end) = polyval(py, [0, 1]);
    end
end

for i = 1:short_n
    pts = short_tracked_points(i:short_n:end, :);
    valid = short_validity(i:short_n:end);
    if sum(valid)<2
        short_matched(i,:) = 0;
        short_matched_validity(i) = 0;
    else
        short_matched(i,:) = [pts(1,:), pts(2,:)];
    end
end
matched = zeros(size(prev_lines));
matched(long, :) = long_matched;
matched(~long, :) = short_matched;
validity = zeros(size(prev_lines, 1), 1);
validity(long) = long_matched_validity;
validity(~long) = short_matched_validity;
validity = logical(validity);
end

