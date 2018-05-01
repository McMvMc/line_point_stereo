function [match] = match_endpoints(pts, lines, r)
% match line endpoints to corners 

lines = [lines(:,1:2); lines(:,3:4)];

[k, d] = dsearchn(pts, lines);
k(d > r) = 0;
match = reshape(k, [], 2);

end

