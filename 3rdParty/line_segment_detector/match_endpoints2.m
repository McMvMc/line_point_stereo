function [match] = match_endpoints2(lines, r)
%MATCH_ENDPOINTS2 Summary of this function goes here
%   Detailed explanation goes here

[k1, d1] = dsearchn(lines(:,1:2), lines(:,3:4));
[k2, d2] = dsearchn(lines(:,3:4), lines(:,3:4));
[k3, d3] = dsearchn(lines(:,1:2), lines(:,1:2));
[k4, d4] = dsearchn(lines(:,3:4), lines(:,1:2));
k1(d1 > r) = 0;
k2(d2 > r) = 0;
k3(d3 > r) = 0;
k4(d4 > r) = 0;
match = [k1, k2, k3, k4];

end

