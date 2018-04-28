% cam -> world
% world -> cam
% cv convention. Need to transpose if using Matlab convention
function [R_b, t_b] = switch_coord_sys(R_a, t_a)
    R_b = R_a';
    t_b = -R_a'*t_a;
end