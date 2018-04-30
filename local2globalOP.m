function [O_l_g, P_l_g] = local2globalOP(prev_O_l_g, prev_P_l_g, R_l_g, t_r_g)
    O_l_g = R_l_g' * prev_O_l_g;
    P_l_g = -R_l_g'*t_r_g + prev_P_l_g;
end