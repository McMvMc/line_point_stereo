function [pts_w_l] = generate_3d_lines(lines_l, lines_r, stereoParams, F)

pts_l = reshape(lines_l', 2, [])';
epi_l = epipolarLine(F, pts_l);

pts_r = reshape(lines_r', 2, []);
pts_h_r = [pts_r; ones(1, size(pts_r, 2))];

inf_lines_r = cross(pts_h_r(:, 1:2:end), pts_h_r(:, 2:2:end));

inf_lines_r = reshape([inf_lines_r; inf_lines_r], 3, []);

pts_r_ext = cross(inf_lines_r, epi_l')';
pts_r_ext = pts_r_ext(:,1:2)./pts_r_ext(:,3);

pts_w_l = triangulate(pts_l, pts_r_ext, stereoParams);

pts_w_l = reshape(pts_w_l', 6, [])';
pts_w_l = [pts_w_l(:, 1:3); pts_w_l(:, 4:6)];

end

