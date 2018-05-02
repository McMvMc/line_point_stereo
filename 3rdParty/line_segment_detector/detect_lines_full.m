function [lines] = detect_lines_full(im)

    scale = 0.5;
    im_down = imresize(im, scale);
    [~, pts] = detect_points(im_down);
    pts = pts.Location/scale;
    lines = detect_lines(im, pts);

end

