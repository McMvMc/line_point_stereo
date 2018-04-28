function [desc, valid_pts] = detect_points(image)

    pts = detectMinEigenFeatures(image);
%     pts = detectHarrisFeatures(image);
    [desc,valid_pts] = extractFeatures(image, pts ,'Method','BRISK');

end