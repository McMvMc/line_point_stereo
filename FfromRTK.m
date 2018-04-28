function F = FfromRTK(R, t, K)
    
    E = toSkew(t)*R;
    F = inv(K')*E*inv(K);

end