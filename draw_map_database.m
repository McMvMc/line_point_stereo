function draw_map_database(worldPoints, O_l_g, P_l_g, O_r_g, P_r_g)
    scaleup = 1;
    cam_size = 0.1;
    
    figure(4), hold on;
    plot3(worldPoints(:,1)*scaleup,worldPoints(:,2)*scaleup,...
                                worldPoints(:,3)*scaleup,'.');
    cam_l = plotCamera('Location',P_l_g,'Orientation',O_l_g,'Opacity',0,...
                       'Size', cam_size);
    cam_r = plotCamera('Location',P_r_g,'Orientation',O_r_g,'Opacity',0,...
                       'Color',[0 1 0], 'Size', cam_size);
    grid on
    drawnow();
end