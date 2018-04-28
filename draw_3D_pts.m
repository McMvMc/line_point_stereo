function draw_3D_pts(worldPoints, O_l_g, P_l_g, O_r_g, P_r_g)
    scaleup = 10;

    figure(3), hold on;
    plot3(worldPoints(:,1)*scaleup,worldPoints(:,2)*scaleup,...
                                worldPoints(:,3)*scaleup,'*');
    cam_l = plotCamera('Location',P_l_g,'Orientation',O_l_g,'Opacity',0);
    cam_r = plotCamera('Location',P_r_g,'Orientation',O_r_g,'Opacity',0,...
                                'Color',[0 1 0]);
    grid on
    drawnow();
end