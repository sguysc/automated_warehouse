%%
close all

for i = [0,1, 3:8, 2] 
    orientation = 1;
    x0 = [0 0 (-1+orientation)*pi/2];
    index = 1;
    el_loc = [];
    pnts = [];
    while 1
        [x_centers, V] = LoadMotionPrimitives(mp_data, i, index, orientation);
        if(isnan(x_centers))
            break;
        end
        el_loc = [el_loc; x0 + x_centers'];
        index = index+1;
        pnt = Ellipse_plot(V(1:2,1:2), [el_loc(end,1) el_loc(end,2)]);
        %plot(el_loc(end,2),-el_loc(end,1),'.k');
        pnts = [pnts; pnt'];
    end
    k = boundary(pnts, 0.9); %.8
    %plot(pnts(k,2), -pnts(k,1));
    %patch(pnts(k,2), -pnts(k,1), 'red');
    f1 = 1:length(k);
    v1 = [pnts(k,2), pnts(k,1)];
    patch('Faces',f1,'Vertices',v1,'FaceColor','blue','FaceAlpha',.1);
    %plot(pnts(2,:),-pnts(1,:), 'Color', [.8 .8 .8]);
    
end


axis equal
%axis([bounds(2) bounds(4) -bounds(3) -bounds(1)])
axis([-1 2 -2 2])
title('Motion primitive library'); 

% grid on;
xlabel('Y [m]'); ylabel('X [m]');
