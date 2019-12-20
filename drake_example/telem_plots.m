%%
close all
clear all
clc

% fid = fopen('telemetry.log', 'rt');

%%
filename = 'telemetry.log';
delimiter = {',',';'};
startRow = 2;

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

% Open the text file.
fileID = fopen(filename,'r');

% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

% Close the text file.
fclose(fileID);

%% Create output variable
telemetry = [dataArray{1:end-1}];

t = telemetry(:,1); t = t-t(1);
frame = telemetry(:,2);
x = telemetry(:,3:5);
x_ref = telemetry(:,6:8);
u_ref = telemetry(:,9:10);
state = telemetry(:,11);
ellipse = telemetry(:,12);
change_control = telemetry(:,13);
action = telemetry(:,14);
u = telemetry(:,15:16);
linvel = telemetry(:,17:19);
rotvel = telemetry(:,20:22);
x_ref_new = telemetry(:,23:25);
u_ref_new = telemetry(:,26:27);


%% Jackal stuff
% fclose(fid);
path = {'R1220', 'R1216','R1212','R1108','R301','R159','R1300','R1125','R798', ...
    'R453','R449','R1311','R2','R1263','R39','R1220'};

pix2m = 0.2;
obstacle = [30.0, 60.0, 50.0,  140.0 ]*pix2m;
		
goals = [70., 70., 90.0; ...
         10., 120., -90.0]*pix2m;
                      
states = [1,10,10; ...
          1,10,14; ...
        1,10,18; ...
        1,10,22; ...
        2,8,24; ...
        2,4,24; ...
        2,3,24; ...
        3,1,22; ...
        3,1,18; ...
        3,1,14; ...
        3,1,10; ...
        3,1,9; ...
        0,3,7; ...
        0,5,7; ...
        0,8,8; ...
        1,10,10];
actions = [0, 0, 0, 5, 0, 2, 5, 0, 0, 0, 2, 5, 1, 3, 5, 0];

pix2m = 0.2; %[m]
bounds = [ 0.,  0., 20., 40.];
cell = 1.25; %[m]
W_xgrid = (bounds(1)+cell) : cell: (bounds(3)-cell);
W_ygrid = (bounds(2)+cell): cell: (bounds(4)-cell);

traj =[];
for i = 1:size(states,1)
    traj =[traj ; W_xgrid(states(i,2)+1), W_ygrid(states(i,3)+1)];
end
%%
close all

figure;

% plot(x_ref(:,2),-x_ref(:,1)); hold all
plot(x_ref_new(:,2),-x_ref_new(:,1), 'LineWidth',2); hold all
plot(x(:,2), -x(:,1), 'LineWidth',2);
plot(traj(:,2), -traj(:,1), 'kd', 'MarkerSize', 16);
for i=1:length(path)
    text(traj(i,2), -traj(i,1), path{i})
end
for i=1:size(goals,1)
    plot(goals(i,2), -goals(i,1), 's', 'MarkerSize', 16, 'Color', [0,0.5,0]);
    text(goals(i,2), -goals(i,1)-0.5, ...
        ['goal ' num2str(i)], 'Color', [0,0.5,0])
end

for i=1:size(bounds,1)
    plot([bounds(i,2) bounds(i,2)], [-bounds(i,1) -bounds(i,3)], 'k', 'LineWidth',3);
    plot([bounds(i,2) bounds(i,4)], [-bounds(i,1) -bounds(i,1)], 'k', 'LineWidth',3);
    plot([bounds(i,4) bounds(i,4)], [-bounds(i,1) -bounds(i,3)], 'k', 'LineWidth',3);
    plot([bounds(i,4) bounds(i,2)], [-bounds(i,3) -bounds(i,3)], 'k', 'LineWidth',3);
end

for i = 1:size(states,1)
    orientation = states(i,1);
    x0 = [traj(i, 1:2) (-1+orientation)*pi/2];
    index = 1;
    el_loc = [];
    pnts = [];
    while 1
        [x_centers, V] = LoadMotionPrimitives(actions(i), index, orientation);
        if(isnan(x_centers))
            break;
        end
        el_loc = [el_loc; x0 + x_centers'];
        index = index+1;
        pnt = Ellipse_plot(V(1:2,1:2), [el_loc(end,1) el_loc(end,2)]);
        %plot(el_loc(end,2),-el_loc(end,1),'.k');
        pnts = [pnts; pnt'];
    end
    k = boundary(pnts, 0.8);
    %plot(pnts(k,2), -pnts(k,1));
    %patch(pnts(k,2), -pnts(k,1), 'red');
    f1 = 1:length(k);
    v1 = [pnts(k,2), -pnts(k,1)];
    patch('Faces',f1,'Vertices',v1,'FaceColor','blue','FaceAlpha',.1);
    %plot(pnts(2,:),-pnts(1,:), 'Color', [.8 .8 .8]);
    
end

for i=1:size(W_ygrid,2)
    plot(repmat(W_ygrid(i),1,size(W_xgrid,2)), -W_xgrid, '+', 'Color', [0.85 0.85 0.85])
end

for i=1:size(obstacle,1)
    plot([obstacle(i,2) obstacle(i,2)], [-obstacle(i,1) -obstacle(i,3)], 'k', 'LineWidth',3);
    plot([obstacle(i,2) obstacle(i,4)], [-obstacle(i,1) -obstacle(i,1)], 'k', 'LineWidth',3);
    plot([obstacle(i,4) obstacle(i,4)], [-obstacle(i,1) -obstacle(i,3)], 'k', 'LineWidth',3);
    plot([obstacle(i,4) obstacle(i,2)], [-obstacle(i,3) -obstacle(i,3)], 'k', 'LineWidth',3);
    v1 = [obstacle(i,2) -obstacle(i,1); ...
         obstacle(i,4)  -obstacle(i,1); ...
         obstacle(i,4)  -obstacle(i,3); ...
         obstacle(i,2)  -obstacle(i,3)];

    f1 = [1 2 3 4];
    patch('Faces',f1,'Vertices',v1,'FaceColor', 'red', 'FaceAlpha',.3);
    text((obstacle(i,2)+obstacle(i,4))/2, -(obstacle(i,3)+obstacle(i,1))/2, ...
        ['obstacle ' num2str(i)])
    
end

axis equal
axis([bounds(2) bounds(4) -bounds(3) -bounds(1)])
title('Workspace, X-Y Plane view'); legend('ref. trajectory', 'Jackal simulation'); 
% grid on;
xlabel('Y [m]'); ylabel('X [m]');

return

figure;
h(1)=subplot(211);
%plot(t, [x_ref(:,1)-x(:,1),x_ref(:,2)-x(:,2),x_ref(:,3)-x(:,3)]); hold all
plot(t, [x_ref(:,1),x_ref(:,2),x_ref(:,3), ...
                  x_ref_new(:,1),x_ref_new(:,2),x_ref_new(:,3), ...
                  x(:,1),x(:,2),x(:,3)]); hold all
%plot(t, [, x(:,2),x(:,3)]); 
%title('pose'); legend('xref', 'yref', '\thetaref'); grid on;
title('pose'); legend('xknot', 'yknot', '\thetaknot', 'xref', 'yref', '\thetaref', 'xsim', 'ysim', '\thetasim'); grid on;
h(2)=subplot(212);
plot(t, [state, ellipse]); legend('state', 'ellipse')
linkaxes(h,'x');

figure;
plot(t, [x(:,1)-x_ref_new(:,1), ...
                  x(:,2)-x_ref_new(:,2), ...
                  x(:,3)-x_ref_new(:,3) ]); 
legend('e_x', 'e_y', 'e_\theta')

figure;
plot(t, [u_ref(:,1), u_ref(:,2)]); hold all
plot(t, [u_ref_new(:,1), u_ref_new(:,2)]); 
plot(t, [u(:,1), u(:,2), u(:,2)/(0.42*1).*tan(u(:,1))]); hold all
plot(t, [linvel(:,1) rotvel(:,3)]);
legend('\delta_k_n_o_t','u_k_n_o_t', '\delta_r_e_f','u_r_e_f', ...
    '\delta_s_i_m','u_s_i_m', '\omega_s_i_m', 'u_m_e_a_s','\omega_m_e_a_s');


