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

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to the format.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');

%% Close the text file.
fclose(fileID);

%% Post processing for unimportable data.
% No unimportable data rules were applied during the import, so no post
% processing code is included. To generate code which works for
% unimportable data, select unimportable cells in a file and regenerate the
% script.

%% Create output variable
telemetry = [dataArray{1:end-1}];

t = telemetry(:,1);
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
% u_tmp=[];
% while ~feof(fid)
%     line_ex = fgetl(fid); 
%     
%     %disp(line_ex)
%     if(~isempty(strfind(line_ex, 'changed')))
%         % it's a change in control msg
%         sss=textscan(line_ex, '%[^)] %[^[] [%f,%f,%f]; u=[%f,%f]; xref=[%f,%f,%f]; uref=[%f,%f]');
%         u_tmp = [sss{6}, sss{7}];
%     end
%     if(~isempty(strfind(line_ex, 'F=')))
%         sss=textscan(line_ex, '%[^F] F=%d; x=(%f, %f, %f); xr=(%f, %f, %f); ur=(%f, %f); s=%d; e=%d; dc=%d');
%         x = [x; [sss{3}, sss{4}, sss{5}]];
%         u = [u; u_tmp];
%         x_ref = [x_ref; [sss{6}, sss{7}, sss{8}]];
%         u_ref = [u_ref; [sss{9}, sss{10}]];
%         frame = [frame; sss{2}];
%         state = [state; sss{11}];
%         ellipse = [ellipse; sss{12}];
%         change_control = [change_control; sss{13}];
%     end
% end
% sss=textscan(fid, '%[^F] F=%d; x=(%f, %f, %f); xr=(%f, %f, %f); ur=(%f, %f); s=%d; e=%d; dc=%d; a=%d; u=(%f, %f)');
% x = [sss{3}, sss{4}, sss{5}];
% u = [sss{15}, sss{16}];
% x_ref = [sss{6}, sss{7}, sss{8}];
% u_ref = [sss{9}, sss{10}];
% frame = double(sss{2});
% state = sss{11};
% ellipse = sss{12};
% change_control = sss{13};
% action = sss{14};

%%
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
plot(x_ref_new(:,2),-x_ref_new(:,1)); hold all
plot(x(:,2), -x(:,1));
plot(traj(:,2), -traj(:,1), 'kd', 'MarkerSize', 16);
for i=1:length(path)
    text(traj(i,2), -traj(i,1), path{i})
end
for i=1:size(goals,1)
    plot(goals(i,2), -goals(i,1), 'gs', 'MarkerSize', 16);
    text(goals(i,2), -goals(i,1)-0.5, ...
        ['goal ' num2str(i)], 'Color','g')
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


for i=1:size(bounds,1)
    plot([bounds(i,2) bounds(i,2)], [-bounds(i,1) -bounds(i,3)], 'k', 'LineWidth',3);
    plot([bounds(i,2) bounds(i,4)], [-bounds(i,1) -bounds(i,1)], 'k', 'LineWidth',3);
    plot([bounds(i,4) bounds(i,4)], [-bounds(i,1) -bounds(i,3)], 'k', 'LineWidth',3);
    plot([bounds(i,4) bounds(i,2)], [-bounds(i,3) -bounds(i,3)], 'k', 'LineWidth',3);
end

axis equal

axis([bounds(2) bounds(4) -bounds(3) -bounds(1)])
title('Workspace, X-Y Plane view'); legend('ref. trajectory', 'simulation'); grid on;
xlabel('Y [m]'); ylabel('X [m]');

figure;
h(1)=subplot(211);
%plot(frame/1000, [x_ref(:,1)-x(:,1),x_ref(:,2)-x(:,2),x_ref(:,3)-x(:,3)]); hold all
plot(frame/1000, [x_ref(:,1),x_ref(:,2),x_ref(:,3), ...
                  x_ref_new(:,1),x_ref_new(:,2),x_ref_new(:,3), ...
                  x(:,1),x(:,2),x(:,3)]); hold all
%plot(frame/1000, [, x(:,2),x(:,3)]); 
%title('pose'); legend('xref', 'yref', '\thetaref'); grid on;
title('pose'); legend('xknot', 'yknot', '\thetaknot', 'xref', 'yref', '\thetaref', 'xsim', 'ysim', '\thetasim'); grid on;
h(2)=subplot(212);
plot(frame/1000, [state, ellipse]); legend('state', 'ellipse')
linkaxes(h,'x');

figure;
plot(frame/1000, [x(:,1)-x_ref_new(:,1), ...
                  x(:,2)-x_ref_new(:,2), ...
                  x(:,3)-x_ref_new(:,3) ]); 
legend('e_x', 'e_y', 'e_\theta')

figure;
plot(frame/1000, [u_ref(:,1), u_ref(:,2)]); hold all
plot(frame/1000, [u_ref_new(:,1), u_ref_new(:,2)]); 
plot(frame/1000, [u(:,1), u(:,2), u(:,2)/(0.42*57).*tan(u(:,1))]); hold all
legend('\delta_k_n_o_t','u_k_n_o_t', '\delta_r_e_f','u_r_e_f', '\delta_s_i_m','u_s_i_m', '\omega_s_i_m');


