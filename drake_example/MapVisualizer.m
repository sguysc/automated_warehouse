%%
%close all
clear all
clc

% fid = fopen('telemetry.log', 'rt');

%%
% filename = 'telemetry.log';
% cd('../telemetry')
[file,path] = uigetfile('../telemetry/*.log');
% cd('..');
if ~isequal(file,0)
   filename = fullfile(path,file);
else
    return
end
map_name = 'lab';
%map_name = 'raymond';

map_text = fileread([map_name '.map']);
map_data = jsondecode(map_text);
mp_text = fileread([map_name '.motion']);
mp_data = jsondecode(mp_text);
states_text = fopen([path strtok(file,'.') '.state'], 'r');
delimiter = {',',';'};
startRow = 1;
formatSpec = '%d%s%d%[^\n\r]';
states_data = textscan(states_text, formatSpec, 'Delimiter', delimiter, 'TextType', 'string', 'EmptyValue', NaN, 'HeaderLines' ,startRow-1, 'ReturnOnError', false, 'EndOfLine', '\r\n');
fclose(states_text);

delimiter = {',',';'};
startRow = 2;

formatSpec = '%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%[^\n\r]';

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
t_sensetime = telemetry(:,28);

%% Jackal stuff
% fclose(fid);

%pix2m = 0.2;
%obstacle = [30.0, 60.0, 50.0,  140.0 ]*pix2m;
num_robots = map_data.robots;

obstacle = [];
for i =1:length(map_data.obstacles)
    obstacle = [obstacle; map_data.obstacles(i).x, map_data.obstacles(i).y, map_data.obstacles(i).X,  map_data.obstacles(i).Y  ];
end	
%goals = [70., 70., 90.0; ...
%         10., 120., -90.0]*pix2m;
% need to add dimension here for several robots
goals = {};
for r = 1:num_robots
    goals_robot_i = map_data.(['r' num2str(r-1)]);
    goals{r} = [];
    for i = 1:length(goals_robot_i)
        goals{r} = [goals{r}; goals_robot_i(i).x, goals_robot_i(i).y, goals_robot_i(i).teta ] ;
    end	
end

no_enter = [];
for i =1:length(map_data.no_enter)
    no_enter = [no_enter; map_data.no_enter(i).x, map_data.no_enter(i).y, map_data.no_enter(i).X,  map_data.no_enter(i).Y  ];
end	

one_ways_N = []; one_ways_S = []; one_ways_E = []; one_ways_W = [];
for i =1:length(map_data.one_ways)
    if(map_data.one_ways(i).D == 2)
        one_ways_N = [one_ways_N; map_data.one_ways(i).x, map_data.one_ways(i).y, map_data.one_ways(i).X,  map_data.one_ways(i).Y  ];
    elseif (map_data.one_ways(i).D == 1)
        one_ways_E = [one_ways_E; map_data.one_ways(i).x, map_data.one_ways(i).y, map_data.one_ways(i).X,  map_data.one_ways(i).Y  ];
    elseif (map_data.one_ways(i).D == 0)
        one_ways_S = [one_ways_S; map_data.one_ways(i).x, map_data.one_ways(i).y, map_data.one_ways(i).X,  map_data.one_ways(i).Y  ];
    elseif (map_data.one_ways(i).D == 3)
        one_ways_W = [one_ways_W; map_data.one_ways(i).x, map_data.one_ways(i).y, map_data.one_ways(i).X,  map_data.one_ways(i).Y  ];
    end
end	
% lab
%path = {'H1X12Y6', 'H3X15Y26', 'H3X1Y2'};
path = states_data{2};
%path = {'H1X14Y7', 'H2X11Y9', 'H2X7Y10', 'H1X5Y13', 'H1X4Y17', 'H1X4Y21', 'H1X4Y25'};
%states = [1,12,6; ...
%          3,15,26;...
%          3,1,2];
%actions = [2, 5, 4, 6, 3, 0, 0];
actions = states_data{3};
states = zeros(length(path), 3);
for i = 1:length(path)
    tmp = sscanf(path{i}, 'H%dX%dY%d');
    states(i,:) = tmp';
end

%pix2m = 0.2; %[m]
bounds = [ map_data.workspace.x,  map_data.workspace.y, map_data.workspace.X, map_data.workspace.Y];
cell   = map_data.cell; % 1.25; %[m]
W_xgrid = (bounds(1)+cell/2) : cell : (bounds(3)-cell/2);
W_ygrid = (bounds(2)+cell/2) : cell : (bounds(4)-cell/2);

traj =[];
for i = 1:size(states,1)
    traj =[traj ; W_xgrid(states(i,2)+1), W_ygrid(states(i,3)+1)];
end
%%
%close all

figure;

vv=1:size(x_ref_new,1);
%vv=1:19980;
%vv=1:size(x,1);
%iii = find(states(:,1)==1 & states(:,2)==10 & states(:,3)==5 );
iii = size(states,1);
%states_plot = 7:28;%[38:42,78:80];
states_plot =1:iii;

% plot(x_ref(:,2),-x_ref(:,1)); hold all
plot(x_ref_new(vv,2),x_ref_new(vv,1), 'k:', 'LineWidth',2); hold all
ax = gca; ax.YDir = 'reverse';
plot(x(vv,2), x(vv,1),  'k-', 'LineWidth',2);

plot(traj(states_plot,2), traj(states_plot,1), 'kd', 'MarkerSize', 16);
for i=states_plot
    %text(traj(i,2), traj(i,1), path{i})
end
% GUY TODO, inflate bounds just because I'm not considering that
% to be out-of-bounds because lab is too small
%bounds = [bounds(1)-.7 bounds(2)-.7 bounds(3)+.7 bounds(4)+.7 ];
for i=1:size(bounds,1)
    plot([bounds(i,2) bounds(i,2)], -[-bounds(i,1) -bounds(i,3)], 'k', 'LineWidth',3);
    plot([bounds(i,2) bounds(i,4)], -[-bounds(i,1) -bounds(i,1)], 'k', 'LineWidth',3);
    plot([bounds(i,4) bounds(i,4)], -[-bounds(i,1) -bounds(i,3)], 'k', 'LineWidth',3);
    plot([bounds(i,4) bounds(i,2)], -[-bounds(i,3) -bounds(i,3)], 'k', 'LineWidth',3);
end

if 0
    prsn = imread('/home/cornell/Documents/writing_papers/automated_warehouse/person_topview.png');
    prsn = imrotate(prsn, 90);
    imagesc(-0.2+[-.3 .3], -1.5+[-.2 .2], prsn);
end

for i = states_plot 
    orientation = states(i,1);
    x0 = [traj(i, 1:2) (-1+orientation)*pi/2];
    index = 1;
    el_loc = [];
    pnts = [];
    while 1
        [x_centers, V] = LoadMotionPrimitives(mp_data, actions(i), index, orientation);
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

for i=1:size(W_ygrid,2)
    plot(repmat(W_ygrid(i),1,size(W_xgrid,2)), W_xgrid, '+', 'Color', [0.85 0.85 0.85])
end

for i=1:size(obstacle,1)
    plot([obstacle(i,2) obstacle(i,2)], -[-obstacle(i,1) -obstacle(i,3)], 'k', 'LineWidth',3);
    plot([obstacle(i,2) obstacle(i,4)], -[-obstacle(i,1) -obstacle(i,1)], 'k', 'LineWidth',3);
    plot([obstacle(i,4) obstacle(i,4)], -[-obstacle(i,1) -obstacle(i,3)], 'k', 'LineWidth',3);
    plot([obstacle(i,4) obstacle(i,2)], -[-obstacle(i,3) -obstacle(i,3)], 'k', 'LineWidth',3);
    v1 = [obstacle(i,2) obstacle(i,1); ...
         obstacle(i,4)  obstacle(i,1); ...
         obstacle(i,4)  obstacle(i,3); ...
         obstacle(i,2)  obstacle(i,3)];

    f1 = [1 2 3 4];
    patch('Faces',f1,'Vertices',v1,'FaceColor', 'red', 'FaceAlpha',.3);
    text(obstacle(i,2)+(obstacle(i,4)-obstacle(i,2))/6, (obstacle(i,3)+obstacle(i,1))/2, ...
        ['$O_{' num2str(i) '}$'], 'Interpreter', 'latex')
    
end

for i=1:size(no_enter,1)
    plot([no_enter(i,2) no_enter(i,2)], -[-no_enter(i,1) -no_enter(i,3)], 'k', 'LineWidth',3);
    plot([no_enter(i,2) no_enter(i,4)], -[-no_enter(i,1) -no_enter(i,1)], 'k', 'LineWidth',3);
    plot([no_enter(i,4) no_enter(i,4)], -[-no_enter(i,1) -no_enter(i,3)], 'k', 'LineWidth',3);
    plot([no_enter(i,4) no_enter(i,2)], -[-no_enter(i,3) -no_enter(i,3)], 'k', 'LineWidth',3);
    v1 = [no_enter(i,2) no_enter(i,1); ...
         no_enter(i,4)  no_enter(i,1); ...
         no_enter(i,4)  no_enter(i,3); ...
         no_enter(i,2)  no_enter(i,3)];

    f1 = [1 2 3 4];
    patch('Faces',f1,'Vertices',v1,'FaceColor', 'magenta', 'FaceAlpha',.3);
    text(no_enter(i,2)+(no_enter(i,4)-no_enter(i,2))/6, (no_enter(i,3)+no_enter(i,1))/2, ...
        ['$NEZ_{' num2str(i) '}$'], 'Interpreter', 'latex')
    
end
num_robots = 1;
for r = 1:num_robots
    for i=1:size(goals{r},1)
        plot(goals{r}(i,2), goals{r}(i,1), 's', 'MarkerSize', 16, 'Color', [0,0.5,0]);
        text(goals{r}(i,2)+0.1, goals{r}(i,1)+0.1, ...
            ['goal' num2str(i)], 'Color', [0,0.5,0],'FontWeight','bold')
    end
end
%[0.7, 0.7, 0.9]
for i=1:size(one_ways_N,1)
    p1 = [(one_ways_N(i,2)+one_ways_N(i,4))/2 one_ways_N(i,3)];                         % First Point
    p2 = [(one_ways_N(i,2)+one_ways_N(i,4))/2 one_ways_N(i,1)];                         % Second Point
    dp = p2-p1;                         % Difference
    quiver(p1(1),p1(2),dp(1),1.0*dp(2),0.0, 'Color', 'r', 'LineWidth',2)
end
for i=1:size(one_ways_S,1)
    p1 = [(one_ways_S(i,2)+one_ways_S(i,4))/2 one_ways_S(i,1)];                         % First Point
    p2 = [(one_ways_S(i,2)+one_ways_S(i,4))/2 one_ways_S(i,3)];                         % Second Point
    dp = p2-p1;                         % Difference
    quiver(p1(1),p1(2),dp(1),1.00*dp(2),0.0, 'Color', 'r', 'LineWidth',2)
end
for i=1:size(one_ways_E,1)
    p1 = [one_ways_E(i,2) (one_ways_E(i,1)+one_ways_E(i,3))/2];                         % First Point
    p2 = [one_ways_E(i,4) (one_ways_E(i,1)+one_ways_E(i,3))/2];                         % Second Point
    dp = p2-p1;                         % Difference
    quiver(p1(1),p1(2),1.00*dp(1),dp(2),0.0, 'Color', 'r', 'LineWidth',2)
end
for i=1:size(one_ways_W,1)
    p1 = [one_ways_W(i,4) (one_ways_W(i,1)+one_ways_W(i,3))/2];                         % First Point
    p2 = [one_ways_W(i,2) (one_ways_W(i,1)+one_ways_W(i,3))/2];                         % Second Point
    dp = p2-p1;                         % Difference
    quiver(p1(1),p1(2),1.00*dp(1),dp(2),0.0, 'Color', 'r', 'LineWidth',2)
end

axis equal
%axis([bounds(2) bounds(4) -bounds(3) -bounds(1)])
axis([bounds(2)-0.2 bounds(4)+0.2 bounds(1)-0.2 bounds(3)+0.2])
title('Example2 workspace, top view'); 

% grid on;
xlabel('y [m]'); ylabel('x [m]');

%%
if 0
    problematic_funnel = 22;
    action_not_chosen = 5;

    i = problematic_funnel ;
    orientation = states(i,1);
    x0 = [traj(i, 1:2) (-1+orientation)*pi/2];
    index = 1;
    el_loc = [];
    pnts = [];
    while 1
        [x_centers, V] = LoadMotionPrimitives(mp_data, action_not_chosen, index, orientation);
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
    f1 = 1:length(k);
    v1 = [pnts(k,2), pnts(k,1)];
    patch('Faces',f1,'Vertices',v1,'FaceColor','red','FaceAlpha',.3);
end

jckl = imread('/home/cornell/Documents/writing_papers/automated_warehouse/jackal_topview.png');
jckl = imrotate(jckl, 90);
imagesc([x(vv(end),2)-.2 x(vv(end),2)+.2], [x(vv(end),1)-.2 x(vv(end),1)+.2], jckl);
if 0
    prsn = imread('/home/cornell/Documents/writing_papers/automated_warehouse/person_topview.png');
    prsn = imrotate(prsn, 180);
    imagesc(3.+[-.2 .2], -1.5+[-.3 .3], prsn);
    quiver(.2,-1.5,2.5,0.0,0.0, 'Color', [0.5, 0.5, 0.5], 'LineWidth',2)
end

legend('Ref. trajectory', 'Robot'); 