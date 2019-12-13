%%
close all
clear all
clc

fid = fopen('telemetry.log', 'rt');

%%
frame = [];
x = [];
x_ref = [];
u = [];
u_ref = [];
state = [];
ellipse = [];
change_control = [];
action = [];
u_tmp=[];
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
sss=textscan(fid, '%[^F] F=%d; x=(%f, %f, %f); xr=(%f, %f, %f); ur=(%f, %f); s=%d; e=%d; dc=%d; a=%d; u=(%f, %f)');
x = [sss{3}, sss{4}, sss{5}];
u = [sss{15}, sss{16}];
x_ref = [sss{6}, sss{7}, sss{8}];
u_ref = [sss{9}, sss{10}];
frame = double(sss{2});
state = sss{11};
ellipse = sss{12};
change_control = sss{13};
action = sss{14};
%%
fclose(fid);

%%
close all

figure;

plot(x_ref(:,2),-x_ref(:,1)); hold all
plot(x(:,2), -x(:,1)); 
axis equal
title('pose'); legend('ref','sim'); grid on;

figure;
subplot(211)
%plot(frame/1000, [x_ref(:,1)-x(:,1),x_ref(:,2)-x(:,2),x_ref(:,3)-x(:,3)]); hold all
plot(frame/1000, [x_ref(:,1),x_ref(:,2),x_ref(:,3),x(:,1),x(:,2),x(:,3)]); hold all
%plot(frame/1000, [, x(:,2),x(:,3)]); 
%title('pose'); legend('xref', 'yref', '\thetaref'); grid on;
title('pose'); legend('xref', 'yref', '\thetaref', 'xsim', 'ysim', '\thetasim'); grid on;
subplot(212)
plot(frame/1000, [state, ellipse]); legend('state', 'ellipse')

figure;
plot(frame/1000, [u_ref(:,1), u_ref(:,2)]); hold all
plot(frame/1000, [u(:,1), u(:,2)]); hold all
legend('\delta_r_e_f','u_r_e_f', '\delta_s_i_m','u_s_i_m');


