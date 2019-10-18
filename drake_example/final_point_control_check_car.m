function final_point_control_check()
close all
times=0:0.01:10;

% xf =[1.38669205, 0.61862571, 0.6697654 ]'; 
xf = [3.1, 1.9, 0.1]';
T = eye(3); 
% T = [-0.08434561,  0.00531625;
%       0.25015733,  0.20463393];
xf = T*xf;
uf = [-0.77599333,  1.15555556]';% [-1.3962634 ,  1.15555556]';
R = 0.01;
x0 = xf+R*[2*(rand(size(xf))-.5);];
figure;
% A = [0, 1; -10*cos(xf(1)) -1]; B = [0;1];
% [K,~,~]= lqr(A,B,10*eye(2),1*eye(1));
K = [ 0.55988621,  2.38464409,  2.76500913; ...
        2.38464409, -0.55988621, -0.8456573 ];
options = odeset('RelTol',1e-5);%,'Stats','on','OutputFcn',@odeplot);
[T, x] = ode45(@(t,x)car_dynamics(t, x, K, uf, xf), ...
                    times, x0, options) ;

plot(T, x(end,1)-x(:,1),'b-', T, x(end,2)-x(:,2), 'g-', T, x(end,3)-x(:,3), 'r-'); hold all;
% plot([T(1) T(end)], [xf(1) xf(1)], 'b');
% plot([T(1) T(end)], [xf(2) xf(2)], 'g');
% plot([T(1) T(end)], [xf(3) xf(3)], 'r');

[T, x] = ode45(@(t,x)car_approx_dynamics(t, x), ...
                    times, xf-x0, options) ;

plot(T, x(:,1),'b:', T, x(:,2), 'g:', T, x(:,3), 'r:'); hold all;
% V = (((-3.1000000000000001 + x(:,1)) .* (1.4837669049548923 .* (-3.1000000000000001 + x(:,1)) - 0.54459785166515851 .* (-1.8999999999999999 + x(:,2)) + 0.0016067430675332931 .* (-0.10000000000000003 + x(:,3)))) + ((-1.8999999999999999 + x(:,2)) .* ( - 0.54459785166515862 .* (-3.1000000000000001 + x(:,1)) + 6.9054387378206794 .* (-1.8999999999999999 + x(:,2)) + 0.036868266945400185 .* (-0.10000000000000003 + x(:,3)))) + ((-0.10000000000000003 + x(:,3)) .* (0.0016067430675332939 .* (-3.1000000000000001 + x(:,1)) + 0.036868266945400185 .* (-1.8999999999999999 + x(:,2)) + 0.036527396478655862 .* (-0.10000000000000003 + x(:,3)))));
% plot(T, V);
zoom on
figure;
[x, y] = meshgrid(-2:0.1:2, -2:0.1:2);
z = xf(3)*zeros(size(x));
V = ((x .* (1.2710439216280629 .* x + 0.041416275308525152 .* y - 0.0062330193291722513 .* z)) + (y .* (0.041416275308525152 .* x + 0.93737513363479996 .* y + 0.13005899823762895 .* z)) + (z .* ( - 0.0062330193291722027 .* x + 0.13005899823762895 .* y + 0.14413063778897017 .* z)));
Vdot = ((( - 4.123991426359594 .* x - 23.646710430174462 .* y - 27.530004103469132 .* z + ((-842.38518808143465 .* (z.^3)) ./ 6) + 2 .* ((-724.55952735254027 .* y .* (z.^2)) ./ 6) + ((-724.55952735254016 .* y .* (z.^2)) ./ 6) + ((-623.20982967158932 .* (y.^2) .* z) ./ 6) + 2 .* ((-623.20982967158909 .* (y.^2) .* z) ./ 6) + ((-536.03281201954962 .* (y.^3)) ./ 6) + 2 .* ((-141.25061610207391 .* x .* (z.^2)) ./ 6) + ((-141.25061610207388 .* x .* (z.^2)) ./ 6) + 2 .* ((-121.42601259684383 .* x .* y .* z) ./ 6) + ((-121.42601259684379 .* x .* y .* z) ./ 6) + 3 .* ((-121.42601259684378 .* x .* y .* z) ./ 6) + 3 .* ((-104.38270636332155 .* x .* (y.^2)) ./ 6) + ((-91.078972005052165 .* (z.^2)) ./ 2) + 2 .* ((-77.930402411031281 .* y .* z) ./ 2) + ((-66.675806993664608 .* (y.^2)) ./ 2) + 2 .* ((-22.664038859713386 .* (x.^2) .* z) ./ 6) + ((-22.664038859713383 .* (x.^2) .* z) ./ 6) + 2 .* ((-19.466580820491501 .* (x.^2) .* y) ./ 6) + ((-19.466580820491497 .* (x.^2) .* y) ./ 6) + 2 .* ((-9.0992598875724564 .* x .* z) ./ 2) + ((-7.7221084999052971 .* x .* y) ./ 2) + ((-7.7221084999052962 .* x .* y) ./ 2) + ((-3.3868985363560489 .* (x.^3)) ./ 6) + ((0.049416654099535551 .* (x.^2)) ./ 2)) .* ( - 0.012466038658344454 .* x + 0.26011799647525791 .* y + 0.28826127557794035 .* z)) + (( - 2.3727308015319104 .* x + 0.55708911045314402 .* y + 0.10270492023415845 .* z + ((-7.531485227842035 .* (z.^2)) ./ 2) + ((-1.7855700002399373 .* (z.^3)) ./ 6) + 3 .* ((-0.55708911045314402 .* y .* (z.^2)) ./ 6) + 2 .* ((-0.055895353219691496 .* y .* z) ./ 2) + 2 .* ((0.23806716691875224 .* x .* z) ./ 2) + 3 .* ((2.3727308015319104 .* x .* (z.^2)) ./ 6)) .* (2.5420878432561258 .* x + 0.082832550617050305 .* y - 0.012466038658344454 .* z)) + (( - 0.23806716691875224 .* x + 0.055895353219691496 .* y + 7.4470603698588169 .* z + ((-7.6159100858252522 .* (z.^3)) ./ 6) + 2 .* ((-2.3727308015319104 .* x .* z) ./ 2) + 3 .* ((-0.055895353219691496 .* y .* (z.^2)) ./ 6) + 3 .* ((0.23806716691875224 .* x .* (z.^2)) ./ 6) + 2 .* ((0.55708911045314402 .* y .* z) ./ 2) + ((0.94413746023704781 .* (z.^2)) ./ 2)) .* (0.082832550617050305 .* x + 1.8747502672695999 .* y + 0.26011799647525791 .* z)));

surf(x,y,V, 'FaceAlpha',0.2); hold all;
surf(x,y,Vdot, 'FaceAlpha',0.2);

V = (0.65689224047432226 + 0.17799610627774348 .* x + 0.088632059399032931 .* y + 0.0023501249102540313 .* z + 0.20605743741392482 .* (x .* y) + 0.0047574968361556664 .* (x .* z) + 0.04010694720495888 .* (y .* z) + 0.7933959606226243 .* (x.^2) + 0.67379111463147245 .* (y.^2) + 0.039054713714198837 .* (z.^2));
Vdot = ((( - 4.123991426359594 .* x - 23.646710430174462 .* y - 27.530004103469132 .* z + ((-842.38518808143465 .* (z.^3)) ./ 6) + 2 .* ((-724.55952735254027 .* y .* (z.^2)) ./ 6) + ((-724.55952735254016 .* y .* (z.^2)) ./ 6) + ((-623.20982967158932 .* (y.^2) .* z) ./ 6) + 2 .* ((-623.20982967158909 .* (y.^2) .* z) ./ 6) + ((-536.03281201954962 .* (y.^3)) ./ 6) + 2 .* ((-141.25061610207391 .* x .* (z.^2)) ./ 6) + ((-141.25061610207388 .* x .* (z.^2)) ./ 6) + 2 .* ((-121.42601259684383 .* x .* y .* z) ./ 6) + ((-121.42601259684379 .* x .* y .* z) ./ 6) + 3 .* ((-121.42601259684378 .* x .* y .* z) ./ 6) + 3 .* ((-104.38270636332155 .* x .* (y.^2)) ./ 6) + ((-91.078972005052165 .* (z.^2)) ./ 2) + 2 .* ((-77.930402411031281 .* y .* z) ./ 2) + ((-66.675806993664608 .* (y.^2)) ./ 2) + 2 .* ((-22.664038859713386 .* (x.^2) .* z) ./ 6) + ((-22.664038859713383 .* (x.^2) .* z) ./ 6) + 2 .* ((-19.466580820491501 .* (x.^2) .* y) ./ 6) + ((-19.466580820491497 .* (x.^2) .* y) ./ 6) + 2 .* ((-9.0992598875724564 .* x .* z) ./ 2) + ((-7.7221084999052971 .* x .* y) ./ 2) + ((-7.7221084999052962 .* x .* y) ./ 2) + ((-3.3868985363560489 .* (x.^3)) ./ 6) + ((0.049416654099535551 .* (x.^2)) ./ 2)) .* (0.0023501249102540313 + 0.0047574968361556664 .* x + 0.04010694720495888 .* y + 0.078109427428397674 .* z)) + (( - 2.3727308015319104 .* x + 0.55708911045314402 .* y + 0.10270492023415845 .* z + ((-7.531485227842035 .* (z.^2)) ./ 2) + ((-1.7855700002399373 .* (z.^3)) ./ 6) + 3 .* ((-0.55708911045314402 .* y .* (z.^2)) ./ 6) + 2 .* ((-0.055895353219691496 .* y .* z) ./ 2) + 2 .* ((0.23806716691875224 .* x .* z) ./ 2) + 3 .* ((2.3727308015319104 .* x .* (z.^2)) ./ 6)) .* (0.17799610627774348 + 1.5867919212452486 .* x + 0.20605743741392482 .* y + 0.0047574968361556664 .* z)) + (( - 0.23806716691875224 .* x + 0.055895353219691496 .* y + 7.4470603698588169 .* z + ((-7.6159100858252522 .* (z.^3)) ./ 6) + 2 .* ((-2.3727308015319104 .* x .* z) ./ 2) + 3 .* ((-0.055895353219691496 .* y .* (z.^2)) ./ 6) + 3 .* ((0.23806716691875224 .* x .* (z.^2)) ./ 6) + 2 .* ((0.55708911045314402 .* y .* z) ./ 2) + ((0.94413746023704781 .* (z.^2)) ./ 2)) .* (0.088632059399032931 + 0.20605743741392482 .* x + 1.3475822292629449 .* y + 0.04010694720495888 .* z)));
surf(x,y,V, 'FaceAlpha',0.9); hold all;
surf(x,y,Vdot, 'FaceAlpha',0.9);


axis([-2 2 -2 2 -1 1*2])

end

function xdot = car_dynamics(t,x, K, u, xd)
    xdot = 0*x;
    L = 1.0;
%     xdot(1) = (-2.2204460492503131e-16 - 2.8115338151280138 * (-3.1000000000000001 + x(1)) + 0.12467524917432853 * (-1.8999999999999999 + x(2)) + 0.29062218214616281 * (-0.10000000000000003 + x(3)) + ((-1.2312513831192315 * ((-0.10000000000000003 + x(3))^2)) / 2) + ((-1.1025926649111579 * ((-0.10000000000000003 + x(3))^3)) / 6) + 3 * ((-0.12467524917432853 * (-1.8999999999999999 + x(2)) * ((-0.10000000000000003 + x(3))^2)) / 6) + 2 * ((-0.012509250243078097 * (-1.8999999999999999 + x(2)) * (-0.10000000000000003 + x(3))) / 2) + 2 * ((0.28209432339802509 * (-3.1000000000000001 + x(1)) * (-0.10000000000000003 + x(3))) / 2) + 3 * ((2.8115338151280138 * (-3.1000000000000001 + x(1)) * ((-0.10000000000000003 + x(3))^2)) / 6));
%     xdot(2) = (-5.5511151231257827e-17 - 0.28209432339802509 * (-3.1000000000000001 + x(1)) + 0.012509250243078097 * (-1.8999999999999999 + x(2)) + 1.1905169870535861 * (-0.10000000000000003 + x(3)) + 2 * ((-2.8115338151280138 * (-3.1000000000000001 + x(1)) * (-0.10000000000000003 + x(3))) / 2) + ((-1.2719857791848768 * ((-0.10000000000000003 + x(3))^3)) / 6) + 3 * ((-0.012509250243078097 * (-1.8999999999999999 + x(2)) * ((-0.10000000000000003 + x(3))^2)) / 6) + 2 * ((0.12467524917432853 * (-1.8999999999999999 + x(2)) * (-0.10000000000000003 + x(3))) / 2) + 3 * ((0.28209432339802509 * (-3.1000000000000001 + x(1)) * ((-0.10000000000000003 + x(3))^2)) / 6) + ((0.69660742352866034 * ((-0.10000000000000003 + x(3))^2)) / 2));
%     xdot(3) = (-9.7699626167013776e-15 + 11.223240929675072 * (-3.1000000000000001 + x(1)) - 108.9957386471317 * (-1.8999999999999999 + x(2)) - 110.72125700558496 * (-0.10000000000000003 + x(3)) + ((-172832.12257904885 * ((-0.10000000000000003 + x(3))^3)) / 6) + ((-171787.77838502754 * (-1.8999999999999999 + x(2)) * ((-0.10000000000000003 + x(3))^2)) / 6) + ((-171787.77838502752 * (-1.8999999999999999 + x(2)) * ((-0.10000000000000003 + x(3))^2)) / 6) + ((-171787.77838502749 * (-1.8999999999999999 + x(2)) * ((-0.10000000000000003 + x(3))^2)) / 6) + ((-170745.56775913894 * ((-1.8999999999999999 + x(2))^2) * (-0.10000000000000003 + x(3))) / 6) + 2 * ((-170745.56775913891 * ((-1.8999999999999999 + x(2))^2) * (-0.10000000000000003 + x(3))) / 6) + ((-169705.48721974602 * ((-1.8999999999999999 + x(2))^3)) / 6) + ((-3554.9398598703192 * ((-0.10000000000000003 + x(3))^2)) / 2) + 2 * ((-3524.4550619080669 * (-1.8999999999999999 + x(2)) * (-0.10000000000000003 + x(3))) / 2) + ((-3494.0344345019516 * ((-1.8999999999999999 + x(2))^2)) / 2) + ((16.659002608308189 * ((-3.1000000000000001 + x(1))^2)) / 2) + ((35.363810762918718 * ((-3.1000000000000001 + x(1))^3)) / 6) + 2 * ((109.31752394940582 * (-3.1000000000000001 + x(1)) * (-0.10000000000000003 + x(3))) / 2) + ((110.3672990652795 * (-3.1000000000000001 + x(1)) * (-1.8999999999999999 + x(2))) / 2) + ((110.36729906527951 * (-3.1000000000000001 + x(1)) * (-1.8999999999999999 + x(2))) / 2) + 2 * ((419.22463058230051 * ((-3.1000000000000001 + x(1))^2) * (-0.10000000000000003 + x(3))) / 6) + ((419.22463058230062 * ((-3.1000000000000001 + x(1))^2) * (-0.10000000000000003 + x(3))) / 6) + 3 * ((420.41937577287842 * ((-3.1000000000000001 + x(1))^2) * (-1.8999999999999999 + x(2))) / 6) + 2 * ((904.53307719747136 * (-3.1000000000000001 + x(1)) * ((-0.10000000000000003 + x(3))^2)) / 6) + ((904.53307719747283 * (-3.1000000000000001 + x(1)) * ((-0.10000000000000003 + x(3))^2)) / 6) + 4 * ((941.14875949940506 * (-3.1000000000000001 + x(1)) * (-1.8999999999999999 + x(2)) * (-0.10000000000000003 + x(3))) / 6) + 2 * ((941.14875949940529 * (-3.1000000000000001 + x(1)) * (-1.8999999999999999 + x(2)) * (-0.10000000000000003 + x(3))) / 6) + 2 * ((977.68079947830392 * (-3.1000000000000001 + x(1)) * ((-1.8999999999999999 + x(2))^2)) / 6) + ((977.68079947830552 * (-3.1000000000000001 + x(1)) * ((-1.8999999999999999 + x(2))^2)) / 6));
    U = u - K * (x - xd);
    xdot(1) = U(2)*cos(x(3));
    xdot(2) = U(2)*sin(x(3));
    xdot(3) = U(2)/L*tan(U(1));
    xdot = xdot - [ 1.14978259,  0.11536306, -1.13402183]';
end

function xdot = car_approx_dynamics(t,x)
    xdot = 0*x;

%     xdot(1) = ( - 2.3727308015319104 * x(1) + 0.55708911045314402 * x(2) + 0.10270492023415845 * x(3) + ((-7.531485227842035 * (x(3)^2)) / 2) + ((-1.7855700002399373 * (x(3)^3)) / 6) + 3 * ((-0.55708911045314402 * x(2) * (x(3)^2)) / 6) + 2 * ((-0.055895353219691496 * x(2) * x(3)) / 2) + 2 * ((0.23806716691875224 * x(1) * x(3)) / 2) + 3 * ((2.3727308015319104 * x(1) * (x(3)^2)) / 6));
%     xdot(2) = ( - 0.23806716691875224 * x(1) + 0.055895353219691496 * x(2) + 7.4470603698588169 * x(3) + ((-7.6159100858252522 * (x(3)^3)) / 6) + 2 * ((-2.3727308015319104 * x(1) * x(3)) / 2) + 3 * ((-0.055895353219691496 * x(2) * (x(3)^2)) / 6) + 3 * ((0.23806716691875224 * x(1) * (x(3)^2)) / 6) + 2 * ((0.55708911045314402 * x(2) * x(3)) / 2) + ((0.94413746023704781 * (x(3)^2)) / 2));
%     xdot(3) = ( - 4.123991426359594 * x(1) - 23.646710430174462 * x(2) - 27.530004103469132 * x(3) + ((-842.38518808143465 * (x(3)^3)) / 6) + 2 * ((-724.55952735254027 * x(2) * (x(3)^2)) / 6) + ((-724.55952735254016 * x(2) * (x(3)^2)) / 6) + ((-623.20982967158932 * (x(2)^2) * x(3)) / 6) + 2 * ((-623.20982967158909 * (x(2)^2) * x(3)) / 6) + ((-536.03281201954962 * (x(2)^3)) / 6) + 2 * ((-141.25061610207391 * x(1) * (x(3)^2)) / 6) + ((-141.25061610207388 * x(1) * (x(3)^2)) / 6) + 2 * ((-121.42601259684383 * x(1) * x(2) * x(3)) / 6) + ((-121.42601259684379 * x(1) * x(2) * x(3)) / 6) + 3 * ((-121.42601259684378 * x(1) * x(2) * x(3)) / 6) + 3 * ((-104.38270636332155 * x(1) * (x(2)^2)) / 6) + ((-91.078972005052165 * (x(3)^2)) / 2) + 2 * ((-77.930402411031281 * x(2) * x(3)) / 2) + ((-66.675806993664608 * (x(2)^2)) / 2) + 2 * ((-22.664038859713386 * (x(1)^2) * x(3)) / 6) + ((-22.664038859713383 * (x(1)^2) * x(3)) / 6) + 2 * ((-19.466580820491501 * (x(1)^2) * x(2)) / 6) + ((-19.466580820491497 * (x(1)^2) * x(2)) / 6) + 2 * ((-9.0992598875724564 * x(1) * x(3)) / 2) + ((-7.7221084999052971 * x(1) * x(2)) / 2) + ((-7.7221084999052962 * x(1) * x(2)) / 2) + ((-3.3868985363560489 * (x(1)^3)) / 6) + ((0.049416654099535551 * (x(1)^2)) / 2));
xdot(1) = ( - 2.3727308015319104 * x(1) + 0.55708911045314402 * x(2) + 0.10270492023415845 * x(3) + ((-7.531485227842035 * (x(3)^2)) / 2) + 5 * ((-2.3727308015319104 * x(1) * (x(3)^4)) / 120) + ((-1.7855700002399373 * (x(3)^3)) / 6) + 3 * ((-0.55708911045314402 * x(2) * (x(3)^2)) / 6) + 4 * ((-0.23806716691875224 * x(1) * (x(3)^3)) / 24) + 2 * ((-0.055895353219691496 * x(2) * x(3)) / 2) + 4 * ((0.055895353219691496 * x(2) * (x(3)^3)) / 24) + 2 * ((0.23806716691875224 * x(1) * x(3)) / 2) + 5 * ((0.55708911045314402 * x(2) * (x(3)^4)) / 120) + 3 * ((2.3727308015319104 * x(1) * (x(3)^2)) / 6) + ((3.4684350802457158 * (x(3)^5)) / 120) + ((7.7003349438084703 * (x(3)^4)) / 24));

xdot(2) = ( - 0.23806716691875224 * x(1) + 0.055895353219691496 * x(2) + 7.4470603698588169 * x(3) + ((-7.6159100858252522 * (x(3)^3)) / 6) + ((-2.6270025402428265 * (x(3)^4)) / 24) + 2 * ((-2.3727308015319104 * x(1) * x(3)) / 2) + 4 * ((-0.55708911045314402 * x(2) * (x(3)^3)) / 24) + 5 * ((-0.23806716691875224 * x(1) * (x(3)^4)) / 120) + 3 * ((-0.055895353219691496 * x(2) * (x(3)^2)) / 6) + 5 * ((0.055895353219691496 * x(2) * (x(3)^4)) / 120) + 3 * ((0.23806716691875224 * x(1) * (x(3)^2)) / 6) + 2 * ((0.55708911045314402 * x(2) * x(3)) / 2) + ((0.94413746023704781 * (x(3)^2)) / 2) + 4 * ((2.3727308015319104 * x(1) * (x(3)^3)) / 24) + ((7.7847598017916875 * (x(3)^5)) / 120));

xdot(3) = ( - 4.123991426359594 * x(1) - 23.646710430174462 * x(2) - 27.530004103469132 * x(3) + ((-110493.15292971443 * (x(3)^5)) / 120) + ((-95116.955659901068 * x(2) * (x(3)^4)) / 120) + ((-95116.955659901025 * x(2) * (x(3)^4)) / 120) + 3 * ((-95116.955659900981 * x(2) * (x(3)^4)) / 120) + 3 * ((-81880.224402149397 * (x(2)^2) * (x(3)^3)) / 120) + 2 * ((-81880.224402149383 * (x(2)^2) * (x(3)^3)) / 120) + 2 * ((-81880.224402149368 * (x(2)^2) * (x(3)^3)) / 120) + 2 * ((-81880.224402149339 * (x(2)^2) * (x(3)^3)) / 120) + ((-81880.224402149324 * (x(2)^2) * (x(3)^3)) / 120) + 4 * ((-70485.309076972684 * (x(2)^3) * (x(3)^2)) / 120) + 4 * ((-70485.30907697267 * (x(2)^3) * (x(3)^2)) / 120) + 2 * ((-70485.309076972655 * (x(2)^3) * (x(3)^2)) / 120) + ((-60675.964531675098 * (x(2)^4) * x(3)) / 120) + 4 * ((-60675.964531675054 * (x(2)^4) * x(3)) / 120) + ((-52231.591572552701 * (x(2)^5)) / 120) + ((-19713.70431366445 * x(1) * (x(3)^4)) / 120) + ((-19713.704313664435 * x(1) * (x(3)^4)) / 120) + ((-19713.704313664428 * x(1) * (x(3)^4)) / 120) + 2 * ((-19713.704313664424 * x(1) * (x(3)^4)) / 120) + 3 * ((-16966.105212185485 * x(1) * x(2) * (x(3)^3)) / 120) + 2 * ((-16966.105212185481 * x(1) * x(2) * ...
 (x(3)^3)) / 120) + 6 * ((-16966.105212185477 * x(1) * x(2) * (x(3)^3)) / 120) + 3 * ((-16966.105212185474 * x(1) * x(2) * (x(3)^3)) / 120) + 4 * ((-16966.105212185463 * x(1) * x(2) * (x(3)^3)) / 120) + 2 * ((-16966.105212185459 * x(1) * x(2) * (x(3)^3)) / 120) + 2 * ((-14601.388274813406 * x(1) * (x(2)^2) * (x(3)^2)) / 120) + 8 * ((-14601.388274813398 * x(1) * (x(2)^2) * (x(3)^2)) / 120) + 9 * ((-14601.388274813396 * x(1) * (x(2)^2) * (x(3)^2)) / 120) + 3 * ((-14601.388274813395 * x(1) * (x(2)^2) * (x(3)^2)) / 120) + 8 * ...
 ((-14601.388274813393 * x(1) * (x(2)^2) * (x(3)^2)) / 120) + ((-12566.207064400784 * x(1) * (x(2)^3) * x(3)) / 120) + 7 * ((-12566.207064400776 * x(1) * (x(2)^3) * x(3)) / 120) + 10 * ((-12566.207064400774 * x(1) * (x(2)^3) * x(3)) / 120) + 2 * ((-12566.207064400773 * x(1) * (x(2)^3) * x(3)) / 120) + ((-10814.646635821642 * x(1) * (x(2)^4)) / 120) + ((-10814.646635821637 * x(1) * (x(2)^4)) / 120) + ((-10814.646635821635 * x(1) * (x(2)^4)) / 120) + 2 * ((-10814.646635821633 * x(1) * (x(2)^4)) / 120) + ((-8072.9198543635648 * (x(3)^4)) / 24) + ((-6943.7558718657783 * x(2) * (x(3)^3)) / 24) + 3 * ((-6943.7558718657774 * x(2) * (x(3)^3)) / 24) + 3 * ((-5972.4857510373367 * (x(2)^2) * (x(3)^2)) / 24) + 3 * ((-5972.4857510373358 * (x(2)^2) * (x(3)^2)) / 24) + 3 * ((-5137.0366052057088 * (x(2)^3) * x(3)) / 24) + ((-5137.0366052057079 * (x(2)^3) * x(3)) / 24) + ((-4418.4204562905716 * (x(2)^4)) / 24) + ((-3453.1945496815774 * (x(1)^2) * (x(3)^3)) / 120) + 2 * ((-3453.194549681577 * (x(1)^2) * (x(3)^3)) / 120) + ((-3453.1945496815761 * (x(1)^2) * (x(3)^3)) / 120) + 2 * ((-3453.1945496815752 * (x(1)^2) * (x(3)^3)) / 120) + ((-3453.1945496815742 * (x(1)^2) * (x(3)^3)) / 120) + 3 * ((-3453.1945496815733 * (x(1)^2) * (x(3)^3)) / 120) + 2 * ((-2970.9296530982729 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + ((-2970.929653098272 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + 9 * ((-2970.9296530982715 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + 5 * ((-2970.929653098271 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + ((-2970.9296530982706 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + 2 * ((-2970.9296530982701 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + 2 * ((-2970.9296530982697 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + 5 * ((-2970.9296530982692 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + 2 * ((-2970.9296530982683 * ...
 (x(1)^2) * x(2) * (x(3)^2)) / 120) + ((-2970.9296530982679 * (x(1)^2) * x(2) * (x(3)^2)) / 120) + 13 * ((-2556.0015796433322 * (x(1)^2) * (x(2)^2) * x(3)) / 120) + 9 * ((-2556.0015796433318 * (x(1)^2) * (x(2)^2) * x(3)) / 120) + 5 * ((-2556.0015796433313 * (x(1)^2) * (x(2)^2) * x(3)) / 120) + 2 * ((-2556.0015796433304 * (x(1)^2) * (x(2)^2) * x(3)) / 120) + ((-2556.0015796433299 * (x(1)^2) * (x(2)^2) * x(3)) / 120) + ((-2199.0103948652732 * (x(1)^2) * (x(2)^3)) / 120) + 5 * ((-2199.0103948652722 * (x(1)^2) * (x(2)^3)) / 120) + ((-2199.0103948652713 * (x(1)^2) * (x(2)^3)) / 120) + 3 * ((-2199.0103948652709 * (x(1)^2) * (x(2)^3)) / 120) + ((-1353.771368733253 * x(1) * (x(3)^3)) / 24) + 3 * ((-1353.7713687332523 * x(1) * (x(3)^3)) / 24) + 2 * ((-1163.7705661584687 * x(1) * x(2) * (x(3)^2)) / 24) + 2 * ((-1163.7705661584685 * x(1) * x(2) * (x(3)^2)) / 24) + 2 * ((-1163.7705661584682 * x(1) * x(2) * (x(3)^2)) / 24) + 5 * ((-1163.770566158468 * x(1) * x(2) * (x(3)^2)) / 24) + ((-1163.7705661584678 * x(1) * x(2) * (x(3)^2)) / 24) + 2 * ((-1000.4257306997423 * x(1) * (x(2)^2) * x(3)) / 24) + 3 * ((-1000.4257306997422 * x(1) * (x(2)^2) * x(3)) / 24) + 7 * ((-1000.4257306997421 * x(1) * (x(2)^2) * x(3)) / 24) + ...
 ((-859.99860715463581 * x(1) * (x(2)^3)) / 24) + 3 * ((-859.99860715463558 * x(1) * (x(2)^3)) / 24) + ((-842.38518808143465 * (x(3)^3)) / 6) + 2 * ((-724.55952735254027 * x(2) * (x(3)^2)) / 6) + ((-724.55952735254016 * x(2) * (x(3)^2)) / 6) + ((-623.20982967158932 * (x(2)^2) * x(3)) / 6) + 2 * ((-623.20982967158909 * (x(2)^2) * x(3)) / 6) + ((-590.16967066391805 * (x(1)^3) * (x(3)^2)) / 120) + 5 * ((-590.16967066391794 * (x(1)^3) * (x(3)^2)) / 120) + 2 * ((-590.16967066391783 * (x(1)^3) * (x(3)^2)) / 120) + ((-590.1696706639176 * (x(1)^3) * (x(3)^2)) / 120) + ((-590.16967066391726 * (x(1)^3) * (x(3)^2)) / 120) + ((-536.03281201954962 * (x(2)^3)) / 6) + ((-507.51961473604911 * ...
 (x(1)^3) * x(2) * x(3)) / 120) + 2 * ((-507.51961473604899 * (x(1)^3) * x(2) * x(3)) / 120) + 2 * ((-507.51961473604894 * (x(1)^3) * x(2) * x(3)) / 120) + 4 * ((-507.51961473604888 * (x(1)^3) * x(2) * x(3)) / 120) + 3 * ((-507.51961473604882 * (x(1)^3) * x(2) * x(3)) / 120) + 3 * ((-507.51961473604877 * (x(1)^3) * x(2) * x(3)) / 120) + 2 * ((-507.5196147360486 * (x(1)^3) * x(2) * x(3)) / 120) + 2 * ((-507.51961473604854 * (x(1)^3) * x(2) * x(3)) / 120) + ((-507.51961473604848 * (x(1)^3) * x(2) * x(3)) / 120) + 2 * ((-436.44061794410965 * (x(1)^3) * (x(2)^2)) / 120) + ((-436.44061794410959 * (x(1)^3) * (x(2)^2)) / 120) + ((-436.44061794410953 * (x(1)^3) * (x(2)^2)) / 120) + ((-436.44061794410948 * (x(1)^3) * (x(2)^2)) / 120) + 3 * ((-436.44061794410925 * (x(1)^3) * (x(2)^2)) / 120) + 2 * ((-436.44061794410914 * (x(1)^3) * (x(2)^2)) / 120) + 3 * ((-217.24295321791146 * (x(1)^2) * ...
 (x(3)^2)) / 24) + 3 * ((-217.24295321791141 * (x(1)^2) * (x(3)^2)) / 24) + 4 * ((-186.59462205028663 * (x(1)^2) * x(2) * x(3)) / 24) + ((-186.5946220502866 * (x(1)^2) * x(2) * x(3)) / 24) + 3 * ((-186.59462205028657 * (x(1)^2) * x(2) * x(3)) / 24) + 3 * ((-186.59462205028655 * (x(1)^2) * x(2) * x(3)) / 24) + ((-186.59462205028652 * (x(1)^2) * x(2) * x(3)) / 24) + 5 * ((-160.26743061880228 * (x(1)^2) * (x(2)^2)) / 24) + ((-160.26743061880222 * (x(1)^2) * (x(2)^2)) / 24) + 2 * ((-141.25061610207391 * x(1) * (x(3)^2)) / 6) + ((-141.25061610207388 * x(1) * (x(3)^2)) / 6) + 2 * ((-121.42601259684383 * x(1) * x(2) * x(3)) / 6) + ((-121.42601259684379 * x(1) * x(2) * x(3)) / 6) + 3 * ...
 ((-121.42601259684378 * x(1) * x(2) * x(3)) / 6) + 3 * ((-104.38270636332155 * x(1) * (x(2)^2)) / 6) + ((-97.418377973522809 * (x(1)^4) * x(3)) / 120) + 3 * ((-97.41837797352278 * (x(1)^4) * x(3)) / 120) + ((-97.418377973522723 * (x(1)^4) * x(3)) / 120) + ((-91.078972005052165 * (x(3)^2)) / 2) + ((-83.720674796784664 * (x(1)^4) * x(2)) / 120) + 2 * ((-83.72067479678465 * (x(1)^4) * x(2)) / 120) + ((-83.720674796784635 * (x(1)^4) * x(2)) / 120) + ((-83.720674796784579 * (x(1)^4) * x(2)) / 120) + 2 * ((-77.930402411031281 * x(2) * x(3)) / 2) + ((-66.675806993664608 * (x(2)^2)) / 2) + ((-32.471466805275689 * (x(1)^3) * x(3)) / 24) + 2 * ((-32.471466805275682 * (x(1)^3) * x(3)) / 24) + ((-32.471466805275668 * (x(1)^3) * x(3)) / 24) + ((-27.849953801845903 * (x(1)^3) * x(2)) / 24) + ((-27.849953801845899 * (x(1)^3) * x(2)) / 24) + ((-27.849953801845892 * (x(1)^3) * x(2)) / 24) + ((-27.849953801845881 * (x(1)^3) * x(2)) / 24) + 2 * ((-22.664038859713386 * (x(1)^2) * x(3)) / 6) + ((-22.664038859713383 * (x(1)^2) * x(3)) / 6) + 2 ...
 * ((-19.466580820491501 * (x(1)^2) * x(2)) / 6) + ((-19.466580820491497 * (x(1)^2) * x(2)) / 6) + ((-15.254247610117677 * (x(1)^5)) / 120) + 2 * ((-9.0992598875724564 * x(1) * x(3)) / 2) + ((-7.7221084999052971 * x(1) * x(2)) / 2) + ((-7.7221084999052962 * x(1) * x(2)) / 2) + ((-4.2428596711491933 * (x(1)^4)) / 24) + ((-3.3868985363560489 * (x(1)^3)) / 6) + ((0.049416654099535551 * (x(1)^2)) / 2));
end

