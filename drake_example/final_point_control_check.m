function final_point_control_check()
close all
times=0:0.01:10;
xf = [ 1.50000000e+00; 0;  7.85398163e-01];
x0 = xf+0.1*[2*(rand(2,1)-.5); 2*(rand(1,1)-.5)];
% x0 = [1.04; .332 ; 0.797];
figure;
% subplot(211);
options = odeset('RelTol',1e-5,'Stats','on');%,'OutputFcn',@odeplot);
[T, xcl] = ode45(@(t,x)approx_dynamics(t, x, xf), ...
                    times, x0, options) ;
% figure;
% plot(xcl(:,1), xcl(:,2)); hold all
%plot(x(1,:), x(2,:));
hold all;
V = (((-1.4999999999999998 + xcl(:,1)) .* (4.8008600999593014 .* (-1.4999999999999998 + xcl(:,1)) ...
       - 0.98790740152737377 .* (-0.78539816339744839 + xcl(:,3)) - 1.4491942689985993 .* ...
       (6.9388939039072284e-18 + xcl(:,2)))) + ((-0.78539816339744839 + xcl(:,3)) .* ( - ...
       0.98790740152737322 .* (-1.4999999999999998 + xcl(:,1)) + 1.4361830490185388 .* ...
       (-0.78539816339744839 + xcl(:,3)) + 0.68372066922887398 .* (6.9388939039072284e-18 + ...
       xcl(:,2)))) + (( - 1.4491942689985997 .* (-1.4999999999999998 + xcl(:,1)) + ...
       0.68372066922887442 .* (-0.78539816339744839 + xcl(:,3)) + 4.5682688351564416 ...
       * (6.9388939039072284e-18 + xcl(:,2))) .* (6.9388939039072284e-18 + xcl(:,2))));
Vdot = diff(V)./diff(T);
plot(T, xcl(:,1),'b-', T, xcl(:,2), 'b-');
plot(T, V, 'r-');
plot(T(2:end), 10*Vdot, 'g-');
Qf =  [ 4.8008601 , -1.44919427, -0.9879074 ;
       -1.44919427,  4.56826884,  0.68372067;
       -0.9879074 ,  0.68372067,  1.43618305];
tmp = Qf*(xcl-xf')';
% Vanother = zeros(size(T));
% for i=1:size(tmp,2)
%     Vanother(i) = (xcl(i,:)-xf')*tmp(:,i);
% end
% plot(T, 100*Vanother, '.');  
   
% subplot(212);
[T, xcl] = ode45(@(t,x)full_dynamics(t, x, xf), ...
                    times, x0, options) ;
hold all;
V = (((-1.4999999999999998 + xcl(:,1)) .* (4.8008600999593014 .* (-1.4999999999999998 + xcl(:,1)) ...
       - 0.98790740152737377 .* (-0.78539816339744839 + xcl(:,3)) - 1.4491942689985993 .* ...
       (6.9388939039072284e-18 + xcl(:,2)))) + ((-0.78539816339744839 + xcl(:,3)) .* ( - ...
       0.98790740152737322 .* (-1.4999999999999998 + xcl(:,1)) + 1.4361830490185388 .* ...
       (-0.78539816339744839 + xcl(:,3)) + 0.68372066922887398 .* (6.9388939039072284e-18 + ...
       xcl(:,2)))) + (( - 1.4491942689985997 .* (-1.4999999999999998 + xcl(:,1)) + ...
       0.68372066922887442 .* (-0.78539816339744839 + xcl(:,3)) + 4.5682688351564416 ...
       * (6.9388939039072284e-18 + xcl(:,2))) .* (6.9388939039072284e-18 + xcl(:,2))));
Vdot = diff(V)./diff(T);
plot(T, xcl(:,1), 'b:', T, xcl(:,2), 'b:');
plot(T, V, 'r:');
plot(T(2:end), 10*Vdot, 'g:');

end


function xdot = the_dynamics(t, x)
    xdot = x;

    xdot(1) = (( - 1.7996170331373038 * (-1.4999999999999998 + x(1)) - 0.614088168801185 * (-0.78539816339744839 + x(3)) - 2.6002650891865011 * (6.9388939039072284e-18 + x(2))) * (0.70710678118654746 - 0.70710678118654757 * (-0.78539816339744839 + x(3)) - ((0.70710678118654746 * ((-0.78539816339744839 + x(3))^2)) / 2) + ((0.70710678118654746 * ((-0.78539816339744839 + x(3))^4)) / 24) + ((0.70710678118654757 * ((-0.78539816339744839 + x(3))^3)) / 6)));
    xdot(2) = (( - 1.7996170331373038 * (-1.4999999999999998 + x(1)) - 0.614088168801185 * (-0.78539816339744839 + x(3)) - 2.6002650891865011 * (6.9388939039072284e-18 + x(2))) * (0.70710678118654757 + 0.70710678118654746 * (-0.78539816339744839 + x(3)) - ((0.70710678118654746 * ((-0.78539816339744839 + x(3))^3)) / 6) - ((0.70710678118654757 * ((-0.78539816339744839 + x(3))^2)) / 2)));
    xdot(3) = (( - 1.7996170331373038 * (-1.4999999999999998 + x(1)) - 0.614088168801185 * (-0.78539816339744839 + x(3)) - 2.6002650891865011 * (6.9388939039072284e-18 + x(2))) * (2.600265089186502 * (-1.4999999999999998 + x(1)) - 3.7801687063692455 * (-0.78539816339744839 + x(3)) - 1.7996170331373023 * (6.9388939039072284e-18 + x(2)) + ((2 * ((2.600265089186502 * (-1.4999999999999998 + x(1)) - 3.7801687063692455 * (-0.78539816339744839 + x(3)) - 1.7996170331373023 * (6.9388939039072284e-18 + x(2)))^3)) / 6)));
    
end


function xdot = approx_dynamics(t,x,x_d)
    xdot = x;
    L=1;
    u_d = [0;0];
    K = [-2.60026509,  1.79961703,  3.78016871; ...
          1.79961703,  2.60026509,  0.61408817];

    
    U = u_d - K*(x-x_d); 
	delta_fb = U(1);
	u_fb = U(2);
	theta = x(3);
            
%   xdot(1) = u_fb*taylor_cos(theta, x_d(3));
% 	xdot(2) = u_fb*taylor_sin(theta, x_d(3));
% 	xdot(3) = u_fb*taylor_tan(delta_fb, u_d(1))/L;
%     xdot(1) = ( - 1.2725214076702032 * (-1.4999999999999998 + x(1)) - 0.43422590840574715 * (-0.78539816339744839 + x(3)) - 1.8386650774464175 * (6.9388939039072284e-18 + x(2)) + ((0.86845181681149441 * ((-0.78539816339744839 + x(3))^2)) / 2) + 3 * ((1.2725214076702032 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^2)) / 6) + 2 * ((1.2725214076702034 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3))) / 2) + ((1.3026777252172415 * ((-0.78539816339744839 + x(3))^3)) / 6) + 3 * ((1.8386650774464175 * ((-0.78539816339744839 + x(3))^2) * (6.9388939039072284e-18 + x(2))) / 6) + 2 * ((1.8386650774464179 * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 2));
%     xdot(2) = ( - 1.2725214076702034 * (-1.4999999999999998 + x(1)) - 0.43422590840574721 * (-0.78539816339744839 + x(3)) - 1.8386650774464179 * (6.9388939039072284e-18 + x(2)) + 2 * ((-1.8386650774464175 * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 2) + 2 * ((-1.2725214076702032 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3))) / 2) + ((-0.8684518168114943 * ((-0.78539816339744839 + x(3))^2)) / 2) + 3 * ((1.2725214076702034 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^2)) / 6) + ((1.3026777252172417 * ((-0.78539816339744839 + x(3))^3)) / 6) + 3 * ((1.8386650774464179 * ((-0.78539816339744839 + x(3))^2) * (6.9388939039072284e-18 + x(2))) / 6));
%     xdot(3) = (((-9.3589626903446383 * ((-1.4999999999999998 + x(1))^2)) / 2) + 2 * ((-3.522757068084176 * (-1.4999999999999998 + x(1)) * (6.9388939039072284e-18 + x(2))) / 2) + ((4.6427137573076687 * ((-0.78539816339744839 + x(3))^2)) / 2) + 2 * ((5.2060639650985125 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3))) / 2) + ((9.3589626903446277 * ((6.9388939039072284e-18 + x(2))^2)) / 2) + 2 * ((10.934564246829954 * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 2));
xdot(1) = ( - 1.2725214076702032 * (-1.4999999999999998 + x(1)) - 0.43422590840574715 * (-0.78539816339744839 + x(3)) - 1.8386650774464175 * (6.9388939039072284e-18 + x(2)) + 4 * ((-1.8386650774464179 * ((-0.78539816339744839 + x(3))^3) * (6.9388939039072284e-18 + x(2))) / 24) + ((-1.7369036336229888 * ((-0.78539816339744839 + x(3))^4)) / 24) + 4 * ((-1.2725214076702034 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^3)) / 24) + ((0.86845181681149441 * ((-0.78539816339744839 + x(3))^2)) / 2) + 3 * ((1.2725214076702032 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^2)) / 6) + 2 * ((1.2725214076702034 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3))) / 2) + ((1.3026777252172415 * ((-0.78539816339744839 + x(3))^3)) / 6) + 3 * ((1.8386650774464175 * ((-0.78539816339744839 + x(3))^2) * (6.9388939039072284e-18 + x(2))) / 6) + 2 * ((1.8386650774464179 * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 2));
xdot(2) = ( - 1.2725214076702034 * (-1.4999999999999998 + x(1)) - 0.43422590840574721 * (-0.78539816339744839 + x(3)) - 1.8386650774464179 * (6.9388939039072284e-18 + x(2)) + 2 * ((-1.8386650774464175 * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 2) + 2 * ((-1.2725214076702032 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3))) / 2) + ((-0.8684518168114943 * ((-0.78539816339744839 + x(3))^2)) / 2) + 4 * ((1.2725214076702032 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^3)) / 24) + 3 * ((1.2725214076702034 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^2)) / 6) + ((1.3026777252172417 * ((-0.78539816339744839 + x(3))^3)) / 6) + ((1.7369036336229886 * ((-0.78539816339744839 + x(3))^4)) / 24) + 4 * ((1.8386650774464175 * ((-0.78539816339744839 + x(3))^3) * (6.9388939039072284e-18 + x(2))) / 24) + 3 * ((1.8386650774464179 * ((-0.78539816339744839 + x(3))^2) * (6.9388939039072284e-18 + x(2))) / 6));
xdot(3) = (((-253.11795774158807 * ((-1.4999999999999998 + x(1))^4)) / 24) + 6 * ((-204.69078848468808 * ((-1.4999999999999998 + x(1))^2) * ((-0.78539816339744839 + x(3))^2)) / 24) + 9 * ((-150.26579688001956 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3)) * ((6.9388939039072284e-18 + x(2))^2)) / 24) + 3 * ((-150.26579688001954 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3)) * ((6.9388939039072284e-18 + x(2))^2)) / 24) + 6 * ((-144.12909521105536 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^2) * (6.9388939039072284e-18 + x(2))) / 24) + 5 * ((-144.12909521105533 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^2) * (6.9388939039072284e-18 + x(2))) / 24) + ((-144.1290952110553 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^2) * (6.9388939039072284e-18 + x(2))) / 24) + 4 * ((-110.40793595936748 * (-1.4999999999999998 + x(1)) * ((6.9388939039072284e-18 + x(2))^3)) / 24) + ((-9.3589626903446383 * ((-1.4999999999999998 + x(1))^2)) / 2) + 2 * ((-3.522757068084176 * (-1.4999999999999998 + x(1)) * (6.9388939039072284e-18 + x(2))) / 2) + ((4.6427137573076687 * ((-0.78539816339744839 + x(3))^2)) / 2) + 2 * ((5.2060639650985125 * (-1.4999999999999998 + x(1)) * (-0.78539816339744839 + x(3))) / 2) + ((9.3589626903446277 * ((6.9388939039072284e-18 + x(2))^2)) / 2) + 2 * ((10.934564246829954 * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 2) + 6 * ((20.530105119262458 * ((-1.4999999999999998 + x(1))^2) * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 24) + 3 * ((20.530105119262466 * ((-1.4999999999999998 + x(1))^2) * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 24) + 2 * ((20.530105119262487 * ((-1.4999999999999998 + x(1))^2) * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 24) + ((20.530105119262494 * ((-1.4999999999999998 + x(1))^2) * (-0.78539816339744839 + x(3)) * (6.9388939039072284e-18 + x(2))) / 24) + 2 * ((39.952794597684104 * ((-1.4999999999999998 + x(1))^3) * (6.9388939039072284e-18 + x(2))) / 24) + 2 * ((39.952794597684111 * ((-1.4999999999999998 + x(1))^3) * (6.9388939039072284e-18 + x(2))) / 24) + 4 * ((57.51536955197858 * (-1.4999999999999998 + x(1)) * ((-0.78539816339744839 + x(3))^3)) / 24) + 4 * ((65.938703934695283 * ((-1.4999999999999998 + x(1))^2) * ((6.9388939039072284e-18 + x(2))^2)) / 24) + 2 * ((65.938703934695297 * ((-1.4999999999999998 + x(1))^2) * ((6.9388939039072284e-18 + x(2))^2)) / 24) + ((121.24054987219706 * ((6.9388939039072284e-18 + x(2))^4)) / 24) + 2 * ((198.16117981733629 * (-0.78539816339744839 + x(3)) * ((6.9388939039072284e-18 + x(2))^3)) / 24) + 2 * ((198.16117981733632 * (-0.78539816339744839 + x(3)) * ((6.9388939039072284e-18 + x(2))^3)) / 24) + ((254.38707618198993 * ((-1.4999999999999998 + x(1))^3) * (-0.78539816339744839 + x(3))) / 24) + 3 * ((254.38707618198995 * ((-1.4999999999999998 + x(1))^3) * (-0.78539816339744839 + x(3))) / 24) + ((265.37149117095504 * ((-0.78539816339744839 + x(3))^4)) / 24) + 6 * ((297.54506363084107 * ((-0.78539816339744839 + x(3))^2) * ((6.9388939039072284e-18 + x(2))^2)) / 24) + 4 * ((375.67017472556995 * ((-0.78539816339744839 + x(3))^3) * (6.9388939039072284e-18 + x(2))) / 24));
end


function xdot = full_dynamics(t,x,x_d)
    xdot = x;
    L=1;
    u_d = [0;0];
    K = [-2.60026509,  1.79961703,  3.78016871; ...
          1.79961703,  2.60026509,  0.61408817];

    
    U = u_d - K*(x-x_d); 
	delta_fb = U(1);
	u_fb = U(2);
	theta = x(3);
            
%   xdot(1) = u_fb*cos(theta);
% 	xdot(2) = u_fb*sin(theta);
% 	xdot(3) = u_fb*tan(delta_fb)/L;
    xdot(1) = (( - 1.7996170331373038 * (-1.4999999999999998 + x(1)) - 0.614088168801185 * (-0.78539816339744839 + x(3)) - 2.6002650891865011 * (6.9388939039072284e-18 + x(2))) * cos(x(3)));
    xdot(2) = (( - 1.7996170331373038 * (-1.4999999999999998 + x(1)) - 0.614088168801185 * (-0.78539816339744839 + x(3)) - 2.6002650891865011 * (6.9388939039072284e-18 + x(2))) * sin(x(3)));
    xdot(3) = (( - 1.7996170331373038 * (-1.4999999999999998 + x(1)) - 0.614088168801185 * (-0.78539816339744839 + x(3)) - 2.6002650891865011 * (6.9388939039072284e-18 + x(2))) * tan((2.600265089186502 * (-1.4999999999999998 + x(1)) - 3.7801687063692455 * (-0.78539816339744839 + x(3)) - 1.7996170331373023 * (6.9388939039072284e-18 + x(2)))));
end

function tc = taylor_cos(x, x0)
	tc = ( cos(x0) - sin(x0)*(x-x0) - (cos(x0)*(x-x0)^2)/2.0 ...
		 + (sin(x0)*(x-x0)^3)/6.0 + (cos(x0)*(x-x0)^4)/24.0 ) ;
end

function ts = taylor_sin(x, x0)
    ts = ( sin(x0) + cos(x0)*(x-x0) - (sin(x0)*(x-x0)^2)/2.0 ...
         - (cos(x0)*(x-x0)^3)/6.0  ) ;
end
function tt = taylor_tan(x, x0)
	tt = ( tan(x0) + (tan(x0)^2+1.0)*(x-x0) + ...
	     ((2.0*tan(x0)*(tan(x0)^2 + 1.0))*(x-x0)^2)/2.0  + ...
	     ((2.0*(tan(x0)^2 + 1)^2 + (4.0*tan(x0)^2)*(tan(x0)^2 + 1.0))...
             *(x-x0)^3)/6.0 );
end