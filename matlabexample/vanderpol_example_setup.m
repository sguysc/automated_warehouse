%% Disable Warning

warning('off','MATLAB:nearlySingularMatrix');

%% Define the Dynamics
n=2;   % Number of States

% Generate a funciton f0 which uses simple arithmetic.
f0 = @(t,x,u) [-x(2) ; x(1) + x(2)*(x(1)^2-1)]; % Populate this with your dynamics.

[ts,xlim] = ode45(@(t,x) f0(t,x,0),[0 -6.69],[-0.1144;2.0578]);


%% Define a goal region.
% x0 is the center of the goal region.
% S0 defines the goal region (x-xT)'S0(x-xT) <= 1.
% Here we find an ellipse contained in an inner-estimate
% of the basin of attraction for the system.

% Pick an initial condition in the basin.
xT = [-1 ; -1];

% Find the basin of attraction estimate 
x0 = zeros(n,1);
Q = eye(n);
x = msspoly('x',2);

A = double(subs(diff(f0(0,x,0),x),x,x0));

xbar = x;
x = xbar + x0;
P0 = lyap(A',Q);
rho0 = ti_poly_roa(xbar,f0(0,x),...
                  xbar'*P0*xbar);
P = P0/rho0;

% Calculate the contained ellispe.
S0 = inner_ellipse(P,x0,xT);

%S0 = 1.01*P0/rho0;


%% Now the Timevarying case.


tspan = [0 2]; % duration of input.

[ts,xs] = ode45(@(t,x)f0(t,x,0),fliplr(tspan),xT);

t0s = flipud(ts);
x0s = flipud(xs);

%%  See how Ricatti works

xpp = spline(t0s,x0s');

[A,B] = tv_poly_linearize(f0,@(t) ppval(xpp,t),@(t) 0);

% You will need to replace these with appropriate cost functions for your
% LQR control task.
Q = @(t) eye(2);

[ts,Ss] = tv_lyapunov(tspan,A,Q,S0);

Spp = spline(ts,Ss);
S = @(t) ppval(Spp,t);
Ac = @(t) A(t) - B(t)*K(t);
taus = ts;

N = length(taus);
Ppp = interp1(taus,reshape(permute(Ss,[3 1 2]),N,n*n),'linear', ...
              'pp');

%% This example is undriven, so turn off feedback and nominal command.
upp = spline(taus,ones(N,1));
K = @(t) [ 0 0];