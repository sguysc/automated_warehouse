%% Disable Warning
close all
clear all
clc

warning('off','MATLAB:nearlySingularMatrix');

%% Define the Dynamics
mass=1; g=10; l=1;

n=2;   % Number of States
m=1;   % Number of inputs

% Generate a funciton f0 which uses simple arithmetic.
% f0 = @(t,x,u) [Vconst*cos(x(3)) ; Vconst*sin(x(3)); u]; % car dynamics.
f0 = @(t,x,u) [x(2); 1/mass/l^2 * (u-mass*g*l*(x(1)-pi))]; % pendulum dynamics.

% [ts, xlim] = ode45(@(t,x) f0(t,x,0),[0 -10],[5;5;pi/4]);

%% Define a goal region.
xT = [pi; 0]; %zeros(n,1);
u0 = zeros(m,1);
Q = eye(n);
R = 10*eye(m);
[K0,S0,rho0] = ti_poly_lqr_roa(@(x,u) f0(0,x,u),xT,u0,Q,R);
S0 = 1.01*S0/rho0;

% xT is the center of the goal region.
% S0 defines the goal region (x-xT)'S(x-xT) <= 1.
% We use an estimate of the  basin of attraction for infinite
% horizon LQR, but choose your own.

%% Now the Timevarying case.

% Pick an input signal, we integrate backwards form the goal (you would
% need to replace this with the output of a trajectory optimization).
u0 = @(t) [ -(t).*(t+5).*(t-5)/10 ];
tspan = [0 5]; % duration of input.

[ts,xs] = ode45(@(t,x)f0(t,x,u0(t)),fliplr(tspan),xT);

t0s = flipud(ts);
x0s = flipud(xs);

%%  See how Ricatti works

xpp = spline(t0s,x0s');
upp = spline(t0s,u0(t0s'));

[A,B] = tv_poly_linearize(f0,@(t) ppval(xpp,t),@(t) ppval(upp,t));

% You will need to replace these with appropriate cost functions for your
% LQR control task.
Q = @(t) diag([ 1 1 ]);
R = @(t) 1*eye(m);

[ts,Ss] = tv_lqr_riccati(tspan,...
                              A,B,...
                              Q,R,S0);
Spp = spline(ts,Ss);
S = @(t) ppval(Spp,t);
K = @(t) inv(R(t))*B(t)'*S(t);
Ac = @(t) A(t) - B(t)*K(t);
Q0  = @(t) (Q(t) + S(t)*B(t)*inv(R(t))*B(t)'*S(t));

[taus,Ps] = tv_lyapunov(tspan,@(t) Ac(t),Q0,S0);
N = length(taus);
Ppp = interp1(taus,reshape(permute(Ps,[3 1 2]),N,n*n),'linear', ...
              'pp');
upp = spline(taus,u0(taus'));

%%

c = 3;
rhot = exp(c*(taus-max(taus))/(max(taus)-min(taus)));
rhopp = interp1(taus,rhot,'linear','pp');

figure;
for iter=1:2
    lagrange_multipliers;
    if ~all([gammas{:}] < 0), 
        if iter == 1, error('Initial rho(t) infeasible, increase c.'); 
        end
    end
    plot(ts,ppval(rhopp,ts)); drawnow;
    improve_rho;
    plot(ts,ppval(rhopp,ts)); drawnow;
end
