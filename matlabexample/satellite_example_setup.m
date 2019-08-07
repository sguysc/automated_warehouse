%% Disable Warning

warning('off','MATLAB:nearlySingularMatrix');

%% Define the Dynamics
n=6;   % Number of States
m = 3; % Number of Inputs

% Generate a funciton f0 which uses simple arithmetic.
Is = [5 3 2]; I = diag(Is);
l = [2 3 1]; r = [3 1 2];
f = @(omega,sigma,T)...
    [diag(Is(r)-Is(l))*inv(I)*omega(l).*omega(r) ;
     (1/4)*[(1-sigma'*sigma)*eye(3)+2*sigma*sigma' - ...
            2*(sigma([1 3 2; 3 2 1; 2 1 3])-diag(sigma))]*omega]...
    + [inv(I)*T ; zeros(3,1)];
f0 = @(t,x,u) f(x(1:3),x(4:6),u); % Populate this with your dynamics.


%% Define a goal region.
xT = zeros(n,1);
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
u0 = @(t) [ -(t).*(t+5).*(t-5)/100;
            -(t).*(t+5).*(t-5)/100;
            (t).*(t+5).*(t-5)/100];
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
Q = @(t) diag([ 1 1 1 10 10 10]);
R = @(t) 10*eye(3);

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

