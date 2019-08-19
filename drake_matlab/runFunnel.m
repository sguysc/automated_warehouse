% function runFunnel

% Declare Dubins car model
p = DubinsPlant();

% Set input limits
% p = setInputLimits(p,[-inf 0]',[inf 1]');
p = setInputLimits(p,[-inf]',[inf]');

% Trajectory optimization
x0 = [0;0; 0*pi]; % Initial state that trajectory should start from
xf = [1.5;0;-0*pi/4]; % Final desired state
tf0 = 1.5; % Guess for how long trajectory should take
[utraj,xtraj]=runDircol(p,x0,xf,tf0);

% Do tvlqr
Q = eye(3); %eye(4);
R = 1; % 1*eye(2);
Qf = 1*Q;

[c,V]=tvlqr(p,xtraj,utraj,Q,R,Qf);
CL = feedback(p,c);
poly = taylorApprox(CL,xtraj,[],3);
ts = xtraj.getBreaks();

% Options for funnel computation
options = struct();
options.rho0_tau = 2; % Determine initial guess for rho
options.max_iterations = 5; % Maximum number of iterations to run for
options.stability = false;

% Do funnel computation
Vtraj=sampledFiniteTimeVerification(poly,ts,Qf,V,options);
disp('done');

% Convert V back to state frame and plot it
Vxframe = Vtraj.inFrame(p.getStateFrame());
figure(1)
options.plotdims = [1 2];
%
plotFunnel(Vxframe,options);
fnplt(xtraj,[1 2]);
axis equal
hold on

% Tests to make sure simulated trajectories stay inside computed funnel
doTest = 1;
if doTest
    
    % Create closed loop system with optimized controller
    sysCl = feedback(p,c);
    V0 = Vtraj.getPoly(0); % Get inlet of funnel
    options.x0 = [0;0;0];
    xinit = getLevelSet(decomp(V0),V0,options);
    
    Vsall = [];
    figure(2)
    hold on
    grid on
    for j = 1:5
        Vs = [];
        % Guy changed and added a uniform random var for theta
        x0 = [0.95*xinit(:,j); 0*pi*2*(rand(1)-0.5)] + xtraj.eval(0); % Simulate from 0.95*boundary of funnel
        xsim = sysCl.simulate([0 ts(numel(ts))],x0);
        for k = 1:length(ts)
            Vs = [Vs, Vxframe.eval(ts(k),xsim.eval(ts(k)))];
        end
        Vsall = [Vsall, Vs];
        
        plot(ts,Vs)
        plot(ts,ones(1,length(ts)),'ro')
        drawnow;
        figure(1)
        local_sim = xsim.eval(0:0.01:ts(numel(ts)));
        plot(local_sim(1,:), local_sim(2,:), 'DisplayName', ['j=' num2str(j)]);
        figure(2)
    end
    
    if ~(all(Vsall < 1))
        success = false;
        disp('A Trajectory left the funnel!!')
    else
        success = true;
        disp('All simulated trajectories stayed inside funnel.')
    end
    
end

toc