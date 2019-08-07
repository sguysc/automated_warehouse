satellite_example_setup;

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