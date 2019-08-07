vanderpol_example_setup;

c = 3;
rhot = exp(c*(taus-max(taus))/(max(taus)-min(taus)));
rhopp = interp1(taus,rhot,'linear','pp');




% Repeat to improve rho
figure;
for iter=1:1
    lagrange_multipliers;
    if ~all([gammas{:}] < 0), 
        if iter == 1, error('Initial rho(t) infeasible, increase c.'); 
        end
    end
    plot_vanderpol
    improve_rho;
end
plot_vanderpol
