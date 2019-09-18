

%% 
function taylor_attempt()
syms x x0 y y0 t t0 del del0 u u0 L real



f0 = f(x0,y0,t0,del0,u0, L);
disp(f0)



end

function ret = f(x,y,t,del,u , L)
    ret = [u*cos(t); ...
     u*sin(t); ...
     u/L*tan(del)];
end