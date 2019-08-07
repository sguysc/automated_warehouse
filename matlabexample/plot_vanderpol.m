%--------------- Plot Example
subplot(3,1,1:2)
fill(xlim(:,1),xlim(:,2),[0.8 0.8 0.2])
th = linspace(-pi,pi,100);
hold on
ell = P^(-1/2)*[cos(th) ; sin(th)];
fill(ell(1,:),ell(2,:),0.9*ones(1,3))

for i = N-1:-1:1
    St = double(reshape(subs(Pi(:,i),t,0),n,n));
    xt = double(subs(x0i(:,i),t,0));
    ell = (St./ppval(rhopp,ts(i)))^(-1/2)*[cos(th) ; sin(th)];
    fill(xt(1)+ell(1,:),xt(2)+ell(2,:),0.8*ones(1,3))
end

plot(x0s(:,1),x0s(:,2))
hold off
subplot(3,1,3)
plot(ts,ppval(rhopp,ts),'-o')
ylim([0 1])
drawnow
%------------------ End Plotting