p0 = [0 0 0 0]';
p1 = [1,0 0 0]';
accel = 500;
vel = 60;
Ts = 1e-3;
fs = 0;
fe = 60;
path = linearTBI(p0,p1,fs,fe,vel,accel,Ts);
plot([p0(1) p1(1)],[p0(2),p1(2)],'rx')
hold on
plot(path(1,:),path(2,:),'b-')
vel = diff(path,1,2);
figure(2)
plot(vecnorm(vel))