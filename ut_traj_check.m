close all;
load('traj_test')
x_ref = out.x.data(2:end);
y_ref = out.y.data(2:end);
z_ref = out.e.data(2:end);
vx_ref = diff(out.x.data(2:end))/1e-3;
ax_ref = diff(vx_ref)/1e-3;
vy_ref = diff(out.y.data(2:end))/1e-3;
ay_ref = diff(vy_ref)/1e-3;
vz_ref = diff(z_ref)/1e-3;
az_ref = diff(vz_ref)/1e-3;
figure(1)
subplot(311)
plot(z_ref)
subplot(312)
plot(vz_ref)
subplot(313)
plot(az_ref)

query_point =16121;
t_range_1 = (query_point-500):query_point;
t_range_2 = (query_point+1):(query_point+50);
figure(2)
plot(x_ref,y_ref)
hold on 
plot(x_ref(t_range_2),y_ref(t_range_2),'Color','magenta','LineWidth',1.5)
plot(x_ref(t_range_1),y_ref(t_range_1),'g-')
plot(x_ref(t_range_1(1)),y_ref(t_range_1(1)),'bo')
plot(x_ref(query_point-1),y_ref(query_point-1),'ro')
plot(x_ref(query_point),y_ref(query_point),'rx')
plot(x_ref(t_range_2(end)),y_ref(t_range_2(end)),'bx')
