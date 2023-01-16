function [path] = circularTBI(p_c,R,fila_l,theta_start,theta_end,fs,fe,accelx,accely,feedrate,Ts)
    theta_diff = theta_end - theta_start;
    theta = abs(theta_diff);
    omega = feedrate/R;
    alpha = min(accelx,accely)/R;
    omegas = fs/R;
    omegae = fe/R;
    T2 = 1/omega*(theta-1/alpha*omega^2+(omegae^2+omegas^2)/2/alpha);
    if(T2<0)
        omega = sqrt(alpha*(theta+omegae^2/2/alpha+omegas^2/2/alpha));
        T2 = 1/omega*(theta-1/alpha*omega^2+(omegae^2+omegas^2)/2/alpha);
    end
    T1 = (omega-omegas)/alpha;
    T3 = (omega-omegae)/alpha;
    
    N1 = ceil(T1/Ts);
    N2 = ceil(T2/Ts);
    N3 = ceil(T3/Ts);

    T1_new = N1*Ts;
    T2_new = N2*Ts;
    T3_new = N3*Ts;

    omega_new = 1/(T1_new+2*T2_new+T3_new)*(2*theta-omegas*T1_new-omegae*T3_new)*sign(theta_diff);
    alpha_new = omega_new/T1;
    theta1    = theta_start+1/2*alpha_new*((0:N1)*Ts).^2;
    theta2    = theta1(end)+omega_new*(1:N2)*Ts;
    theta3    = theta2(end)+omega_new*(1:N3)*Ts-1/2*alpha_new*((1:N3)*Ts).^2;
    theta = [theta1,theta2,theta3];
    pathx     = p_c(1)+R*cos(theta);
    pathy     = p_c(2)+R*sin(theta);
    pathz     = repelem(p_c(3),length(pathx));
    pathe     = theta/(theta(end)-theta(1))*fila_l;
    path = [pathx;pathy;pathz;pathe];
end

