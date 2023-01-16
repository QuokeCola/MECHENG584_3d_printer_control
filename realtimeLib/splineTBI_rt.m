% @input  p0       [4*N array] spline points in [x;y;z;e], z values should be same!
% @input  fs       [mm/s]      desired start feedrate (in final trajectory may differs a little)
% @input  fe       [mm/s]      desired end feedrate (in final trajectory may differs a little)
% @input  feedrate [mm/s]      desired feedrate at constant speed (in final trajectory may differs a little)
% @input  accel    [mm/s^2]    desired acceleration.
% @input  Ts       [sec]       sampling time.
% @input  L        [mm]        length of trajectory piece measured by splineTBI
% @return path     [4*N array] interpolated path
%         fe       [mm/s]      end feedrate (for next TBI) 
function [path,finished,N] = splineTBI_rt(pset,fs,fe,feedrate_raw,accel,Ts,L,query_N)
    feedrate = feedrate_raw;
    % interpolate points wrt time to get more accurate path (length).
    p_diffs_1 = pset(:,2:end)-pset(:,1:end-1);
    timestamp_T_1 = [0,cumsum(vecnorm(p_diffs_1(1:2,:)))/feedrate];
    % get fit spline traj and precise L
    p_interp_f = [spline(timestamp_T_1,pset(1,:));spline(timestamp_T_1,pset(2,:));spline(timestamp_T_1,pset(3,:));spline(timestamp_T_1,pset(4,:))];
    % TBI along path with trapzoidal profile (get time profile)
    T2 = 1/feedrate*(L-1/accel*feedrate^2+(fe^2+fs^2)/2/accel);
    if(T2<0)
        fe = 0;
        feedrate = sqrt(accel*(L+fe^2/2/accel+fs^2/2/accel));
        if(fs>feedrate)
            fs = feedrate;
        end
        if(fe>feedrate)
            fe = feedrate;
        end
        T2 = 1/feedrate*(L-1/accel*feedrate^2+(fe^2+fs^2)/2/accel);
    end
    T1 = (feedrate-fs)/accel;
    T3 = (feedrate-fe)/accel;

    N1 = ceil(T1/Ts);
    N2 = ceil(T2/Ts);
    N3 = ceil(T3/Ts);

    T1_new = N1*Ts;
    T2_new = N2*Ts;
    T3_new = N3*Ts;

    feedrate_new = 1/(T1_new+2*T2_new+T3_new)*(2*L-fs*T1_new-fe*T3_new);
    accel_new = (feedrate_new-fs)/T1_new;
    % if T1 == 0; fs == feedrate, calculate accel for fe;
    if(accel_new == inf||accel_new == -inf)
        accel_new = (feedrate_new-fe)/T3_new;
    end
    % if still accel == inf, T3 == 0; whole path constant speed. accel = 0; 
    if(accel_new == inf||accel_new == -inf)
        accel_new = 0;
    end
    s1_start = 0;
    if(N1~=0)
        %accel stage exists
        s2_start = 1/2*accel_new*(N1*Ts)^2+fs*N1*Ts;
    else
        s2_start = s1_start;
    end
    if(N2 ~= 0)
        s3_start = s2_start+feedrate_new*(N2*Ts);
    else
        s3_start = s2_start;
    end
    if(query_N<=N1)
        curr_s = s1_start+1/2*accel_new*(query_N*Ts)^2+fs*query_N*Ts;
    elseif(query_N<=N1+N2)
        curr_s = s2_start+feedrate_new*(query_N-N1)*Ts;
    elseif(query_N<=N1+N2+N3)
        curr_s = s3_start+feedrate_new*(query_N-N1-N2)*Ts-1/2*accel_new*((query_N-N1-N2)*Ts)^2;
    else
        curr_s = L;
    end

    % get the time that it takes to get to resire length
    query_t_2 = curr_s/L*timestamp_T_1(end);
    % query the corresponding position
    % interpolate extrude
    extrude = (curr_s/L)*(pset(4,end)-pset(4,1))+pset(4,1);
    path = [ppval(p_interp_f(1),query_t_2);ppval(p_interp_f(2),query_t_2);pset(3,1);extrude];
    finished = query_N>=N1+N2+N3-1;
    N = N1+N2+N3-1;
end