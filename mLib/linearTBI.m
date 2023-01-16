% @input  p_start  [x;y;z;e]   start point
% @input  p_end    [x;y;z;e]   end point
% @input  fs       [mm/s]      desired start feedrate (in final trajectory may differs a little)
% @input  fe       [mm/s]      desired end feedrate (in final trajectory may differs a little)
% @input  feedrate [mm/s]      desired feedrate at constant speed (in final trajectory may differs a little)
% @input  accel    [mm/s^2]    desired acceleration.
% @input  Ts       [sec]       sampling time.
% @return path     [4*N array] interpolated path
%         fe       [mm/s]      end feedrate (for next TBI) 
function [path,fe,L] = linearTBI(p_start,p_end,fs,fe,feedrate,accel,Ts)
    p_diff = p_end-p_start;
    L      = norm(p_diff(1:3));
    if(norm(L)==0)
        % retract, only moves e. or z move. Need to be slower.
        L = norm(p_diff);
        accel = 100;
        feedrate = 10;
    end
    dir    = p_diff/L;

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

    feedrate_new = 1/(T1_new+2*T2_new+T3_new)*(2*L-fs*T1_new-fe*T3_new)*dir;
    accel_new = (norm(feedrate_new(1:3))-fs)/T1*dir;
    % if T1 == 0; fs == feedrate, calculate accel for fe;
    if(ismember(inf,accel_new)||ismember(-inf,accel_new))
        accel_new = (feedrate_new-fe*dir)/T3_new;
    end
    % if still accel == inf, T3 == 0; whole path constant speed. accel = 0; 
    if(ismember(inf,accel_new)||ismember(-inf,accel_new))
        accel_new = 0*dir;
    end
    if(N1~=0)
        p1 = p_start+1/2*accel_new*((0:N1)*Ts).^2+dir*fs*(0:N1)*Ts;
    else
        p1 = [];
    end
    if(N2~=0)
        if(~isempty(p1))
            p2 = p1(:,end)+feedrate_new*(1:N2)*Ts;
        else
            p2 = p_start+feedrate_new*(1:N2)*Ts;
        end
    else
        p2 = [];
    end
    if(N3~=0)
        if(N2==0)
            if(N1==0)
                p3 = p_start+feedrate_new*(1:N3)*Ts-1/2*accel_new*((1:N3)*Ts).^2;
            else
                p3 = p1(:,end)+feedrate_new*(1:N3)*Ts-1/2*accel_new*((1:N3)*Ts).^2;
            end
        else
            p3 = p2(:,end)+feedrate_new*(1:N3)*Ts-1/2*accel_new*((1:N3)*Ts).^2;
        end
    else
        p3 = [];
    end
    path=[p1,p2,p3];
end