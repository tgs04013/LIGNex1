function X = pathPlan(Ts,Tf)
    X(:,1) = [10000;0;0;120];

    t = 0 : Ts : Tf;
    sample_size = size(t,2);

    for i = 1:sample_size-1
        if (i*Ts < 50)
            Omega = 0;
        elseif i*Ts < 185
            Omega = deg2rad(1);
        elseif i*Ts < 400
            Omega = 0;
        elseif i*Ts < 445
            Omega = deg2rad(3);
        else
            Omega = 0;
        end
        
        Omega = Omega + 0.000000000000000000000000000001;

        F = [1 sin(Omega*Ts)/Omega     0 -(1-cos(Omega*Ts))/Omega;
             0 cos(Omega*Ts)           0 -sin(Omega*Ts);
             0 (1-cos(Omega*Ts))/Omega 1 sin(Omega*Ts)/Omega;
             0 sin(Omega*Ts)           0 cos(Omega*Ts) ];

        X(:,i+1) = F * X(:,i);
    end
end