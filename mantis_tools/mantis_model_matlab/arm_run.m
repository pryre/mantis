function [ dy ] = arm_run( t, y, v, Dq, Cqqd, Kqd, phi, vars, tspan, do_control )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    disp([num2str(100*(t(end) - tspan(1)) / (tspan(2) - tspan(1))), '%'])

    qd = [vars(2);vars(4)];

    if do_control == 1
        u = Dq*v + (Cqqd + Kqd)*qd + phi;
    else
        u = zeros(2,1);
    end
    
    u_sub = double(subs(u, vars', y'));

    accel = -(inv(Dq)*(Cqqd*qd + Kqd*qd + phi)) + inv(Dq)*u_sub; %The acceleration this timestep
    accel_sub = double(subs(accel, vars', y')); %Extra zeros to sub in for acceleration terms

    dy = zeros(4,1);
    dy(1) = y(2);
    dy(2) = accel_sub(1);
    dy(3) = y(4);
    dy(4) = accel_sub(2);


end

