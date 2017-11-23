function [ dy ] = multirotor_run( t, y, v, Dq, Cqqd, phi, vars, tspan )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    disp([num2str(100*(t(end) - tspan(1)) / (tspan(2) - tspan(1))), '%'])

    qd = [vars(2);vars(4);vars(6);vars(8);vars(10);vars(12)];

    u = Dq*v + Cqqd*qd + phi;
    u_sub = double(subs(u, vars', y'));

    accel = -(inv(Dq)*(Cqqd*qd + phi)) + inv(Dq)*u_sub; %The acceleration this timestep
    accel_sub = double(subs(accel, vars', y')); %Extra zeros to sub in for acceleration terms

    dy = zeros(12,1);
    dy(1) = y(2);
    dy(2) = accel_sub(1);
    dy(3) = y(4);
    dy(4) = accel_sub(2);
    dy(5) = y(6);
    dy(6) = accel_sub(3);
    dy(7) = y(8);
    dy(8) = accel_sub(4);
    dy(9) = y(10);
    dy(10) = accel_sub(5);
    dy(11) = y(12);
    dy(12) = accel_sub(6);
end

