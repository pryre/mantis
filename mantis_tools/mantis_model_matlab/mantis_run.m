function [ dy ] = mantis_run( t, y, v, vel_body_rot, Dq_eq, Cqqd_eq, N_eq, tspan, do_control )
%MANTIS_RUN Runs a simulation for the Mantis MM-UAV
%   Description
    
    disp([num2str(100*(t(end) - tspan(1)) / (tspan(2) - tspan(1))), '%'])
    
    qd = [y(2); y(4); y(6); y(8); y(10); y(12); y(14); y(16)];
    
    
    Dq = zeros(size(Dq_eq));
    Cqqd = zeros(size(Cqqd_eq));
    N = zeros(size(N_eq));

    for i = 1:numel(Dq_eq)
        Dq(i) = Dq_eq{i}(y(1),y(2),y(3),y(4),y(5),y(6),y(7),y(8),y(9),y(10),y(11),y(12),y(13),y(14),y(15),y(16));
    end

    for i = 1:numel(Cqqd)
        Cqqd(i) = Cqqd_eq{i}(y(1),y(2),y(3),y(4),y(5),y(6),y(7),y(8),y(9),y(10),y(11),y(12),y(13),y(14),y(15),y(16));
    end
   
    for i = 1:numel(N)
        N(i) = N_eq{i}(y(1),y(2),y(3),y(4),y(5),y(6),y(7),y(8),y(9),y(10),y(11),y(12),y(13),y(14),y(15),y(16));
    end
    
    if do_control == 1
        error('Not just yet!')
        %u = Dq*v + (Cqqd + Kqd)*qd + phi;
    else
        u = zeros(8,1);
    end
    
    %accel = -(inv(Dq)*(Cqqd*qd + Kqd*qd + phi)) + inv(Dq)*u_sub;
    accel = -(Dq\((Cqqd)*qd + N)) + Dq\u;

    %dy(1) = y(2);
    %dy(2) = accel(1);
    %dy(3) = y(4);
    %dy(4) = accel(2);
    
    dy = [qd'; accel'];
    dy = dy(:);
    
end