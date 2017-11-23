function [ dy ] = mantis_run_pitch_only( t, y, u, Dq, Cqqd, phi, vars, tspan )
%MANTIS_RUN Runs a simulation for the Mantis MM-UAV
%   y: Current state of the MM-UAV
%   u: Control input
%   p: Parameter structure as loaded by mantis_params()
%   
%   dy: The new state of the MM-UAV


%% Example

%function dy = cartpend(y,m,M,L,g,d,u)
%
%Sy = sin(y(3));
%Cy = cos(y(3));
%D = m*L*L*(M+m*(1-Cy^2));

%dy(1,1) = y(2);
%dy(2,1) = (1/D)*(-m^2*L^2*g*Cy*Sy + m*L^2*(m*L*y(4)^2*Sy - d*y(2))) + m*L*L*(1/D)*u;
%dy(3,1) = y(4);
%dy(4,1) = (1/D)*((m+M)*m*g*L*Sy - m*L*Cy*(m*L*y(4)^2*Sy - d*y(2))) - m*L*Cy*(1/D)*u +.01*randn;


%%

%xd1 = x2;
%xd2 = -(inv(Dq(1,:))*(Cqqd(1,:)*x2+phi(1))) + inv(Dq(1,:))*u(1);
%xd3 = x4;
%xd4 = -(inv(Dq(2,:))*(Cqqd(2,:)*x4+phi(2))) + inv(Dq(2,:))*u(2);
%xd5 = x6;
%xd6 = -(inv(Dq(3,:))*(Cqqd(3,:)*x6+phi(3))) + inv(Dq(3,:))*u(3);

disp([num2str(100*(t(end) - tspan(1)) / (tspan(2) - tspan(1))), '%'])

vels = [y(2);y(4);y(6)];
motor_damping = 0.1*[0,0,0;0,1,0;0,0,1];

plant = -(inv(Dq)*(Cqqd*vels + motor_damping*vels + phi)) + inv(Dq)*u;
plant_sub = double(subs(plant, vars', [y', 0, 0, 0]));

dy = zeros(6,1);
dy(1) = y(2);
dy(2) = plant_sub(1);
dy(3) = y(4);
dy(4) = plant_sub(2);
dy(5) = y(6);
dy(6) = plant_sub(3);


end