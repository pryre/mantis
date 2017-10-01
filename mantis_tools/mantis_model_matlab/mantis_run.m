function [ dy ] = mantis_run( y, u, p )
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



end