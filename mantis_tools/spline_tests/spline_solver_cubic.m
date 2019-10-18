function [ a ] = spline_solver_cubic( dt, q0, qd0, qf, qdf )
%QUINTIC_SPLINE_SOLVER Computes a quintic polynomial reference trajectory
%   Primary use of this solution is to provide continous acceleration (and 
%   therefore a non-impulsive jerk), which should not induce vibrational 
%   modes during operation.
%
%   There are 6 constraints (3 initial, 3 final), therefore a 5th order
%   polynomial is required (this equation is derrived to get the equations 
%   needed for the matrix representation):
%       q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
%
%   Inputs:
%          n: Number of points to calculate
%         t0: Initial time
%         tf: Final time
%         q0: Initial position
%        qd0: Initial velocity
%       qdd0: Initial acceleration
%         qf: Final position
%        qdf: Final position
%       qddf: Final acceleration


    %% Simultaneous Polynomial Equations
    % Use a normalized time to build the matrix
    % This helps ensure that we stay away from floating point issues
%     t0 = 0;
%     tf = 1;
%     dt = stf - st0;
    
%     M = [ 1, t0, t0^2,   t0^3;
%           0,  1, 2*t0, 3*t0^2;
%           1, tf, tf^2,   tf^3;
%           0,  1, 2*tf, 3*tf^2];


    %% Coefficient Solver
    
    % Normalized solver
%     sdt = 1;
   
    b=[q0; qd0*dt; qf; qdf*dt];
    M = spline_solver_gen_tnorm_ls( length(b) );
    a = inv(M)*b;

end

