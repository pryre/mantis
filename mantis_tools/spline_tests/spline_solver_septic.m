function [ a ] = spline_solver_septic( dt, q0, qd0, qdd0, qddd0, qf, qdf, qddf, qdddf )
%QUINTIC_SPLINE_SOLVER Computes a septic polynomial reference trajectory
%   Primary use of this solution is to provide continous acceleration (and 
%   therefore a non-impulsive jerk), which should not induce vibrational 
%   modes during operation.
%
%   There are 6 constraints (3 initial, 3 final), therefore a 5th order
%   polynomial is required (this equation is derrived to get the equations 
%   needed for the matrix representation):
%       q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 + a6*t^5 + a7*t^5
%
%   Inputs:
%          n: Number of points to calculate
%         t0: Initial time
%         tf: Final time
%         q0: Initial position
%        qd0: Initial velocity
%       qdd0: Initial acceleration
%      qddd0: Initial Jerk
%         qf: Final position
%        qdf: Final position
%       qddf: Final acceleration
%      qdddf: Final Jerk


    %% Simultaneous Polynomial Equations
    % Use a normalized time to build the matrix
    % This helps ensure that we stay away from floating point issues
%     t0 = 0;
%     tf = 1;
%     dt = stf - st0;
    
%     M = [ 1, t0, t0^2,   t0^3,    t0^4,    t0^5,     t0^6,     t0^7;
%           0,  1, 2*t0, 3*t0^2,  4*t0^3,  5*t0^4,   6*t0^5,   7*t0^6;
%           0,  0,    2,   6*t0, 12*t0^2, 20*t0^3,  30*t0^4,  42*t0^5;
%           0,  0,    0,      6,   24*t0, 60*t0^2, 120*t0^3, 210*t0^4;
%           1, tf, tf^2,   tf^3,    tf^4,    tf^5,     tf^6,     tf^7;
%           0,  1, 2*tf, 3*tf^2,  4*tf^3,  5*tf^4,   6*tf^5,   7*tf^6;
%           0,  0,    2,   6*tf, 12*tf^2, 20*tf^3,  30*tf^4,  42*tf^5;
%           0,  0,    0,      6,   24*tf, 60*tf^2, 120*tf^3, 210*tf^4];


    %% Coefficient Solver
    
    % Normalized solver
%     dt = 1;
    
    b=[q0; qd0*dt; qdd0*(dt^2); qddd0*(dt^3); qf; qdf*dt; qddf*(dt^2); qdddf*(dt^3)];
    M = spline_solver_gen_tnorm_ls( length(b) );
    a = inv(M)*b;
    
end

