function [ st, sq ] = spline_solver_nonic( n, st0, stf, q0, qd0, qdd0, qddd0, qdddd0, qf, qdf, qddf, qdddf, qddddf )
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
    t0 = 0;
    tf = 1;
    dt = stf - st0;
    
%     M = [ 1, t0, t0^2,   t0^3,    t0^4,    t0^5,     t0^6,     t0^7,      t0^8,      t0^9;
%           0,  1, 2*t0, 3*t0^2,  4*t0^3,  5*t0^4,   6*t0^5,   7*t0^6,    8*t0^7,    9*t0^8;
%           0,  0,    2,   6*t0, 12*t0^2, 20*t0^3,  30*t0^4,  42*t0^5,   56*t0^6,   72*t0^7;
%           0,  0,    0,      6,   24*t0, 60*t0^2, 120*t0^3, 210*t0^4,  336*t0^5,  504*t0^6;
%           0,  0,    0,      0,      24,  120*t0, 360*t0^2, 840*t0^3, 1680*t0^4, 3024*t0^5;
%           1, tf, tf^2,   tf^3,    tf^4,    tf^5,     tf^6,     tf^7,      tf^8,      tf^9;
%           0,  1, 2*tf, 3*tf^2,  4*tf^3,  5*tf^4,   6*tf^5,   7*tf^6,    8*tf^7,    9*tf^8;
%           0,  0,    2,   6*tf, 12*tf^2, 20*tf^3,  30*tf^4,  42*tf^5,   56*tf^6,   72*tf^7;
%           0,  0,    0,      6,   24*tf, 60*tf^2, 120*tf^3, 210*tf^4,  336*tf^5,  504*tf^6;
%           0,  0,    0,      0,      24,  120*tf, 360*tf^2, 840*tf^3, 1680*tf^4, 3024*tf^5];

    %% Coefficient Solver
    
    b=[q0; qd0*dt; qdd0*(dt^2); qddd0*(dt^3); qdddd0*(dt^4); qf; qdf*dt; qddf*(dt^2); qdddf*(dt^3); qddddf*(dt^4)];
    M = spline_solver_gen_tnorm_ls( length(b) );
    a = inv(M)*b;
    

    %% Spline Calculator
    
    t = linspace(t0, tf, n);
    st = linspace(st0, stf, n);
    c = ones(size(t));
    
    q =        a(1).*c +     a(2).*t +     a(3).*t.^2 +     a(4).*t.^3 +      a(5).*t.^4 +       a(6).*t.^5 +      a(7).*t.^6 +     a(8).*t.^7 +    a(9).*t.^8 + a(10).*t.^9;
    qd =       a(2).*c +   2*a(3).*t +   3*a(4).*t.^2 +   4*a(5).*t.^3 +    5*a(6).*t.^4 +     6*a(7).*t.^5 +    7*a(8).*t.^6 +   8*a(9).*t.^7 + 9*a(10).*t.^8;
    qdd =    2*a(3).*c +   6*a(4).*t +  12*a(5).*t.^2 +  20*a(6).*t.^3 +   30*a(7).*t.^4 +    42*a(8).*t.^5 +   56*a(9).*t.^6 + 72*a(10).*t.^7;
    qddd =   6*a(4).*c +  24*a(5).*t +  60*a(6).*t.^2 + 120*a(7).*t.^3 +  210*a(8).*t.^4 +   336*a(9).*t.^5 + 504*a(10).*t.^6;
    qdddd = 24*a(5).*c + 120*a(6).*t + 360*a(7).*t.^2 + 840*a(8).*t.^3 + 1680*a(9).*t.^4 + 3024*a(10).*t.^5;
    
    % Devide derivatives by dt 
    sq = [q; qd./dt; qdd./(dt^2); qddd./(dt^3); qdddd./(dt^4)];    
end

