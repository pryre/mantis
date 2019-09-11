function [ R, w, wd ] = map_angles_rates_accels( a, ad, add, psi, psid, psidd )
%UNTITLED3 Summary of this function goes here
%   a:   Acceleration
%   ad:  Jerk
%   add: Snap

    z_w = [0;0;1]; % u_1 = m||t||

    % Derivative Variables
    b = norm(a);
    adot = dot(a,ad);
    bd = adot/b;
    bdd = -adot/(b^2) + (adot + dot(a,add))/b;
%     bdd = (-(adot)^2)/(b^3) + (adot + dot(a,add))/b;
%     bdd = -(1/b)*(bdot^2) + (adot + dot(a,add))/b;

    z_b = a/b; % Eq. 6
    x_c = [cos(psi); sin(psi); 0];

    y_b_dir = cross(z_b,x_c);
    y_b = y_b_dir/norm(y_b_dir);

    x_b = cross(y_b,z_b);

    R = [x_b, y_b, z_b]; % Eq. 6+3

    % Rates
    h_w = 1/b*(ad  - bd*z_b);
%     h_w = (1/a_n)*(ad - (dot(z_b,ad))*z_b) % Eq. 7+1

    p = -dot(h_w,y_b);
    q = dot(h_w,x_b);

    w_cw = psid*z_w;
    r = dot(w_cw,z_b);

    w = [p; q; r];

    w_bw = R*w;
%     w_bc = w_bw - w_cw; %XXX: Only needed to prove that wd_cw is needed.

    % Accels
    h_wd = (1/b)*(add - bdd*z_b - 2*bd*h_w - b*cross(w_bw,h_w));
    pd = -dot(h_wd,y_b);
    qd = dot(h_wd,x_b);

    wd_cw = psidd*z_w;
    rd = dot(wd_cw,z_b);

    wd = [pd; qd; rd];

end









