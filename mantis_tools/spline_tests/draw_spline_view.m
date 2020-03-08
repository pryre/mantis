% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

%% Setup

close all;
clear;
clc;

set(0,'defaulttextInterpreter','latex');


%%

t0 = 0;
dt = 1/500;
tf = 10;

vp = [0,-1,2,1,0,0];
via_est_method = 'fdcc';


%%

num_splines = 4;
num_via_points = length(vp);

t = t0:dt:tf;
t_vias = linspace(t0,tf,num_via_points);
% sd = t_vias(2) - t_vias(1);

vias = gen_vias(via_est_method, vp, t_vias);
% via_steps = ((tf - t0) / dt) / (num_via_points - 1);

[~, s3] = gen_spline(t_vias, vias, 3, dt);
[~, s5] = gen_spline(t_vias, vias, 5, dt);
[~, s7] = gen_spline(t_vias, vias, 7, dt);
[~, s9] = gen_spline(t_vias, vias, 9, dt);


%%

f1 = draw_spline_view_figure( t, s3, t_vias, vias, 3, '\(\mathbf{3^{rd}}\) \textbf{Order Polynomial Spline (Cubic)}' );
f2 = draw_spline_view_figure( t, s5, t_vias, vias, 5, '\(\mathbf{5^{th}}\) \textbf{Order Polynomial Spline (Quintic)}' );
f3 = draw_spline_view_figure( t, s7, t_vias, vias, 7, '\(\mathbf{7^{th}}\) \textbf{Order Polynomial Spline (Septic)}' );
f4 = draw_spline_view_figure( t, s9, t_vias, vias, 9, '\(\mathbf{9^{th}}\) \textbf{Order Polynomial Spline (Nonic)}' );


%%
print(f1, './figures/spline_cubic', '-depsc')
print(f2, './figures/spline_quintic', '-depsc')
print(f3, './figures/spline_septic', '-depsc')
print(f4, './figures/spline_nonic', '-depsc')

%%
print(f1, './figures/spline_cubic.png', '-dpng', '-r720')
print(f2, './figures/spline_quintic.png', '-dpng', '-r720')
print(f3, './figures/spline_septic.png', '-dpng', '-r720')
print(f4, './figures/spline_nonic.png', '-dpng', '-r720')


