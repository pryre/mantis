function [ config ] = gen_config(t0, dt, cdt, tf, frame_type, manip_type, n, order, num_vias, dvia_est_method, tname_base, tname_r, control_method, control_fully_actuated, tracking_frame, yaw_w, theta_max, w0p, w0t, w0r)
%GEN_CONFIG Helper function to generate user configs
%   Time Settings: 
%       - t0: Start time
%       - dt: Simulation time step
%       - cdt: Controller time step (>dt)
%       - tf: End time
%   Model Properties:
%       - frame_type: Frame type of the multirotor base
%           - 'quad'
%           - 'hex'
%       - manip_type: Manipulator type
%           - 'serial'
%       - n: Number of maniplator links/joints
%   Spline Settings:
%       - order:
%           - cubic = 3
%           - quntic = 5
%           - septic = 7
%           - nonic = 9
%       - num_vias: Number of vias to use during generation
%       - dvia_est_method: Estimation method for via derivatives
%       - tname_base: Trajectory name for the base to track
%           - 'hover'
%           - 'hover2'
%           - 'x_only'
%           - 'y_only'
%           - 'z_only'
%           - 'yaw_only'
%           - 'circle_flat'
%           - 'circle_flat_yaw'
%           - 'circle_raised'
%           - 'circle_raised_yaw'
%       - tname_r: Trajectory name for each joint to track (cell array (n))
%           - 'steady_0' - Joint angles 0
%           - 'steady_90' - Joint angles 0
%           - 'swing_part' - Joint angles 0->pi/4
%           - 'swing_half' - Joint angles 0->pi/2
%           - 'swing_full' - Joint angles -pi/2->pi/2
%   Control Settings:
%       - control_method: Control method to use
%           - 'npid_px4' - Nonlinear PID (Regulating, PX4 Structure)
%           - 'npid_exp' - Nonlinear PID (Regulating, Expanded PX4 Structure)
%           - 'npid'     - Nonlinear PID (Tracking)
%           - 'ctc'      - Computed Torque Control (Tracking)
%           - 'feed'     - Feed-Forward Compensation (Tracking)
%       - control_fully_actuated: Allows the platform to actuate in all directions if >0
%       - tracking_frame: Frame of refernce for the controller to use during tracking
%           - 0 for tracking the base
%           - n for tracking joint n
%           - n+1 for tracking at the end effector
%       - yaw_w: Weighting for yaw-ing actuation
%       - theta_max: Maximum tilt angle to be tracked
%    Tuning Parameters:
%       - w0p: Natural frequency for tuning position control
%       - w0t: Natural frequency for tuning attitude control
%       - w0r: Natural frequency for tuning joint control

    if cdt <= dt
        error('cdt > dt')
    end
    
    config.g_vec = [0;0;-9.80665]; % TODO: Make a parameter of this func.
    config.g = norm(config.g_vec);

    % Settings
    % Timings
    config.time.t0 = t0;
    config.time.dt = dt;
    config.time.cdt = cdt;
    config.time.tf = tf;

    % Model
    config.model.frame_type = frame_type;
    config.model.manip_type = manip_type;
    config.model.n = n;
    
    % Spline
    config.spline.order = order;
    config.spline.num_vias = num_vias;
    config.spline.dvia_est_method = dvia_est_method;
    config.spline.tname_base = tname_base;
    config.spline.tname_r = tname_r;

    % Control
    config.control.method = control_method;
    config.control.fully_actuated = control_fully_actuated;
    config.control.tracking_frame = tracking_frame;
    config.control.yaw_w = yaw_w;    % Yaw weighting for rotational tracking
    config.control.theta_max = theta_max; % Maximum thrust vectoring angle (from vertical)

    % Tuning
    config.tuning.w0p = w0p;
    config.tuning.w0t = w0t;
    config.tuning.w0r = w0r;
    
    % Display for logging
    print_config(config);
end

