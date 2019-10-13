function [ configs, x0_overrides ] = gen_test_cases( test_name )
%GEN_TEST_CASES Helper function to generate preset test cases
% test_name:
%   - 'hover_0_0':         (arm downward)
%   - 'hover_90_0':        (arm forward)
%   - 'full_state_error':  ([1,1,0;pi/2] -> [0,0,1,0], arm down -> arm up)
%   - 'inversion_0_0':     Inversion recovery, roll axis return to hover (arm downward)
%   - 'inversion_90_0':    Inversion recovery, roll axis return to hover (arm forward)
%   - 'tuning_converging': Tuning test to demonstrate different error dynamics (converging)
%   - 'tuning_diverging':  Tuning test to demonstrate different error dynamics (converging)
%   - 'spiral_base_slow':  Spiral trajectory for robot base (dt=40)
%   - 'spiral_base_quick': Spiral trajectory for robot base (dt=10)
%   - 'spiral_end_slow':   Spiral trajectory for end effector (dt=40)
%   - 'spiral_end_quick':  Spiral trajectory for end effector (dt=10)
%   - 'wind_compensation': Demonstrates the use of the position tracking integrator

    % Bad starting states:
    % x0(sn.STATE_Q) = eul2quat([pi/2,0,0])';    % Half yaw error rotation (World)
    % x0(sn.STATE_Q) = eul2quat([0,0,deg2rad(179)])';    % (Almost) Full roll error rotation (World)
    % x0(sn.STATE_XYZ) = [1;1;0];    % Positional Error (World)
    % x0(sn.STATE_R) = zeros(n,1);   % Arm down Joint Error (World)
    % x0_overrides(1,:) = {'STATE_Q', eul2quat([pi/2,0,0])'};

    configs = cell(0,1);
    x0_overrides = cell(0,2);
    
    default_config = gen_config(0, 1/1000, 1/250, 30, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'hover', {'steady_0';'steady_0'}, ...
                                'npid_px4', 0, 0, 0, 0.6, deg2rad(30), ...
                                1, 15, 4);

    if strcmp(test_name, 'hover_0_0')
        configs{1} = default_config;
        
        configs(2:5,1) = configs(1);
        configs{2}.control.method = 'ctc';
        configs{2}.control.fully_actuated = 1;
        configs{3}.control.method = 'ctc';
        configs{3}.control.fully_actuated = 0;
        configs{4}.control.method = 'feed';
        configs{4}.control.fully_actuated = 1;
        configs{5}.control.method = 'feed';
        configs{5}.control.fully_actuated = 0;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'hover_90_0')
        configs{1} = default_config;
        configs{1}.tname_r{1} = 'steady_90';
        
        configs(2:5,1) = configs(1);
        configs{2}.control.method = 'ctc';
        configs{2}.control.fully_actuated = 1;
        configs{3}.control.method = 'ctc';
        configs{3}.control.fully_actuated = 0;
        configs{4}.control.method = 'feed';
        configs{4}.control.fully_actuated = 1;
        configs{5}.control.method = 'feed';
        configs{5}.control.fully_actuated = 0;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'full_state_error')
        x0_overrides(1,:) = {'STATE_Q', eul2quat([deg2rad(90),0,0])'};
        x0_overrides(2,:) = {'STATE_XYZ', [1;1;0]};
        x0_overrides(3,:) = {'STATE_R', [0;0]};
        
        configs{1} = default_config;
        configs{1}.tname_r{1} = 'steady_90';
        configs{1}.tname_r{2} = 'steady_90';
        
        configs(2:5,1) = configs(1);
        configs{2}.control.method = 'ctc';
        configs{2}.control.fully_actuated = 1;
        configs{3}.control.method = 'ctc';
        configs{3}.control.fully_actuated = 0;
        configs{4}.control.method = 'feed';
        configs{4}.control.fully_actuated = 1;
        configs{5}.control.method = 'feed';
        configs{5}.control.fully_actuated = 0;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'inversion_0_0')
        x0_overrides(1,:) = {'STATE_Q', eul2quat([0,0,deg2rad(179)])'};
        x0_overrides(2,:) = {'STATE_XYZ', [0;0;2]};
        
        configs{1} = default_config;
        configs{1}.tname_base = 'hover2';
        
        configs(2:5,1) = configs(1);
        configs{2}.control.method = 'ctc';
        configs{2}.control.fully_actuated = 1;
        configs{3}.control.method = 'ctc';
        configs{3}.control.fully_actuated = 0;
        configs{4}.control.method = 'feed';
        configs{4}.control.fully_actuated = 1;
        configs{5}.control.method = 'feed';
        configs{5}.control.fully_actuated = 0;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'inversion_90_0')
        x0_overrides(1,:) = {'STATE_Q', eul2quat([0,0,deg2rad(179)])'};
        x0_overrides(2,:) = {'STATE_XYZ', [0;0;2]};
        
        configs{1} = default_config;
        configs{1}.tname_base = 'hover2';
        configs{1}.tname_r{1} = 'steady_90';
        
        configs(2:5,1) = configs(1);
        configs{2}.control.method = 'ctc';
        configs{2}.control.fully_actuated = 1;
        configs{3}.control.method = 'ctc';
        configs{3}.control.fully_actuated = 0;
        configs{4}.control.method = 'feed';
        configs{4}.control.fully_actuated = 1;
        configs{5}.control.method = 'feed';
        configs{5}.control.fully_actuated = 0;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'tuning_converging')
        % This arrangement of gain frequencies gives a response of a
        % critically damped system for both position and attitude tracking.
        % w0r is set to be an order of magnitude faster in response than
        % w0p, which seems to give good results (but I'm not 100% sure why,
        % frequency responses mixing? Maybe a good reference would be
        % similar to a cascade controller timings - should be 5-20x more on
        % who you ask)
        %     w0p = 2;
        %     w0t = 10*w0p;
        %     w0r = 4;
        x0_overrides(1,:) = {'STATE_Q', eul2quat([0,0,deg2rad(179)])'};
        
        configs{1} = default_config;
        configs{1}.control.method = 'ctc';
        configs{1}.control.fully_actuated = 1;
        
        configs(2:3,1) = configs(1);
        configs{2}.tuning.w0p = 2;
        configs{3}.tuning.w0p = 5;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'tuning_oscillating')
        x0_overrides(1,:) = {'STATE_Q', eul2quat([0,0,deg2rad(179)])'};
                                    
        configs{1} = default_config;
        configs{1}.control.method = 'ctc';
        configs{1}.control.fully_actuated = 1;
        configs{1}.tuning.w0p = 2.5;

        configs(2:3,1) = configs(1);
        configs{2}.tuning.w0p = 3;
        configs{3}.tuning.w0p = 3.75;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'tuning_oscillating_more')
        x0_overrides(1,:) = {'STATE_Q', eul2quat([0,0,deg2rad(179)])'};
        
        configs{1} = default_config;
        configs{1}.control.method = 'ctc';
        configs{1}.control.fully_actuated = 1;
        configs{1}.tuning.w0p = 6;
        
        configs(2:3,1) = configs(1);
        configs{2}.tuning.w0p = 6.75;
        configs{3}.tuning.w0p = 7.5;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'tuning_unstable')
        x0_overrides(1,:) = {'STATE_Q', eul2quat([0,0,deg2rad(179)])'};
        
        configs{1} = default_config;
        configs{1}.time.tf = 10;
        configs{1}.control.method = 'ctc';
        configs{1}.control.fully_actuated = 1;
        configs{1}.tuning.w0p = 7.875;
        
        configs(2:3,1) = configs(1);
        configs{2}.tuning.w0p = 8.25;
        configs{3}.tuning.w0p = 8.6250;
    % ---------------------------------------------------------------------
    elseif contains(test_name, 'spiral_base')
        if strcmp(test_name, 'spiral_base_slow')
            dt = 40;
        elseif strcmp(test_name, 'spiral_base_quick')
            dt = 10;
        else
            error(['Unknown test case: ', test_name])
        end
        
        configs{1} = default_config;
        configs{1}.time.tf = dt;
        configs{1}.tname_base = 'circle_raised_yaw';
        configs{1}.tname_r{1} = 'swing_full';
        
        configs(2:5,1) = configs(1);
        configs{2}.control.method = 'ctc';
        configs{2}.control.fully_actuated = 1;
        configs{3}.control.method = 'ctc';
        configs{3}.control.fully_actuated = 0;
        configs{4}.control.method = 'feed';
        configs{4}.control.fully_actuated = 1;
        configs{5}.control.method = 'feed';
        configs{5}.control.fully_actuated = 0;
    % ---------------------------------------------------------------------
    elseif contains(test_name, 'spiral_end')
        if strcmp(test_name, 'spiral_end_slow')
            dt = 40;
        elseif strcmp(test_name, 'spiral_end_quick')
            dt = 10;
        else
            error(['Unknown test case: ', test_name])
        end
        
        % Slight starting offset to position end effector correctly
        x0_overrides(1,:) = {'STATE_XYZ', [1.5;0;1.015]};
        
        configs{1} = default_config;
        configs{1}.time.tf = dt;
        configs{1}.tname_base = 'circle_raised_yaw';
        configs{1}.tname_r{1} = 'swing_full';
        configs{1}.control.tracking_frame = 3;
        
        configs(2:5,1) = configs(1);
        configs{2}.control.method = 'ctc';
        configs{2}.control.fully_actuated = 1;
        configs{3}.control.method = 'ctc';
        configs{3}.control.fully_actuated = 0;
        configs{4}.control.method = 'feed';
        configs{4}.control.fully_actuated = 1;
        configs{5}.control.method = 'feed';
        configs{5}.control.fully_actuated = 0;
    % ---------------------------------------------------------------------
    elseif contains(test_name, 'wind_compensation')
        configs{1} = default_config;
        configs{1}.time.tf = 10;
        configs{1}.spline.tname_base = 'circle_flat';
        configs{1}.control.method = 'ctc';
        configs{1}.wind = [5;0;0];
        
        configs(2,1) = configs(1);
        configs{2}.control.use_pos_int = 1;
    % ---------------------------------------------------------------------
    else
        error(['Unknown test case: ', test_name])
    end
    
    % Display for logging
    print_config(configs{1});
end

