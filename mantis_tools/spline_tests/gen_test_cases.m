function [ configs, x0_overrides ] = gen_test_cases( test_name )
%GEN_TEST_CASES Helper function to generate preset test cases
% test_name:
%   - 'hover_0_0':        (arm downward)
%   - 'hover_90_0':       (arm forward)
%   - 'full_state_error': ([1,1,0;pi/2] -> [0,0,1,0], arm down -> arm up)
%   - 'inversion_0_0':    Inversion recovery, roll axis return to hover (arm downward)
%   - 'inversion_90_0':   Inversion recovery, roll axis return to hover (arm forward)
%   - 'tuning':           Tuning test to demonstrate different error dynamics

    % Bad starting states:
    % x0(sn.STATE_Q) = eul2quat([pi/2,0,0])';    % Half yaw error rotation (World)
    % x0(sn.STATE_Q) = eul2quat([0,0,deg2rad(179)])';    % (Almost) Full roll error rotation (World)
    % x0(sn.STATE_XYZ) = [1;1;0];    % Positional Error (World)
    % x0(sn.STATE_R) = zeros(n,1);   % Arm down Joint Error (World)
    % x0_overrides(1,:) = {'STATE_Q', eul2quat([pi/2,0,0])'};

    configs = cell(0,1);
    x0_overrides = cell(0,2);

    if strcmp(test_name, 'hover_0_0')
        configs{1} = gen_config(0, 1/1000, 1/250, 20, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'hover', {'steady_0';'steady_0'}, ...
                                'npid_px4', 0, 0.6, deg2rad(30), ...
                                2, 20, 4);
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
        configs{1} = gen_config(0, 1/1000, 1/250, 20, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'hover', {'steady_90';'steady_0'}, ...
                                'npid_px4', 0, 0.6, deg2rad(30), ...
                                2, 20, 4);
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
        
        configs{1} = gen_config(0, 1/1000, 1/250, 20, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'hover', {'steady_90';'steady_90'}, ...
                                'npid_px4', 0, 0.6, deg2rad(30), ...
                                2, 20, 4);
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
        
        configs{1} = gen_config(0, 1/1000, 1/250, 20, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'hover2', {'steady_0';'steady_0'}, ...
                                'npid_px4', 0, 0.6, deg2rad(30), ...
                                2, 20, 4);
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
        
        configs{1} = gen_config(0, 1/1000, 1/250, 20, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'hover2', {'steady_90';'steady_0'}, ...
                                'npid_px4', 0, 0.6, deg2rad(30), ...
                                2, 20, 4);
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
    elseif strcmp(test_name, 'tuning')
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
        
        configs{1} = gen_config(0, 1/1000, 1/250, 10, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'hover', {'steady_0';'steady_0'}, ...
                                'ctc', 1, 0.6, deg2rad(30), ...
                                2, 20, 4);
        configs(2:5,1) = configs(1);
        % No control:
        % w0p = 0;
        % w0t = 0;
        % Slight ocsillation:
        configs{2}.tuning.w0p = 5;
        configs{2}.tuning.w0t = 20;
        % Critical Inversion:
        configs{3}.tuning.w0p = 10;
        configs{3}.tuning.w0t = 20;
        % Converging Spiral:
        configs{4}.tuning.w0p = 11;
        configs{4}.tuning.w0t = 20;
        % Diverging Spiral:
        configs{5}.tuning.w0p = 11.5;
        configs{5}.tuning.w0t = 20;
    % ---------------------------------------------------------------------
    elseif strcmp(test_name, 'spiral_base')        
        configs{1} = gen_config(0, 1/1000, 1/250, 40, ...
                                'quad_x4', 'serial', 2, ...
                                9, 9, 'fdcc', 'circle_raised_yaw', {'swing_full';'swing_full'}, ...
                                'npid_px4', 0, 0.6, deg2rad(30), ...
                                2, 20, 4);
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
    else
        error('Unknown test case')
    end
end

