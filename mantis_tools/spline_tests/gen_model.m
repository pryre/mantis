function [ fbmodel ] = gen_model( frame_type, manip_type, n, camera )
%GEN_MODEL Generates the model for an n-link aerial manipulator
% Inputs
%   type: type of manipulator to attach to the floating base (serial, etc.)
%      n: number of links in the kinematic chain

    BASE_THICK = 0.015; % Half thickness
    BASE_RAD = 0.1;
    FRAME_SPAN = 0.225;
    FRAME_RAD = 2*BASE_THICK/3;
    PROP_THICK = 0.005;
    PROP_RAD = 0.1;
    LINK_LEN = 0.25;
    LINK_RAD = 0.01;
    
    % Mass properties (body-fixed to the respecitve body)
    BASE_MASS = 1.83497;
    BASE_IXX = 0.02961;
    BASE_IYY = 0.02961;
    BASE_IZZ = 0.05342;
    LINK_MASS = 0.2;
    LINK_IXX = 0.00001345;
    LINK_IYY = 0.00002333;
    LINK_IZZ = 0.00002098;
    
    % Additional Link setup
    mIc = diag( [LINK_IXX,LINK_IYY,LINK_IZZ] );
        
    model.camera = camera;
    model.NB = n+1;
    model.parent = 0:n;
    model.jtype = cell(1,model.NB);
    model.Xtree = cell(1,model.NB); %model.Xtree = {eye(6), eye(6)};
    model.I = cell(1,model.NB);

    model.gravity = [0 0 0];    % zero gravity is not the default,
                                % so it must be stated explicitly.
                                % This is dealt with during the
                                % control steps instead.

     % Base setup (Sane for all frame types, only difference is visual
        model.jtype{1} = 'R';	% 1st joint will be replaced by floatbase
        model.Xtree(1) = {eye(6)}; % First point is at the origin, plucker -> [eye(3);[0;0;0]]
        model.I{1} = mcI(BASE_MASS, [0,0,0], diag([BASE_IXX,BASE_IYY,BASE_IZZ]));
    if strcmp(frame_type, 'quad_x4')
        fd = FRAME_SPAN*cos(pi/4);
        model.appearance.body{1} = { ... 
            'colour', [0.8 0.3 0.3], ...            % Base Colour
            'cyl', [0 0 -BASE_THICK; 0 0 BASE_THICK], BASE_RAD, ...
            'colour', [0.8 0.3 0.3], ...            % Frame Colour (forward)
            'cyl', [0 0 0; fd -fd 0], FRAME_RAD, ...
            'cyl', [0 0 0; fd fd 0], FRAME_RAD, ...
            'colour', [0.5 0.5 0.5], ...            % Frame Colour (backward)
            'cyl', [0 0 0; -fd fd 0], FRAME_RAD, ...
            'cyl', [0 0 0; -fd -fd 0], FRAME_RAD, ...
            'colour', [0.2 0.2 0.2], ...            % Base Colour
            'cyl', [fd -fd BASE_THICK; fd -fd BASE_THICK+PROP_THICK], PROP_RAD, ...
            'cyl', [-fd fd BASE_THICK; -fd fd BASE_THICK+PROP_THICK], PROP_RAD, ...
            'cyl', [fd fd BASE_THICK; fd fd BASE_THICK+PROP_THICK], PROP_RAD, ...
            'cyl', [-fd -fd BASE_THICK; -fd -fd BASE_THICK+PROP_THICK], PROP_RAD, ...
        };
    else
        error('Unsupported frame type')
    end        
                                
    if strcmp(manip_type, 'serial')
        for i = 2:model.NB
            model.jtype{i} = 'Rz';
            bt = [LINK_LEN, 0, 0];
            
            if (i == 2)
                % Small offset to align with the bottom of the platform for
                % the first joint
                bo = [0 0 -BASE_THICK];
                % Transformation is set up to be rotated downwards such
                % that the z-axis in the direction of -y-axis of the base,
                % and the x-axis is aligned such that q=0 points the arm
                % downwards
                model.Xtree(i) = {plux(rz(-pi/2)*rx(pi/2), bo)};
            else
                % For the rest of the links, we stack them straight
                % downwards, only varying the link length
                bt = [LINK_LEN 0 0];
                model.Xtree(i) = {plux(eye(3), bt)};
            end
            
            %TODO: Link mass properties is estimated to centered for time being
            model.I{i} = mcI(LINK_MASS, bt/2, mIc);
            
            j0s = [0,0,-LINK_RAD];
            j0e = [0,0,LINK_RAD];
            
            model.appearance.body{i} = ...
              { 'colour', [0.3 0.3 0.8], ...            % Joint Colour
                'cyl', [zeros(1,3); bt], LINK_RAD, ...  % Link
                'colour', [0.6 0.3 0.8], ...            % Joint Colour
                'cyl', [j0s; j0e], LINK_RAD };          % Previous Joint
        end
        
        fbmodel = floatbase(model); % replace joint 1 with a chain of 6
                                    % joints emulating a floating base
    else
        error('Unsupported manipulator type')
    end

end
