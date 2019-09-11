function [ ] = draw_quad( ax, x, frame_size, prop_rad )

    pc_right_side = [75, 150, 255] / 255; % light blue
    pc_wrong_side = [255, 0, 0] / 255; % red

    prop_steps = 9;
    prop_sep = [0;0;0.01*prop_rad];
    
    STATE_QW = 1;
    STATE_QX = 2;
    STATE_QY = 3;
    STATE_QZ = 4;
    STATE_X = 5;
    STATE_Y = 6;
    STATE_Z = 7;
    STATE_VX = 8;
    STATE_VY = 9;
    STATE_VZ = 10;
    STATE_WX_B = 11;
    STATE_WY_B = 12;
    STATE_WZ_B = 13;
    STATE_VX_B = 14;
    STATE_VY_B = 15;
    STATE_VZ_B = 16;

    STATE_Q = STATE_QW:STATE_QZ;
    STATE_XYZ = STATE_X:STATE_Z;
    STATE_VXYZ = STATE_VX:STATE_VZ;
    STATE_WXYZ_B = STATE_WX_B:STATE_WZ_B;
    STATE_VXYZ_B = STATE_VX_B:STATE_VZ_B;

    px = x(STATE_X);
    py = x(STATE_Y);
    pz = x(STATE_Z);
    R = quat2rotm(x(STATE_Q)');
    
    %Quadrotor X4
    fs = cos(pi/4) * frame_size / 2;
    layout_1 = [fs;-fs;0];
    layout_2 = [-fs;fs;0];
    layout_3 = [fs;fs;0];
    layout_4 = [-fs;-fs;0];
    
    %% Calculate Frame lines
%     %Quadrotor +4
%     fs = frame_size / 2;
%     frame_x1 = R*[-fs;0;0] + x(STATE_XYZ);
%     frame_x2 = R*[fs;0;0] + x(STATE_XYZ);
%     frame_y1 = R*[0;-fs;0] + x(STATE_XYZ);
%     frame_y2 = R*[0;fs;0] + x(STATE_XYZ);

    frame_p1 = R*layout_1 + x(STATE_XYZ);
    frame_p2 = R*layout_2 + x(STATE_XYZ);
    frame_p3 = R*layout_3 + x(STATE_XYZ);
    frame_p4 = R*layout_4 + x(STATE_XYZ);
    
    frames_a = [frame_p1,frame_p2];
    frames_b = [frame_p3,frame_p4];

    %% Calculate Common Prop
%     %Quadrotor +4
%     prop_p1 = [-frame_size/2;0;0.001];
%     prop_p2 = [frame_size/2;0;0.001]; 
%     prop_p3 = [0;-frame_size/2;0.001]; 
%     prop_p4 = [0;frame_size/2;0.001];

    prop_p1 = layout_1 + prop_sep;
    prop_p2 = layout_2 + prop_sep; 
    prop_p3 = layout_3 + prop_sep; 
    prop_p4 = layout_4 + prop_sep;

    prop_diam = [prop_rad*cos(linspace(0,2*pi,prop_steps));
                 prop_rad*sin(linspace(0,2*pi,prop_steps));
                 zeros(1,prop_steps)];

    %% Calculate "Upwards" prop surface
    prop_1 = zeros(size(prop_diam));
    prop_2 = zeros(size(prop_diam));
    prop_3 = zeros(size(prop_diam));
    prop_4 = zeros(size(prop_diam));
    for i = 1:prop_steps
        prop_1(:,i) = R*(prop_diam(:,i) + prop_p1) + x(STATE_XYZ);
        prop_2(:,i) = R*(prop_diam(:,i) + prop_p2) + x(STATE_XYZ);
        prop_3(:,i) = R*(prop_diam(:,i) + prop_p3) + x(STATE_XYZ);
        prop_4(:,i) = R*(prop_diam(:,i) + prop_p4) + x(STATE_XYZ);
    end
    
    props_x_u = [prop_1(1,:);prop_2(1,:);prop_3(1,:);prop_4(1,:)];
    props_y_u = [prop_1(2,:);prop_2(2,:);prop_3(2,:);prop_4(2,:)];
    props_z_u = [prop_1(3,:);prop_2(3,:);prop_3(3,:);prop_4(3,:)];

    %% Calculate "Downdwards" prop surface
    prop_p1 = layout_1 - prop_sep;
    prop_p2 = layout_2 - prop_sep; 
    prop_p3 = layout_3 - prop_sep; 
    prop_p4 = layout_4 - prop_sep;

    prop_diam = [prop_rad*cos(linspace(0,2*pi,prop_steps));
                 prop_rad*sin(linspace(0,2*pi,prop_steps));
                 zeros(1,prop_steps)];

    prop_1 = zeros(size(prop_diam));
    prop_2 = zeros(size(prop_diam));
    prop_3 = zeros(size(prop_diam));
    prop_4 = zeros(size(prop_diam));
    for i = 1:prop_steps
        prop_1(:,i) = R*(prop_diam(:,i) + prop_p1) + x(STATE_XYZ);
        prop_2(:,i) = R*(prop_diam(:,i) + prop_p2) + x(STATE_XYZ);
        prop_3(:,i) = R*(prop_diam(:,i) + prop_p3) + x(STATE_XYZ);
        prop_4(:,i) = R*(prop_diam(:,i) + prop_p4) + x(STATE_XYZ);
    end
    
    props_x_d = [prop_1(1,:);prop_2(1,:);prop_3(1,:);prop_4(1,:)];
    props_y_d = [prop_1(2,:);prop_2(2,:);prop_3(2,:);prop_4(2,:)];
    props_z_d = [prop_1(3,:);prop_2(3,:);prop_3(3,:);prop_4(3,:)];
    
    %% Draw Quad
    hold on;

    plot3(ax, frames_a(1,:),frames_a(2,:),frames_a(3,:), 'k');
    plot3(ax, frames_b(1,:),frames_b(2,:),frames_b(3,:), 'k');
    patch(ax, props_x_u', props_y_u', props_z_u', pc_right_side);
    patch(ax, props_x_d', props_y_d', props_z_d', pc_wrong_side);

    hold off;
end