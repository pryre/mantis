function [] = plot_axis( f, x, c )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
f2 = figure('Renderer','opengl');
    clf;

    set(f2,'defaultAxesColorOrder',[0,0,0;0,0,0]);

    subplot(5,1,1)
        title(plot_title)

        hold on;
        yyaxis left;
        plot(t,s_x(1,:), 'r-');
        plot(t,x(STATE_X,:), 'b-');
        yyaxis right;
        plot(t,sin(linspace(0,2*pi,length(t))), 'k--')
        hold off;

        grid on;
        yyaxis left;
        ylabel('Position (m)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Sinusoudal Referece (m)');
        ylim([-maxlim maxlim]);
    subplot(5,1,2)
        hold on;
        plot(t,s_x(2,:), 'r-');
        plot(t,x(STATE_VX,:), 'b-');
        hold off;

        grid on;
        ylabel('Velocity (m/s)');
    subplot(5,1,3)
        hold on;
        yyaxis left;
%         plot(t,c(2,:), 'r-');
        plot(t,pitch_c, 'r-');
        plot(t,pitch, 'b-');
        plot(t,atan2(s_x(3,:), g), 'g--');
        yyaxis right;
        plot(t,s_x(3,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time (s)');
        yyaxis left;
        ylabel('Pitch (rad)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Acceleration (m/s^2)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
%         leg = legend('Reference', 'State');
%         set(leg, 'Position',[0.708869049162382 0.242634804985937 0.194642854056188 0.0821428550141199]);

    subplot(5,1,4)
        hold on;
        yyaxis left;
        plot(t,c(CONTROL_WY_B,:), 'r-');
        plot(t,x(STATE_WY_B,:), 'b-');
        yyaxis right;
        plot(t,s_x(4,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time (s)');
        yyaxis left;
        ylabel('Pitch Rate (rad/s)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Jerk (m/s^3)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);

    subplot(5,1,5)
        hold on;
        yyaxis left;
        plot(t,c(CONTROL_DWY_B,:), 'r-');
        yyaxis right;
        plot(t,s_x(5,:), 'k--');
        hold off;

        grid on;
        ytickformat('% .2f');
        xlabel('Time (s)');
        yyaxis left;
        ylabel('Pitch Acceleration (rad/s^2)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);
        yyaxis right;
        ylabel('Snap (m/s^4)');
        maxlim = max(abs(ylim));
        ylim([-maxlim maxlim]);

end

