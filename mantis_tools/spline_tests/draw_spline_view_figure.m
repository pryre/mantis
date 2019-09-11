function [ f ] = draw_spline_view_figure( t, s, tv, v, order, n )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    shape_names = {'Constant', 'Linear', 'Quadratic', 'Cubic', 'Quartic', 'Quintic', 'Sextic', 'Septic', 'Octic', 'Nonic'};

    f = figure();
    clf;
    set(f,'defaultAxesColorOrder',[0,0,0;0,0,0]);
    
    via_steps = floor(length(t)/(length(v)-1));
    
    for i = 1:length(v)-1
            ph = ((i-1)*via_steps)+1;
            pl = (i*via_steps);
            pr = ph:pl;
            
            subplot(5,1,1)
                hold on;
                plot(t(pr),s(1,pr),'-b');
                hold off;
                
            subplot(5,1,2)
                hold on;
                plot(t(pr),s(2,pr),'-b');
                hold off;
                
            subplot(5,1,3)
                hold on;
                plot(t(pr),s(3,pr),'-b');
                hold off;
                
            subplot(5,1,4)
                hold on;
                plot(t(pr),s(4,pr),'-b');
                hold off;
                
            subplot(5,1,5)
                if order > 3
                    hold on;
                    plot(t(pr),s(5,pr),'-b');
                    hold off;
                end
    end
    
    % Some magic to only show vias for orders that use them
    for j = 1:floor((order+1)/2)
        subplot(5,1,j)
            hold on;
            scatter(tv, v(j,:), 'ro')
            hold off;
    end
    
    stname = order+1;
    
    subplot(5,1,1)
        title(n)
        ylabel('Position (\(m\))')
        grid on;
        ylim([-2.5,2.5])
        
        yyaxis right;
        yticks([]);
        ylabel(shape_names{stname})
        
    subplot(5,1,2)
        ylabel('Velocity (\(ms^{-1}\))')
        grid on;
        ylim([-3,3])
    
        yyaxis right;
        yticks([]);
        ylabel(shape_names{stname-1})
        
    subplot(5,1,3)
        ylabel('Accel. (\(ms^{-2}\))')
        grid on;
        ylim([-7.5,7.5])
    
        yyaxis right;
        yticks([]);
        ylabel(shape_names{stname-2})
    
    subplot(5,1,4)
        ylabel('Jerk (\(ms^{-3}\))')
        grid on;
        ylim([-25,25])
    
        yyaxis right;
        yticks([]);
        ylabel(shape_names{stname-3})
    
    subplot(5,1,5)
        ylabel('Snap (\(ms^{-4}\))')
        xlabel('Time (\(s\))')
%         ylim([-45,45])
        ylim([-100,100])
        
        if order <= 3
            xlim([tv(1),tv(end)])
            text(4.25,0.5,'Undefined.');
            yyaxis right;
            yticks([]);
            ylabel('')
        else
            grid on;
            
            yyaxis right;
            yticks([]);
            ylabel(shape_names{stname-4})
        end
end

