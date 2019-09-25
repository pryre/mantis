function [] = save_analysis_latex( test_name, configs, analyses, show_settling_time )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    if ~exist('./figures', 'dir')
        mkdir('./figures');
    end
    
    filepath = './figures/';
    filename = [test_name, '.tex'];
    fid = fopen([filepath, filename], 'w');
    
    for i = 1:length(configs)
        formatSpec = '%s %s %s %s %s %s \\\\\n';
        formatSpecLast = '%s %s %s %s %s %s\n'; % Can't have the \\ on the last line
        formatSpecNoTs = '%s %s %s %s %s \\\\\n';
        formatSpecLastNoTs = '%s %s %s %s %s\n'; % Can't have the \\ on the last line           
        cname = 'UDEF.';
        
        if strcmp(configs{i}.control.method, 'npid_px4')
            cname = 'NPID';
        elseif strcmp(configs{i}.control.method, 'ctc')
            cname = 'CTC';
        elseif strcmp(configs{i}.control.method, 'feed')
            cname = 'Feed';
        else
            error('control name not set')
        end
        
        if configs{i}.control.fully_actuated
            cname = [cname, '-F'];
        else
            cname = [cname, '-U'];
        end
        
        if ~isnan(analyses{i}.performance.ts)
            % Allow for non-zero t0 by using relative ts
            ts = num2str(analyses{i}.performance.ts - configs{i}.time.t0, ' & %.2f');
        else
            ts = ' & ---';
        end
        
        J = num2str(analyses{i}.performance.J_fuel',' & %.1f');
        pmax = num2str(analyses{i}.max_pos_error',' & %.3f');
        p = num2str(analyses{i}.pos_RMSE',' & %.3f');
        w = num2str(analyses{i}.w_RMSE',' & %.3f');
%         r = num2str(analyses{i}.r_RMSE',' & %.3f');
        
        if show_settling_time
            if i < length(configs)
                fprintf(fid, formatSpec, cname, ts, J, pmax, p, w);
            else
                fprintf(fid, formatSpecLast, cname, ts, J, pmax, p, w);
            end
        else
            if i < length(configs)
                fprintf(fid, formatSpecNoTs, cname, J, pmax, p, w);
            else
                fprintf(fid, formatSpecLastNoTs, cname, J, pmax, p, w);
            end
        end
    end
    
    fclose(fid);
end

