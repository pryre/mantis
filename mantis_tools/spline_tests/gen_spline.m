function [ t, s ] = gen_spline( t_vias, vias, order, dt )
%GEN_DATA_POINT_STRUCT Summary of this function goes here
%   Detailed explanation goes here

    % Full trajectory times
    t0 = t_vias(1);
    tf = t_vias(end);
    
    % Pre-allocate vectors
    t = t0:dt:tf;
    s = zeros(5,length(t));
%     t = [];
%     s = zeros(5,0);

    % Itterate over the list of vias in pairs
%     si = 1;
    for i = 2:size(vias,2)
        
        % Segment times
        st0 = t_vias(i-1);
        stf = t_vias(i);
        sdt = stf-st0;
        
%         if mod(sdt,dt)
%             error('Bad alignment between t_vias and dt');
%         end
        
        sr0 = find(t==st0);
        srf = find(t==stf);
        
        if ~any(sr0) || ~any(srf)
            error('Could not align segment times to t');
        end
        
        sr = sr0:srf;
        
        st = linspace(0, 1, length(sr));
        
        
        switch(order)
            case 3
               as = spline_solver_cubic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i));
            case 5
                as = spline_solver_quintic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(3,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i), ...
                                            vias(3,i));
            case 7
                as = spline_solver_septic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(3,i-1), ...
                                            vias(4,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i), ...
                                            vias(3,i), ...
                                            vias(4,i));
            case 9
                as = spline_solver_nonic(sdt, ...
                                            vias(1,i-1), ...
                                            vias(2,i-1), ...
                                            vias(3,i-1), ...
                                            vias(4,i-1), ...
                                            vias(5,i-1), ...
                                            vias(1,i), ...
                                            vias(2,i), ...
                                            vias(3,i), ...
                                            vias(4,i), ...
                                            vias(5,i));
            otherwise
                error('Unknown spline order')
        end


        %% Spline Calculator

        a = zeros(10,1);
        a(1:length(as)) = as;
        
        c = ones(1,length(sr));
        
        q =        a(1).*c +     a(2).*st +     a(3).*st.^2 +     a(4).*st.^3 +      a(5).*st.^4 +       a(6).*st.^5 +      a(7).*st.^6 +     a(8).*st.^7 +    a(9).*st.^8 + a(10).*st.^9;
        qd =       a(2).*c +   2*a(3).*st +   3*a(4).*st.^2 +   4*a(5).*st.^3 +    5*a(6).*st.^4 +     6*a(7).*st.^5 +    7*a(8).*st.^6 +   8*a(9).*st.^7 + 9*a(10).*st.^8;
        qdd =    2*a(3).*c +   6*a(4).*st +  12*a(5).*st.^2 +  20*a(6).*st.^3 +   30*a(7).*st.^4 +    42*a(8).*st.^5 +   56*a(9).*st.^6 + 72*a(10).*st.^7;
        qddd =   6*a(4).*c +  24*a(5).*st +  60*a(6).*st.^2 + 120*a(7).*st.^3 +  210*a(8).*st.^4 +   336*a(9).*st.^5 + 504*a(10).*st.^6;
        qdddd = 24*a(5).*c + 120*a(6).*st + 360*a(7).*st.^2 + 840*a(8).*st.^3 + 1680*a(9).*st.^4 + 3024*a(10).*st.^5;

        % Devide derivatives by segment dt to denormalise
        sq = [q; qd./sdt; qdd./(sdt^2); qddd./(sdt^3); qdddd./(sdt^4)];
        
        s(:,sr) = sq;
    end
end

