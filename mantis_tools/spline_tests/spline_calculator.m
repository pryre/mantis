function [q] = spline_calculator(t, dt, a)
%SPLINE_CALCULATOR Calculates a spline segment based on time and coeffs
%   Detailed explanation goes here
        
%         c = ones(1,length(st));
%         "a" VECTOR USED IS THE REVERSE OF THIS EXAMPLE (i.e. 1 <-> 10)
%         q =        a(1).*c +     a(2).*st +     a(3).*st.^2 +     a(4).*st.^3 +      a(5).*st.^4 +       a(6).*st.^5 +      a(7).*st.^6 +     a(8).*st.^7 +    a(9).*st.^8 + a(10).*st.^9;
%         qd =       a(2).*c +   2*a(3).*st +   3*a(4).*st.^2 +   4*a(5).*st.^3 +    5*a(6).*st.^4 +     6*a(7).*st.^5 +    7*a(8).*st.^6 +   8*a(9).*st.^7 + 9*a(10).*st.^8;
%         qdd =    2*a(3).*c +   6*a(4).*st +  12*a(5).*st.^2 +  20*a(6).*st.^3 +   30*a(7).*st.^4 +    42*a(8).*st.^5 +   56*a(9).*st.^6 + 72*a(10).*st.^7;
%         qddd =   6*a(4).*c +  24*a(5).*st +  60*a(6).*st.^2 + 120*a(7).*st.^3 +  210*a(8).*st.^4 +   336*a(9).*st.^5 + 504*a(10).*st.^6;
%         qdddd = 24*a(5).*c + 120*a(6).*st + 360*a(7).*st.^2 + 840*a(8).*st.^3 + 1680*a(9).*st.^4 + 3024*a(10).*st.^5;
% 
%         % Devide derivatives by segment dt to denormalise
%         sq = [q; qd./sdt; qdd./(sdt^2); qddd./(sdt^3); qdddd./(sdt^4)];

    qn = zeros(5,length(t));
    
    for i = 1:5
        % Perform the polynomial derivative (skip first)
        if i > 1
            a = polyder(a);
        end
        
        for j = 1:length(a)
            p = length(a)-j;

            if p > 0
                qn(i,:) = qn(i,:) + a(j).*(t.^p);
            else
                % This is the constant, so multiply by ones instead
                qn(i,:) = qn(i,:) + a(j);
            end
        end
        
    end
    
    q = zeros(size(qn));
    q(1,:) = qn(1,:);
    for i = 2:size(qn,1)
        q(i,:) = qn(i,:)./(dt^(i-1));
    end
    
%     q = [qn; qn./sdt; qdd./(sdt^2); qddd./(sdt^3); qdddd./(sdt^4)];

end

