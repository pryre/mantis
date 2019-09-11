function [ w, wd ] = map_body_rates_accels( a, ad, add )
%UNTITLED3 Summary of this function goes here
%   a:   Acceleration
%   ad:  Jerk
%   add: Snap

    am = norm(a);
    
    b = a/am;
    bd = ad/am + a*((-dot(a,ad))/(am^3));
    
    c = dot(ad,b);
    cd = dot(add,b) + dot(bd,ad);
    
    d = ad - c*b;
    dd = add - (cd*b + c*bd);
    
    e = cross(a,d);
    ed = cross(ad,d) + cross(a,dd);
    
    f = 1/(am^2);
    fd = (-2*dot(a,ad))/(am^4);
    
    w = e*f;
    wd = ed*f + e*fd;

end

