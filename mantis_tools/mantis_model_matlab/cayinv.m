function [ V ] = cayinv( Vh )
%CAY Performs the invearse Cayley map of a SE(3) pose
%   Detailed explanation goes here

    if size(Vh) ~= size(zeros(4,4))
        error('Input matrix must be of size 4x4')
    end

    w_hat = Vh(1:3,1:3);
    v = Vh(1:3,4);
    
    if w_hat == -eye(3)
        warn('Bad inverse: R == -I')
    end
    
    V1 = vee_down(-2*inv(eye(3)+w_hat)*(eye(3)-w_hat));
    V2 = inv(eye(3)+w_hat)*v;
    
    V = [V1 ; V2];
end

