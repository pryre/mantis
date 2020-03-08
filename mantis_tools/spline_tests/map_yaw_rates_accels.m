% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

% This Source Code Form is subject to the terms of the Mozilla Public
% License, v. 2.0. If a copy of the MPL was not distributed with this
% file, You can obtain one at https://mozilla.org/MPL/2.0/.

function [ we_sp, wed_sp ] = map_yaw_rates_accels( R_sp, w_sp, dyaw, ddyaw )
%UNTITLED3 Summary of this function goes here

% %     wed = R_sp_s'*[0;0;dyaw];
% %     wdedd = R_sp_s'*[0;0;ddyaw];
%     eul = rotm2eul(R_sp, 'ZYX');
% %     % phi = eul(3);
% %     % theta = eul(2);
% %     % psi = eul(1);
% %     % Lbi: L^{B}_{I} (L^{I}_{B}???)
% %     
% %     % http://www.stengel.mycpanel.princeton.edu/Quaternions.pdf
% %     Lbi = [1,0,-sin(eul(2));
% %            0, cos(eul(3)), sin(eul(3))*cos(eul(2));
% %            0, -sin(eul(3)), cos(eul(3))*cos(eul(2))];
% %     % Shorthand for Yaw-Rate-Only
%     Lbiz = [-sin(eul(2));
%             sin(eul(3))*cos(eul(2));
%             cos(eul(3))*cos(eul(2))];
%         
%     Lbizd = [0;
%              dyaw*cos(eul(3))*cos(eul(2));
%              dyaw*(-sin(eul(3)))*(cos(eul(2)))];
% 
%     % https://arxiv.org/pdf/1609.06088.pdf
%     %   w_a = R_ab * w_b
%     % For d(eul) = [0;0;d(psi)]:
%     %   w_b = Lbi*d(eul) === w_b = R_ab*d(eul)
%     % Therefore:
%     %   w_b = R_ab' * [0;0;d(psi)]
% %     ed_sp = [0;0;dyaw];
% %     edd_sp = [0;0;ddyaw];
% % 
% %     we_sp = R_sp'*ed_sp;
% %     wed_sp = (-vee_up(w_sp)*R_sp')*ed_sp + R_sp'*edd_sp;
% 
% %     ed_sp = [0;0;dyaw];
% %     edd_sp = [0;0;ddyaw];
% %     we_sp = R_sp'*ed_sp;
% %     wed_sp = (-vee_up(w_sp)*R_sp')*ed_sp + R_sp'*edd_sp;
% %     wed_sp = (-vee_up(we_sp)*R_sp')*ed_sp + R_sp'*edd_sp;
%     we_sp = Lbiz*dyaw;
%     wed_sp = Lbizd*dyaw + Lbiz*ddyaw;


    % In this case, we want to get the w_a (where w_a_z coincides with
    % euler yaw rate with no rotation) into the reference rotation frame
    % (R_sp). In this sense, we are doing the reverse of the function
    % error_w_wd_trans(). The process is simplified as w_a has no rotation.

    ed_sp = [0;0;dyaw];
    edd_sp = [0;0;ddyaw];

%     % R_sp->R_e
%     R_e = eye(3);
%     eR = R_sp'*R_e;
    eR = R_sp';
    eRd = (-vee_up(w_sp)*R_sp');

%     % ew is the omega error in the body frame at the current time
    we_sp = eR*ed_sp;
    wed_sp = eRd*ed_sp + eR*edd_sp;


end

