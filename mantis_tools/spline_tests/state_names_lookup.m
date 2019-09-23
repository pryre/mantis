function [ sn ] = state_names_lookup(n)
%STATE_NAMES_LOOKUP State names lookup for state vector
% Defines a set of constants to use with the state and control vectors.

    sn.STATE_QW = 1;
    sn.STATE_QX = 2;
    sn.STATE_QY = 3;
    sn.STATE_QZ = 4;
    sn.STATE_X = 5;
    sn.STATE_Y = 6;
    sn.STATE_Z = 7;
    sn.STATE_R = 8:8+n-1;
    sn.STATE_VX = 8+n;
    sn.STATE_VY = 9+n;
    sn.STATE_VZ = 10+n;
    sn.STATE_WX_B = 11+n;
    sn.STATE_WY_B = 12+n;
    sn.STATE_WZ_B = 13+n;
    sn.STATE_VX_B = 14+n;
    sn.STATE_VY_B = 15+n;
    sn.STATE_VZ_B = 16+n;
    sn.STATE_RD = 17+n:17+2*n-1;

    sn.STATE_Q = sn.STATE_QW:sn.STATE_QZ;
    sn.STATE_XYZ = sn.STATE_X:sn.STATE_Z;
    sn.STATE_VXYZ = sn.STATE_VX:sn.STATE_VZ;
    sn.STATE_WXYZ_B = sn.STATE_WX_B:sn.STATE_WZ_B;
    sn.STATE_VXYZ_B = sn.STATE_VX_B:sn.STATE_VZ_B;
    sn.STATE_XI_B = [sn.STATE_WXYZ_B, sn.STATE_VXYZ_B];
    sn.STATE_REDUCED = [sn.STATE_WXYZ_B,sn.STATE_VXYZ_B,sn.STATE_RD];
    sn.STATE_REDUCED_W_B = 1:3;
    sn.STATE_REDUCED_V_B = 4:6;
    sn.STATE_REDUCED_XI_B = [sn.STATE_REDUCED_W_B, sn.STATE_REDUCED_V_B];
    sn.STATE_REDUCED_RD = 7:7+n-1;

    sn.CONTROL_AX = 1;
    sn.CONTROL_AY = 2;
    sn.CONTROL_AZ = 3;
    sn.CONTROL_QW = 4;
    sn.CONTROL_QX = 5;
    sn.CONTROL_QY = 6;
    sn.CONTROL_QZ = 7;
    sn.CONTROL_R = 8:8+n-1;
    sn.CONTROL_E_X = 8+n;
    sn.CONTROL_E_Y = 9+n;
    sn.CONTROL_E_Z = 10+n;
    sn.CONTROL_WX_B = 11+n;
    sn.CONTROL_WY_B = 12+n;
    sn.CONTROL_WZ_B = 13+n;
    sn.CONTROL_RD = 14+n:14+2*n-1;
    sn.CONTROL_DWX_B = 14+2*n;
    sn.CONTROL_DWY_B = 15+2*n;
    sn.CONTROL_DWZ_B = 16+2*n;
    sn.CONTROL_DVX_B = 17+2*n;
    sn.CONTROL_DVY_B = 18+2*n;
    sn.CONTROL_DVZ_B = 19+2*n;
    sn.CONTROL_RDD = 20+2*n:20+3*n-1;
    sn.CONTROL_TAU_BW = 20+3*n:20+3*n+3-1;
    sn.CONTROL_TAU_BV = 20+3*n+3:20+3*n+6-1;
    sn.CONTROL_TAU_R = 20+3*n+6:20+4*n+6-1;

    sn.CONTROL_A = sn.CONTROL_AX:sn.CONTROL_AZ;
    sn.CONTROL_Q = sn.CONTROL_QW:sn.CONTROL_QZ;
    sn.CONTROL_E = sn.CONTROL_E_X:sn.CONTROL_E_Z;
    sn.CONTROL_WXYZ_B = sn.CONTROL_WX_B:sn.CONTROL_WZ_B;
    sn.CONTROL_DWXYZ_B = sn.CONTROL_DWX_B:sn.CONTROL_DWZ_B;
    sn.CONTROL_DVXYZ_B = sn.CONTROL_DVX_B:sn.CONTROL_DVZ_B;
    sn.CONTROL_TAU = [sn.CONTROL_TAU_BW, sn.CONTROL_TAU_BV, sn.CONTROL_TAU_R];

    sn.SPLINE_POS = 1;
    sn.SPLINE_VEL = 2;
    sn.SPLINE_ACC = 3;
    sn.SPLINE_JERK = 4;
    sn.SPLINE_SNAP = 5;
end

