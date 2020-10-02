function J = DerivativeOf_f(C, R, K, X, Case)
% BRIEF
% =====
% Here, we are going to find out the derivative of the of the function "f(R(q),
% C, X) w.r.t the R(q), C and X respectively.
% 1. doh_f/doh_X
%       doh_f/doh_X = [(w*doh_u/doh_X - u*doh_w/doh_X)/w^2; 
%                        (w*doh_v/doh_X - v*doh_w/doh_X)/w^2]
%       doh_u/doh_X = [f*r11+px*r31, f*r12+px*r32, f*r13+px*r33]
%       doh_v/doh_X = [f*r21+py*r31, f*r22+py*r32, f*r23+Py*r33]
%       doh_w/doh_X = [r31, r32, r33]
% where,
%       K = [f 0 p_x; 
%            0 f p_y;
%            0 0 1]
%       R = [r11, r12, r13;
%            r21, r22, r23;
%            r31, r32, r33]
%
% INPUT(s):
%
% OUTPUT(s):
%   J = 2x3 matrix

% Get the pixel coordinates from the 3D coordinates

if Case==1
    % Derivative w.r.t X
    J = DerivativeOf_f_wrt_X(C, R, K, X);
elseif Case==2
    % Derivative w.r.t C
    J = DerivativeOf_f_wrt_X(C, R, K, X);
    J = -J;
else
    % Derivative w.r.t R
    J = DerivativeOf_f_wrt_q(C, R, K, X);
end

end

%% Original Functions
function J = DerivativeOf_f_wrt_X(C, R, K, X)
proj = K*R*(X-C);
u = proj(1);
v = proj(2);
w = proj(3);

f = K(1,1);
px = K(1,3);
py = K(2,3);

r11 = R(1,1); r12 = R(1,2); r13 = R(1,3);
r21 = R(2,1); r22 = R(2,2); r23 = R(2,3);
r31 = R(3,1); r32 = R(3,2); r33 = R(3,3);

du_by_dX = [f*r11+px*r31, f*r12+px*r32, f*r13+px*r33];
dv_by_dX = [f*r21+py*r31, f*r22+py*r32, f*r23+py*r33];
dw_by_dX = [r31, r32, r33];
df_by_dX = [(w*du_by_dX - u*dw_by_dX)/w^2; (w*dv_by_dX - v*dw_by_dX)/w^2];

J = df_by_dX;

end

function J = DerivativeOf_f_wrt_q(C, R, K, X)
proj = K*R*(X-C);
u = proj(1);
v = proj(2);
w = proj(3);

f = K(1,1);
px = K(1,3);
py = K(2,3);

q = rotm2quat(R);
qw = q(1); qx = q(2); qy = q(3); qz = q(4);
df_by_dR = [f*(X-C)', 0, 0, 0, px*(X-C)'; ...
            0, 0, 0, f*(X-C)', py*(X-C)'];
dR_by_dq = [0,      -4*qy,  -4*qz,  0; ...
            2*qy,   2*qx,   -2*qw,  -2*qz; ...
            2*qz,   2*qw,   2*qx,   2*qy; ...
            2*qy,   2*qx,   2*qw,   2*qz; ...
            -4*qx,  0,      -4*qz,  0; ...
            -2*qw,  2*qz,   2*qy,   -2*qx; ...
            2*qz,   -2*qw,  2*qx,   -2*qy; ...
            2*qw,   2*qz,   2*qy,   2*qx; ...
            -4*qx,  -4*qy,  0,      0];
J = df_by_dR * dR_by_dq;
end