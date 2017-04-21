function [Xw, Yw, Zw] = depth2World(depthInMeters, maxDepth)
% This function converts the depth values in meters to world coordinates. The input is a
% MxN matrix and the out will be M*Nx3 matrix. To convert the depth into world coordiantes
% we need the instrinsic parameters of the depth camera. And if we want to translate them
% to the RGB camera frame we also need extrinsic parameters.

if (nargin < 2)
    maxDepth = 3;
end
% Extrinsic parameters of the depth camera. These values are obtained from Herrera's TPAMI
% 2012 paper.
fx_d = 579.83;      % Focal length in X and Y
fy_d = 586.73;
cx_d = 321.55;      % Principle point in X and Y
cy_d = 235.01;

% Find all the values which are in between .3 to 3 meters. We could extend the upper
% limit till 3.5 meters, as this is the upper limit of the kinect.
[row col] = find(depthInMeters > 0.3 & depthInMeters < maxDepth);
indxValues = sub2ind(size(depthInMeters), row, col);


% Convert them into 
Zd = depthInMeters(indxValues);
Xd = (row - cx_d) .* Zd / fx_d;
Yd = (col - cy_d) .* Zd / fy_d;

% Kinect is coordinate frame is different from the regular world frame. So first transform
% all the point from the kinect frame to regular world frame.
Xw = Yd;
Yw = Zd;
Zw = -Xd;