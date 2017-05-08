function [Xw, Yw, Zw] = Depth2World_v2(fileName, maxDepth)
% This function converts the depth values in meters to world coordinates. The 
% input is a MxN matrix and the out will be M*Nx3 matrix. To convert the depth 
% into world coordiantes we need the instrinsic parameters of the depth camera.
% And if we want to translate them to the RGB camera frame we also need 
% extrinsic parameters.

if (nargin < 1)
    maxDepth = 3;
end

% Extrinsic parameters of the depth camera. These values are collected from the
% dicussion forum.
fx=367.286994337726;        % Focal length in X and Y
fy=367.286855347968;
cx=255.165695200749;        % Principle point in X and Y
cy=211.824600345805;
k1=0.0914203770220268;
k2=-0.269349746097515;
k3=0.0925671408453617;
p1=0;
p2=0;

% Read the depth image:
% fileName = '~/Desktop/Data/Tushar_Thang/Data/20161116/2016-11-15-13hr-7min/
%           depthImg_0261.ppm';
% fileName = '~/Desktop/Data/TestData/blah/depthImg_0018.ppm';
imgPixels = imread(fileName);
imgPixels = imgPixels (:, end:-1:1);
x3D = zeros(size(imgPixels));
y3D = zeros(size(imgPixels));
z3D = zeros(size(imgPixels));

% For the depth image the coordinate calculation formula is:
% 3D coordinates from point cloud using depth value.. in Kinect coordinate
% space
for r=1:424
    for c=1:512
        % The depth value is equal to intensity. But it is stored in mm.
        d = double(imgPixels(r,c)) / 1000;
%         u = (c - cx)/fx;
%         v = (r - cy)/fy;
%         z3D(r,c) = sqrt(d^2/(u^2+v^2));
        z3D(r,c) = d;
        x3D(r,c) = (c - cx) * z3D(r,c) / fx;
        y3D(r,c) = (r - cy) * z3D(r,c) / fy;
    end
end

% Remove all the points which are beyond the required depth.
indxValid = z3D > 0.5 & z3D < maxDepth;

Xw = x3D(indxValid);
Yw = y3D(indxValid);
Zw = z3D(indxValid);

% Display the points.
plot3(Xw, Yw, Zw, '.');
