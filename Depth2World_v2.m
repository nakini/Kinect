function [Xw, Yw, Zw] = Depth2World_v2(fileName, maxDepth, flyWinSize)
% This function converts the depth values in meters to world coordinates. The 
% input is a MxN matrix and the out will be M*Nx3 matrix. To convert the depth 
% into world coordiantes we need the instrinsic parameters of the depth camera.
% And if we want to translate them to the RGB camera frame we also need 
% extrinsic parameters.
%
% INPUTs:
%   fileName: Depth image file name
%   maxDepth: Maximum depth which is set by the user
%   flyWinSize: Window size which will be used to get rid of flying pixels
%
% OUTPUTs:
%   Xw, Yw, Zw: X, Y, Z values for each pixel
%

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------

if (nargin < 3)
    flyWinSize = 2;
    if (nargin < 2)
        maxDepth = 3;
    end
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
% imgPixels = imgPixels (:, end:-1:1);
x3D = zeros(size(imgPixels));
y3D = zeros(size(imgPixels));
z3D = zeros(size(imgPixels));

% For the depth image the coordinate calculation formula is:
% 3D coordinates from point cloud using depth value.. in Kinect coordinate
% space
[maxR, maxC] = size(imgPixels);
for r=1:maxR
    for c=1:maxC
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

% Get-rid-of flying pixels using the "window" method. Flying pixel is a inherent
% problem with the ToF sensors.
% TODO:
%   I need to check 2 other mehods where the concept of the normal is being
%   used to get rid of flying pixels.
distMat = zeros(size(imgPixels));
tmpDist = 0;
for r=3:maxR-2
    for c=3:maxC-2
        for y=r-flyWinSize:1:r+flyWinSize
            for x=c-flyWinSize:1:c+flyWinSize
                tmpDist = abs(norm([x3D(r,c) - x3D(y,x), y3D(r,c)-y3D(y,x), ...
                    z3D(r,c)-z3D(y,x)]));
            end
        end
        
        distMat(r,c) = tmpDist;
    end
end
% Set the threshold to remove all the flying pixels.
indxNoFlyPixels = (distMat > 0) & (distMat < 0.05);
% Remove all the points which are beyond the required depth.
indxValid = z3D > 0.5 & z3D < maxDepth;

Xw = x3D(indxValid & indxNoFlyPixels);
Yw = y3D(indxValid & indxNoFlyPixels);
Zw = z3D(indxValid & indxNoFlyPixels);

% Display the points.
% plot3(Xw, Yw, Zw, '.');
