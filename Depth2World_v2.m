function [Xw, Yw, Zw, Rw,Gw,Bw] = Depth2World_v2(depthFileName, ...
    maxDepth, flyWinSize, flyDistTh, mergedFileName)
% This function converts the depth values in meters to world coordinates. The 
% input is a MxN matrix and the out will be M*Nx3 matrix. To convert the depth 
% into world coordiantes we need the instrinsic parameters of the depth camera.
% And if we want to translate them to the RGB camera frame we also need 
% extrinsic parameters.
%
% INPUTs:
%   fileName    : Depth image file name
%   maxDepth    : Maximum depth which is set by the user
%   flyWinSize  : Window size which will be used to get rid of flying pixels
%   flyDistTh   : Threshold to determine whether to keep/discard pixels after  
%           the flying window operation
%   mergedFileName: File of the image that contains the R,G,B values
%
% OUTPUTs:
%   Xw, Yw, Zw  : X, Y, Z values for each pixel
%   Rw, Gw, Bw  : Color informatio for each pixel
%

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
% Some default vlaues
if nargin <5
    mergedFileName = 'No_Merged_FileName';
    if(nargin <4)
        flyDistTh = 0.02;
        if (nargin < 3)
            flyWinSize = 3;
            if (nargin < 2)
                maxDepth = 3;
            end
        end
    end
end
% Extrinsic parameters of the depth camera. These values are collected from the
% dicussion forum.
% fx=367.286994337726;        % Focal length in X and Y
% fy=367.286855347968;
% cx=255.165695200749;        % Principle point in X and Y
% cy=211.824600345805;
% k1=0.0914203770220268;
% k2=-0.269349746097515;
% k3=0.0925671408453617;
% p1=0;
% p2=0;

% Parameters from the libfreenect2 running in dubug mode while the kinect
% is plugged in.		
cx = 262.299194;
cy = 206.395004;
fx = 368.053497;
fy = 368.053497;
k1 = 0.0842951;
k2 = -0.271216;
k3 = 0.10087239;
p1 = 0;
p2 = 0;

% Read the depth image:
% fileName = '~/Desktop/Data/Tushar_Thang/Data/20161116/2016-11-15-13hr-7min/
%           depthImg_0261.ppm';
% fileName = '~/Desktop/Data/TestData/blah/depthImg_0018.ppm';
imgPixels = imread(depthFileName);
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
for r=flyWinSize+1:maxR-flyWinSize
    for c=flyWinSize+1:maxC-flyWinSize
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
indxNoFlyPixels = (distMat > 0) & (distMat < flyDistTh);
% Remove all the points which are beyond the required depth.
indxValid = z3D > 0.5 & z3D < maxDepth;

indCommonValid = indxValid & indxNoFlyPixels;
Xw = x3D(indCommonValid);
Yw = y3D(indCommonValid);
Zw = z3D(indCommonValid);

% Color information from the merged image. If the merged file name is mentioned
% then get the color information
Rw = []; Gw = []; Bw = [];
% If the mergedFileName variable has the default value then it has not been
% mentioned by the user, so don't calculate the RGB values.
if ~strcmpi(mergedFileName,'No_Merged_FileName')
    imgPixelsColor = imread(mergedFileName);
    R = imgPixelsColor(:,:,1);
    G = imgPixelsColor(:,:,2);
    B = imgPixelsColor(:,:,3);
    
    Rw = R(indCommonValid);
    Gw = G(indCommonValid);
    Bw = B(indCommonValid);
end
% Display the points.
% plot3(Xw, Yw, Zw, '.');
