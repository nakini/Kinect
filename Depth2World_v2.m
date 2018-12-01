function [Xw, Yw, Zw, Rw,Gw,Bw, varargout] = Depth2World_v2(depthImg, ...
    maxDepth, flyWinSize, flyDistTh, mergedImg, KK)
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
%   indCommonValid : Valid indices from which the XYZ and RGB values have been
%                   evaluated
%

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
% Some default vlaues
if nargin <= 6
    if nargin == 6 && ~isempty(KK)
        cx = KK(1,3);
        cy = KK(2,3);
        fx = KK(1,1);
        fy = KK(2,2);
    else
        % Extrinsic parameters of the depth camera. These values are collected 
        % from the dicussion forum.
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
    end
    
    % Check for the rest of the arguments.
    if nargin <5
        mergedImg = [];
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
else
    error('ERROR!!! -- Provide propper parameters');
end

% Read the depth image:
% fileName = '~/Desktop/Data/Tushar_Thang/Data/20161116/2016-11-15-13hr-7min/
%           depthImg_0261.ppm';
% fileName = '~/Desktop/Data/TestData/blah/depthImg_0018.ppm';
% depthImg = imread(depthImg);
% depthImg = depthImg (:, end:-1:1);
x3D = zeros(size(depthImg));
y3D = zeros(size(depthImg));
z3D = zeros(size(depthImg));

% For the depth image the coordinate calculation formula is:
% 3D coordinates from point cloud using depth value.. in Kinect coordinate
% space
[maxR, maxC] = size(depthImg);
for r=1:maxR
    for c=1:maxC
        % The depth value is equal to intensity. But it is stored in mm.
        d = double(depthImg(r,c)) / 1000;
        z3D(r,c) = d;
        x3D(r,c) = (c - cx) * z3D(r,c) / fx;
        y3D(r,c) = (r - cy) * z3D(r,c) / fy;
    end
end

% Remove the flying pixels using the window method.
data = struct('x3D', x3D, 'y3D', y3D, 'z3D', z3D);
indxNoFlyPixels = RemoveFlyingPixels(data, flyWinSize, flyDistTh);

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
if ~isempty(mergedImg)
%     depthImgColor = imread(mergedImg);
    R = mergedImg(:,:,1);
    G = mergedImg(:,:,2);
    B = mergedImg(:,:,3);
    
    Rw = R(indCommonValid);
    Gw = G(indCommonValid);
    Bw = B(indCommonValid);
end
% Display the points.
% plot3(Xw, Yw, Zw, '.');

% Return few more information like the index of 3D points in the depth image,
% the value of X, Y and Z in terms of a matrix that is of same size as the depth
% image.
if nargout > 6
    varargout{1} = indCommonValid;
    if nargout > 7
        varargout{2} = x3D;
        if nargout > 8
            varargout{3} = y3D;
            if nargout > 9
                varargout{4} = z3D;
                if nargout > 10
                    error('The out put argument count should not exceed 10');
                end
            end
        end
    end
end