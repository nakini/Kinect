function [Xw, Yw, Zw, Rw,Gw,Bw, varargout] = Depth2World_RS(depthImg, ...
    maxDepth, flyWinSize, flyDistTh, mergedImg, KK)

% Some default vlaues
if nargin <= 6
    if nargin == 6 && ~isempty(KK)
        cx = KK(1,3);
        cy = KK(2,3);
        fx = KK(1,1);
        fy = KK(2,2);
    else
        % Parametes from the Github page -- 
        % https://github.com/IntelRealSense/librealsense/issues/2488#issue-367687873
        % https://github.com/IntelRealSense/librealsense/issues/2948
        % https://github.com/IntelRealSense/librealsense/wiki/
        % Projection-in-RealSense-SDK-2.0#pixel-coordinates
        % https://github.com/IntelRealSense/librealsense/issues/2982
        camera_factor = 1;
%         cx = 212.559662;        % the resolution is 424*240
%         cy = 123.307076;
%         fx = 213.512985;
%         fy = 213.512985;
        
        cx = 661.7976;              % the resolution is 1280*720
        cy = 382.0784;
        fx = 651.5353;
        fy = 645.9398;
    end
    
    % Check for the rest of the arguments.
    if nargin <5
        mergedImg = [];
        if(nargin <4)
            flyDistTh = 0.002;
            if (nargin < 3)
                flyWinSize = 3;
                if (nargin < 2)
                    maxDepth = 0;
                end
            end
        end
    end
else
    error('ERROR!!! -- Provide propper parameters');
end

[maxR, maxC] = size(depthImg);
x3D = zeros(maxR, maxC);
y3D = zeros(maxR, maxC);
z3D = zeros(maxR, maxC);

for r=1:maxR
    for c=1:maxC
        d = depthImg(r,c);
        if  d == 0
            continue;
        end
        z3D(r,c) = d / camera_factor;
        x3D(r,c) = (c - cx) * z3D(r,c) / fx;
        y3D(r,c) = (r - cy) * z3D(r,c) / fy;
    end
end

% Remove the flying pixels using the window method.
data = struct('x3D', x3D, 'y3D', y3D, 'z3D', z3D);
[indxNoFlyPixels, z3D] = RemoveFlyingPixels(data, flyWinSize, flyDistTh);

% Remove all the points which are beyond the required depth.
indxValidZ = z3D > 0.5 & z3D < 1;
indxValidX = x3D > -.75 & x3D < .75;
indxValidY = x3D > -.75 & x3D < .75;

% indCommonValid = indxValidZ & indxNoFlyPixels;
indCommonValid = indxValidZ & indxValidX & indxValidY & indxNoFlyPixels;

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
