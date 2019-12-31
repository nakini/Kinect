function [indxSurfels, z3D] = RemoveFlyingPixels(data, flyWinSize, flyDistTh, ...
    flyPixCorrTh, fixFlyPixFlag)
% Get-rid-of flying pixels using the "window" method. Flying pixel is a inherent
% problem with the ToF sensors. There are another 2 methods based on the normal
% at each pixel. However, it is concluded by the authors that the window method
% is best suitable for removing the flying pixels. For more information read the
% paper: "Identification and Correction of Flying Pixels in Range Camera Data"
%
% INPUT(s):
%   data3D      : Structure containing X, Y and Z values for each pixel
%   flyWinSize  : Window size, usually 1 or 2
%   flyWinTh    : Float values such as 0.08, 0.1, etc.
%   flyPixCorrTh: Flying pixel correction threshold. It should be greater than
%       flyWinTh value.
%
% OUTPUT(s):
%   indxSurfels : Valid indices
%   z3D         : Updated data values
%
% EXAMPLE(s):
%

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
if nargin < 5
    fixFlyPixFlag = false;
end
if nargin < 4
    flyPixCorrTh = flyDistTh + 0.01;
end
[maxR, maxC] = size(data.x3D);          % Image matrix size
distMat = zeros(maxR, maxC);            % Matrix for dist values
distMatFlySurfels = distMat;            % Will hold the corrected flying pixels

% Find the sum of the distances in the bounding box for each pixel.
for r=flyWinSize+1:maxR-flyWinSize
    for c=flyWinSize+1:maxC-flyWinSize
        tmpDist = 0;
        for y=r-flyWinSize:1:r+flyWinSize
            for x=c-flyWinSize:1:c+flyWinSize
                tmpDist = tmpDist + abs(norm([data.x3D(r,c) - data.x3D(y,x), ...
                    data.y3D(r,c)-data.y3D(y,x), ...
                    data.z3D(r,c)-data.z3D(y,x)]));
            end
        end
        distMat(r,c) = tmpDist;
    end
end
% Set the threshold to remove all the flying pixels.
indxValidSurfels = (distMat > 0) & (distMat < flyDistTh);

disp("Total number of pixel: " + nnz(distMat));
disp("Number of pixel below threshold: " + nnz(indxValidSurfels));

% Go through the entire image once again to correct few of the pixels which are
% close to any valid surface.
indxInvalidSurfels = (distMat >= flyDistTh) & (distMat < flyPixCorrTh) ;
disp(['Number of flying pixel: ', num2str(nnz(indxInvalidSurfels))]);
windowLength = 2*flyWinSize+1;
correctedSurfelCount = 0;
if fixFlyPixFlag == true
    for r=flyWinSize+1:maxR-flyWinSize
        for c=flyWinSize+1:maxC-flyWinSize
            % Place the window on each pixel and look for a valid surface pixel
            % inside that window. If there is a valid pixel then update the
            % current one with the smallest valid pixel.
            if indxInvalidSurfels(r,c) == 1
                tmpIndxMat = zeros(windowLength^2, 1);
                tmpValueMat = tmpIndxMat;     	% Hold the indices
                tmpZ = tmpIndxMat;              % Hold corresponding Z values
                elementIndx = 1;
                for y=r-flyWinSize:1:r+flyWinSize
                    for x=c-flyWinSize:1:c+flyWinSize
                        tmpIndxMat(elementIndx, 1) = indxValidSurfels(y,x);
                        tmpValueMat(elementIndx, 1) = distMat(y,x);
                        tmpZ(elementIndx, 1) = data.z3D(y,x);
                        elementIndx = elementIndx + 1;
                    end
                end
                
                % If there is a valid nearby surface pixel then update the
                % current one with the nearest surface pixel.
                nonZeroElements = nnz(tmpIndxMat);
                if nonZeroElements ~= 0
                    [distMatFlySurfels(r,c), indxMin] = min(tmpValueMat);
                    data.z3D(r,c) = tmpZ(indxMin);
                    correctedSurfelCount = correctedSurfelCount + 1;
                end
            else
                continue;
            end
        end
    end
end

% Once again find all the indices and updated Z values.
indxCorrectedSurfels = distMatFlySurfels ~= 0;
indxSurfels = indxValidSurfels | indxCorrectedSurfels;
z3D = data.z3D;

disp("Total number of corrected pixels: " + correctedSurfelCount + newline);
