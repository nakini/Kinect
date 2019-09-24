function [indxSurfels, z3D] = RemoveFlyingPixels(data, flyWinSize, flyDistTh) 
% Get-rid-of flying pixels using the "window" method. Flying pixel is a inherent
% problem with the ToF sensors. There are another 2 methods based on the normal
% at each pixel. However, it is concluded by the authors that the window method
% is best suitable for removing the flying pixels. For more information read the
% paper: "Identification and Correction of Flying Pixels in Range Camera Data"
%
% INPUT(s):
%   data3D      : Structure containing X, Y and Z values for each pixel
%   flyWinSize  : Window size, usually 1 or 2
%   flyWinTh    : Flot values such as 0.08, 0.1, etc.
%
% OUTPUT(s):
%   indxSurfels : Valid indices
%   z3D         : Updated data values
%
% EXAMPLE(s):
%

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
[maxR, maxC] = size(data.x3D);          % Image matrix size
distMat = zeros(maxR, maxC);            % Matrix for dist values

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
indxSurfels = (distMat > 0) & (distMat < flyDistTh);

% Go through the entire image once again to correct few of the pixels which are
% close to any valid surface.
indxInvalidSurfels = distMat > flyDistTh;
windowLength = 2*flyWinSize*+1;
for r=flyWinSize+1:maxR-flyWinSize
    for c=flyWinSize+1:maxC-flyWinSize
        if indxInvalidSurfels(y,x) == 1
            % Move the window on each pixel and find if there is any nearby
            % surface pixel.
            tmpIndxMat = zeros(windowLength * windowLength, 1);
            tmpValueMat = tmpIndxMat;
            tmpZ = tmpIndxMat;
            tmpCount = 1;
            for y=r-flyWinSize:1:r+flyWinSize
                for x=c-flyWinSize:1:c+flyWinSize
                    tmpIndxMat(tmpCount, 1) = indxSurfels(y,x);
                    tmpValueMat(tmpCount, 1) = distMat(y,x);
                    tmpZ(tmpCount, 1) = data.z3D(y,x);
                    tmpCount = tmpCount + 1;
                end
            end
            
            % If there is a valid nearby surface pixel then update the current
            % one with the nearest surface pixel.
            nonZeroElements = nnz(tmpIndxMat);
            if nonZeroElements ~= 0
                [distMat(r,c), indxMin] = min(tmpValueMat);
                data.z3D(r,c) = tmpZ(indxMin);
            end
        else
            continue;
        end
        
    end
end

% Once again find all the indices and updated Z values.
indxSurfels = (distMat > 0) & (distMat < flyDistTh);
z3D = data.z3D;
