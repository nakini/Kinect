function indxSurfels = RemoveFlyingPixels(data, flyWinSize, flyDistTh) 
% Get-rid-of flying pixels using the "window" method. Flying pixel is a inherent
% problem with the ToF sensors. There are another 2 methods based on the normal
% at each pixel. However, it is concluded by the authors that the window method
% is best suitable for removing the flying pixels. For more information read the
% paper: "Identification and Correction of Flying Pixels in Range Camera Data"
%
% INPUT(s):
%   data3D      :
%   flyWinSize  :
%   flyWinTh    :
%
% OUTPUT(s):
%   indxSurfels :

[maxR, maxC] = size(data.x3D);          % Image matrix size
distMat = zeros(maxR, maxC);            % Matrix for dist values
tmpDist = 0;
for r=flyWinSize+1:maxR-flyWinSize
    for c=flyWinSize+1:maxC-flyWinSize
        for y=r-flyWinSize:1:r+flyWinSize
            for x=c-flyWinSize:1:c+flyWinSize
                tmpDist = abs(norm([data.x3D(r,c) - data.x3D(y,x), ...
                    data.y3D(r,c)-data.y3D(y,x), ...
                    data.z3D(r,c)-data.z3D(y,x)]));
            end
        end
        
        distMat(r,c) = tmpDist;
    end
end
% Set the threshold to remove all the flying pixels.
indxSurfels = (distMat > 0) & (distMat < flyDistTh);