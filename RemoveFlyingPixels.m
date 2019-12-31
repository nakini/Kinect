function [indxSurfels, z3D] = RemoveFlyingPixels(data, flyWinSize, flyDistTh, ...
    varargin)
% Get-rid-of flying pixels using the "window" method. Flying pixel is a inherent
% problem with the ToF sensors. There are another 2 methods based on the normal
% at each pixel. However, it is concluded by the authors that the window method
% is best suitable for removing the flying pixels. For more information read the
% paper: "Identification and Correction of Flying Pixels in Range Camera Data"
%
% INPUT(s)
% ========
% 1. data3D: Structure - It contains 3 mxn matrices for X, Y and Z values,
% respectively.
%
% 2. flyWinSize: Numeric value, usually 1 or 2 - It will create a bounding box
% of 2*n + 1 around each pixel.
%
% 3. flyWinTh : Float values such as 0.08, 0.1 -- All the cumulative distances
% beyond these values will be treated as flying pixels.
%
% 4. ['flyDistCorrTh', flyDistCorrTh]: Numeric value - It is percentage value
% which will be added to the "flyDistTh" value to find out the upper bound. All
% the cumulative distances between the "flyDistTh" and the upper bound will be
% corrected and the rest of the flying pixels will be ignored.
%
% 5. ['fixFlyPixFlag', fixFlyPixFlag]: ture/false - Display flag to show the
% number of points filtered out, corrected, etc.
%
% OUTPUT(s)
% =========
% 1. indxSurfels : mxn logical matrix - 1 is used for the valid pixel and the
% rest are set to 0.
%
% 2. z3D: mxn matrix - It holds the updated Z values after removing the flying
% pixels.
%
% EXAMPLE(s):
% ===========
%   data = struct('x3D', x3D, 'y3D', y3D, 'z3D', z3D);
%   [indxNoFlyPixels, z3D] = RemoveFlyingPixels(data, flyWinSize, flyDistTh, ...
%       'flyPixCorrTh', 20, 'fixFlyPixFlag', true);

%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% Validate input arguments -----------------------------------------------------
p = inputParser;
p.StructExpand = false;             % Accept structure as one element

% Compulsory parameters --
addRequired(p, 'data', @validateDataStruct);
addRequired(p, 'flyWinSize', @(x) isnumeric(x));
addRequired(p, 'flyDistTh', @(x) isnumeric(x));

% Optional parameters --
addParameter(p, 'flyDistCorrTh', 10, @(x)isstruct(x));
addParameter(p, 'fixFlyPixFlag', true, @(x) islogical(x));

p.parse(data, flyWinSize, flyDistTh, varargin{:});
disp('Given inputs for RemoveFlyingPixels() function:');
disp(p.Results);
fprintf('\n');

% Store variables into local variables to save typing :-p
flyDistCorrTh = p.Results.flyDistCorrTh;
fixFlyPixFlag = p.Results.fixFlyPixFlag;
flyPixCorrThVal = flyDistTh + flyDistCorrTh/100 * flyDistTh;

% Algorithm --------------------------------------------------------------------
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
indxInvalidSurfels = (distMat >= flyDistTh) & (distMat < flyPixCorrThVal) ;
disp("Number of flying pixel: " + nnz(indxInvalidSurfels));
windowLength = 2*flyWinSize+1;
correctedSurfelCount = 0;
if (fixFlyPixFlag == true) && (nnz(indxInvalidSurfels)>0)
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
end

%% Input arguments validating functions
function TF = validateDataStruct(data)
% First validate whether the structure contains the required fields or not.
if ~all(isfield(data, {'x3D', 'y3D', 'z3D'}))
    error('Provied the proper fields : x3D, y3D, z3D');
elseif ~ismatrix(data.x3D) && ~ismatrix(data.y3D) && ~ismatrix(data.z3D)
    error('The input arguments should be of matrix type');
else
    TF = true;
end
end
