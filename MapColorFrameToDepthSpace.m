function [mergedPC, dataXYZs, dataRGBs, varargout] = MapColorFrameToDepthSpace(...
    depthImg, rgbImg, tformDepth2RGB, maxDepth, denoiseParams, mergedImg)
% In this function, I am going to find out the RGB map(512x424) using the depth
% image(512x424) and the rgb image(1920x1080). For that, I am going to use the
% camera calibration parameters I have obtained from the calibraion process.
%
% INPUT(s):
%   depthImg    : Depth image matrix
%   rgbImg      : Color image matrix
%   tformDepth2RGB  : Structure containing the Rotation and Translation parameters.
%   maxDepth    : Maximum depth which is set by the user
%   denoiseParams   : Parameters for denoising the point cloud.
%       1) flyWinSize   : Window size which will be used to get rid of flying pixels
%       2) flyDistTh    : Threshold to determine whether to keep/discard pixels after  
%                       the flying window operation
%
% OUTPUT(s):
%   mergedImg   : 512x424x3 Matrix containing the RGB values of the valid depth
%               pixels.
%
% EXAMPLE(s):
%   load(calibStereo, 'R', 'T', 'KK_left', 'KK_right');
%   tformDepth2RGB.R = inv(R);
%   tformDepth2RGB.T = -inv(R)*T/1000;
%   tformDepth2RGB.KK_RGB = KK_left;
%   tformDepth2RGB.KK_IR = KK_right;
%   denoiseParamsStruct = struct('flyWinSize', 3, 'flyDistTh', 1.2)
%
%   [~, dataXYZ, dataRGB] = MapColorFrameToDepthSpace(depthImg, ...
%     rgbImg, tformDepth2RGB, maxDepthInMeters,denoiseParamsStruct);

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
% Check the input parameters

% If we already have the merged point cloud as an RGB image then just use the
% RGB values.
if ~isempty(mergedImg)
    [X, Y, Z, Rw, Gw, Bw, indValid] = Depth2World_v2(depthImg, maxDepth, ...
        denoiseParams.flyWinSize, denoiseParams.flyDistTh, mergedImg, ...
        tformDepth2RGB.KK_IR);
    
    dataXYZs = cat(2, X, Y, Z);
    dataRGBs = cat(2, Rw, Gw, Bw);
    
    mergedPC = pointCloud(dataXYZs, 'Color', dataRGBs);
    if nargout > 3
        varargout{1} = indValid;
    end
    if nargout > 4
        varargout{2} = [];
    end
else
    
    % Get all the pixel coordinates that are in the valid range.
    [X, Y, Z] = Depth2World_v2(depthImg, maxDepth, denoiseParams.flyWinSize, ...
        denoiseParams.flyDistTh, [], tformDepth2RGB.KK_IR);
    
    % Transform all the points from the depth image to the color image coordinate
    % system
    dataXYZs = cat(2, X, Y, Z);
    pcInDepthFrame = pointCloud(dataXYZs);       % Create point cloud
    
    % Project the points into the color image frame using the intrinsic parameters
    % of the color-camera.
    pcInRGBFrame = TransformPointCloud(pcInDepthFrame, tformDepth2RGB);
    
    % Get the RGB values of those projected points in a different variable
    dataUVs = ProjectPointsOnImage(pcInRGBFrame.Location, tformDepth2RGB.KK_RGB);
    dataUVs = round(dataUVs);
    
    
    % Get rid of all the points which are out of bound. I mean, remove the pixels
    % which have indices less than 0 or more than 1080(1920).
    indxInvalid = dataUVs(:,2) <= 0 | dataUVs(:,2) >= 1080 | ...
        dataUVs(:,1) <= 0 | dataUVs(:,1) >= 1920;
%     dataUVs(dataUVs(:,2) <= 0, 2) = 1;
%     dataUVs(dataUVs(:,2) >= 1080, 2) = 1080;
    dataUVs = dataUVs(~indxInvalid, :);
    dataXYZs = dataXYZs(~indxInvalid, :);
    
    % Put the RGB values into a 512x424x3 Color image
    [rCol, cCol, ~] = size(rgbImg);
    indxColImg = sub2ind([rCol, cCol], dataUVs(:,2), dataUVs(:,1));
    R = rgbImg(:,:,1);
    validR = R(indxColImg);
    G = rgbImg(:,:,2);
    validG = G(indxColImg);
    B = rgbImg(:,:,3);
    validB = B(indxColImg);
    
    % Generate a point cloud
    dataRGBs = cat(2, validR, validG, validB);
    mergedPC = pointCloud(dataXYZs, 'Color', dataRGBs);
    
    if nargout > 3
        varargout{1} = indxColImg;
    end
    if nargout > 4
        varargout{2} = dataUVs;
    end
end