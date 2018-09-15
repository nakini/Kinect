function mergedPC = MapColorFrameToDepthSpace(depthImg, rgbImg, tformDepth2RGB)
% In this function, I am going to find out the RGB map(512x424) using the depth
% image(512x424) and the rgb image(1920x1080). For that, I am going to use the
% camera calibration parameters I have obtained from the calibraion process.
%
% INPUT(s):
%   depthImg    : Depth image name including the path
%   rgbImg      : Color image name with full path
%   tformDepth2RGB : Structure containing the Rotation and Translation parameters.
%   maxDepth    : Maximum depth which is set by the user
%   flyWinSize  : Window size which will be used to get rid of flying pixels
%   flyDistTh   : Threshold to determine whether to keep/discard pixels after  
%               the flying window operation
%
% OUTPUT(s):
%   mergedImg   : 512x424x3 Matrix containing the RGB values of the valid depth
%               pixels.
%
% EXAMPLE(s):
%

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
% Check the input parameters


% Read the Depth image and undistort it first

% Get all the pixel coordinates that are in the valid range.
[X, Y, Z] = Depth2World_v2(depthImg);

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

% Put the RGB values into a 512x424x3 Color image
dataUVs(dataUVs(:,2) < 0, 2) = 1;
dataUVs(dataUVs(:,2) > 1080, 2) = 1080;

% Read the RGB image and undistort it.
[rCol, cCol, ~] = size(rgbImg);
indxColImg = sub2ind([rCol, cCol], dataUVs(:,2), dataUVs(:,1));
R = rgbImg(:,:,1);
validR = R(indxColImg);
G = rgbImg(:,:,2);
validG = G(indxColImg);
B = rgbImg(:,:,3);
validB = B(indxColImg);

dataRGBs = cat(2, validR, validG, validB);
mergedPC = pointCloud(dataXYZs, 'Color', dataRGBs);