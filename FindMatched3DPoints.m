function [mtch3DIndx1, mtch3DIndx2] = FindMatched3DPoints(pcStruct1, pcStruct2)
% In this function given the RGB points and the corresponding point clouds, I am
% going to findout the 3D points that correspond to the RGB points.
%
% INPUT(s):
% 1 [pcStruct1, pcStruct2]
%   Structure for each point cloud holding the following info
%       1) rgbPts -- Nx2 matrices holding the matching RGB image pixel indices 
%   of each image
%       2) pc -- 'pointCloud' structure holding the 3D point info for each point
%   cloud w.r.t the RGB camera coordinate frame.
%       3) tformDepth2RGB -- Structure holding the stereo calibration parameters
%   of both IR and RGB images and the transformation from Depth-to-RGB.
%   
% OUTPUT(s):
% 1 [mtch3DIndx1, mtch3DIndx2]
%   Indices of 3D points of each point cloud that match with each other
%
% Example(s):
%   pcStructAnch = struct('rgbPts', inlierPtsAnch, 'pc', pcAnch, ...
%       'tformDepth2RGB', tformDepth2RGB);
%   pcStructMoved = struct('rgbPts', inlierPtsMoved, 'pc', pcMoved, ...
%       'tformDepth2RGB', tformDepth2RGB);
%   [tformMoved2Anchor, matchPtsCount, regStats] = ...
%       EstimateTformMatchingRGB(pcStructAnch, pcStructMoved);

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------

% Get the UV coordinates of those projected points on the RGB images
rgbK1 = pcStruct1.tformDepth2RGB.KK_RGB;        % Intrinsic parameters
estimatedUVs1 = ProjectPointsOnImage(pcStruct1.pc.Location, rgbK1);
estimatedUVs1 = round(estimatedUVs1);
rgbK2 = pcStruct2.tformDepth2RGB.KK_RGB;        % Intrinsic
estimatedUVs2 = ProjectPointsOnImage(pcStruct2.pc.Location, rgbK2);
estimatedUVs2 = round(estimatedUVs2);

% For each given UV point in the RGB frame, find the index of nearest estimated 
% UV point using some distance threshold with range search. Here, we consider
% the pixels that with in 2 pixel distance.
givenUVs1 = pcStruct1.rgbPts.Location;
givenUVs2 = pcStruct2.rgbPts.Location;
[mtchIndx1, mtchDist1] = knnsearch(estimatedUVs1, givenUVs1);  % 1st nearest neighbor
[mtchIndx2, mtchDist2] = knnsearch(estimatedUVs2, givenUVs2);

% Prune all the indices which are greater than the threshold distances. Set 0 to
% all indices which are beyond the threshld.
mtchIndx1(mtchDist1 > 2) = 0;
mtchIndx2(mtchDist2 > 2) = 0;

% Only save those indices of the pair of point clouds where we find a match in
% both the point clouds
commIndx = mtchIndx1 & mtchIndx2;

% Return the matched point indices
mtch3DIndx1 = mtchIndx1(commIndx);
mtch3DIndx2 = mtchIndx2(commIndx);
