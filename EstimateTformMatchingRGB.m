function tformPC2toPC1 = EstimateTformMatchingRGB(pcStruct1, pcStruct2)
% rgbPts1, pc1, rgbPts2, pc2, tformDepth2RGB)
% In this function given the RGB points and the corresponding point clouds, I am
% going to findout the 3D points that correspond to the RGB points. Then, using
% the matching 3D points I am going to estimate the transfomation between the 
% point clouds.
%
% INPUT(s):
%   pcStruct        := Structure for each point cloud holding the following info
%       rgbPts := Nx2 matrices holding the matching RGB image indices of each
%           image
%       pc := 'pointCloud' structure holding the 3D point info for each point
%           cloud w.r.t the depth camera coordinate frame.
%       tformDepth2RGB := Structure holding the stereo calibration parameters
%           of both IR and RGB images and the transformation from Depth-to-RGB.
%   
% OUTPUT(s):
%   tformPC2toPC1   := Structure having rotation R and translation T from
%           pc2 to pc1
%
% Example(s):
%
%
%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------

% Project the points into the color frame from the depth frame using the 
% extrinsic parameters.
pc1InRGBFrame = TransformPointCloud(pcStruct1.pc, pcStruct1.tformDepth2RGB);
pc2InRGBFrame = TransformPointCloud(pcStruct2.pc, pcStruct2.tformDepth2RGB);

% Get the RGB values of those projected points in a different variable
rgbUVs1 = ProjectPointsOnImage(pc1InRGBFrame.Location, ...
    pcStruct1.tformDepth2RGB.KK_RGB);
rgbUVs1 = round(rgbUVs1);
rgbUVs2 = ProjectPointsOnImage(pc2InRGBFrame.Location, ...
    pcStruct2.tformDepth2RGB.KK_RGB);
rgbUVs2 = round(rgbUVs2);

% Search for indices of the points given by the user in the point set created in
% the previous set.
numPts = length(pcStruct1.rgbPts);
pcMatch1 = zeros(numPts, 3);
pcMatch2 = zeros(numPts, 3);
pcRowNum = 1;                   % Counter to keep track of valid points
for i = 1:numPts
    % Find the indices of matched points
    tmp1 = round(pcStruct1.rgbPts.Location(i, :));
    [r1, c1] = find(ismember(rgbUVs1, tmp1, 'rows'));
    tmp2 = round(pcStruct2.rgbPts.Location(i, :));
    [r2, c2] = find(ismember(rgbUVs2, tmp2, 'rows'));
    
    % Check whether both of the row numbers are non empty sets
    if (~isempty(r1) && ~isempty(r2))
        pcMatch1(pcRowNum, :) = pcStruct1.pc.Location(r1(1), :);
        pcMatch2(pcRowNum, :) = pcStruct2.pc.Location(r2(1), :);
        pcRowNum = pcRowNum + 1;
    end
end

% Now, estimate the rotation and translation between two point clouds
pcMatch1 = pcMatch1(1:pcRowNum-1, :);
pcMatch2 = pcMatch2(1:pcRowNum-1, :);
[R, T] = EstimateRT(pcMatch2', pcMatch1');     % The fun needs 3xN matrices

% Bundle up
tformPC2toPC1 = struct('R', R, 'T', T);