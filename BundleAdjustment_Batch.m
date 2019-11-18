function [refinedStruct, rawStruct] = BundleAdjustment_Batch(dirStruct, ...
    matchInfo, matchPtsPxls, rtRaw, varargin)
% In this function, I am going to read all the point clouds along with the RGB
% images and the pair wise matching pixels and carry out the bundle adjustment
% to optimize the transformation parameters and the 3D points.
%
% INPUT(s)
% ========
% 1. dirStruct: Directory structure containing the RGB/Depth images and the
% corresponding 'ply' files and the 'rt' files.
%   1) dirName -- Name of the folder containing the depth and RGB images
%   2) plyFolderName -- Name of the folder relative to 'dirName' containing the
%   ply files.
%	3) rtFolderName -- Name of the folder relative to 'dirName' holding all the
%	text files which contain the R|T information.
%
% 2. matchInfo: Mx3 table storing RGB image names, number of matched points of 
% each image with the previous one and the "rmse" value from rigid registration.
% The count and rmse for the 1st image will be 0 and 0, respectively.
%
% 3. matchPtsPxls: Mx2 Cell storing M pairs of structures. Each structure in the
% pair, holds the following fields.
%   1) indxPC -- Px1 vector of indices of matched 3D points of point cloud
%   2) pixelsRGB -- Px2 matrix of 2D pixels of matched RGB image
%
% 4. ['calibStereo', calibStereo]: Mat-file holding stereo calibration parameters 
% between IR and RGB of the Kinect that was used to collect the data.
%
% OUTPUT(s)
% =========
% 1. refinedStruct, rawStruct: These two structs hold the following the
% non-refined and refinded info, respectively. Each structure has:
%   1) xyz -- Px3 3D points from all views
%   2) rt -- Px3 table of transformation matrices for all the views
%
% Example(s):
%
%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------

% Validate input arguments ----------------------------------------------------
p = inputParser;
p.StructExpand = false;                 % Accept strutcture as one element

% Parameters to be used to match 2 RGB images
% Calibration parameters of the Kinect sensors
defaultCalibStereo = ['~/Dropbox/PhD/Data/Calibration/Calibration_20181114/', ...
    'HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];

addRequired(p, 'dirStruct', @validateDirStruct);
addRequired(p, 'matchInfo', @validdateMatchInfo);
addRequired(p, 'matchPtsPxls', @validateMatchPtsPxls);
addRequired(p, 'rtInfo', @validateRtInfo);
addParameter(p, 'calibStereo', defaultCalibStereo, @validateCalibStereo);

p.parse(dirStruct, matchInfo, matchPtsPxls, rtRaw, varargin{:});
disp(p.Results);

% Store variales into local variables to save typing
dirName = dirStruct.dirName;
rtFolderName = dirStruct.rtFolderName;
plyFolderName = dirStruct.plyFolderName;
calibStereo = p.Results.calibStereo;
matchInfo = p.Results.matchInfo;
matchPtsPxls = p.Results.matchPtsPxls;

% Algorithm -------------------------------------------------------------------
% Create the respective lists that will hold all the 3D and 2D points for all 
% images.
totalPts = sum(matchInfo.Matched_Points);   % Total number points from all images
xyzRaw = zeros(totalPts, 3);                % 3D point list
pxlList(1, 1:totalPts) = pointTrack;        % 2D pixel list of pointTrack objs

% Go through each pair and load the transformation matrix, the point cloud
numImgs = size(matchInfo, 1);
rtRaw = cell(numImgs, 3);                  % Store R and T along with the index
rtRaw(1, :) = {1, eye(3,3), [0, 0, 0]};    % 1st image is the anchor
endIndx = 0;
for iImg = 2:numImgs                       % Ignore the 1st pair which is empty
    indxAnch = iImg - 1;
    indxMoved = iImg;
    
    numMtchPts = matchInfo.Matched_Points(indxMoved, 1);    % Number of points
    anchName = matchInfo.Name(indxAnch, 1);                 % Name of the pc
    movedName = matchInfo.Name(indxMoved, 1);
    startIndx = endIndx + 1;
    endIndx  = endIndx + numMtchPts;
    
    % Read the save point clouds
    % ==========================
    % Names of the point clouds
    tmpPlyName = [dirName, '/', plyFolderName, '/depthImg_'];
    pcFullNameAnch = [tmpPlyName, char(anchName), '.ply'];
    pcFullNameMoved = [tmpPlyName, char(movedName), '.ply'];
    
    % Load the pcs and segregate the matching points from each pair
    pcAnch = pcread(pcFullNameAnch);
    pcMoved = pcread(pcFullNameMoved);
    pcAnchMtcPts = pcAnch.Location(matchPtsPxls{iImg, 1}.indxPC, :);
    pcMovedMtcPts = pcMoved.Location(matchPtsPxls{iImg, 2}.indxPC, :);
    
	% Store the matching 3D points list
    xyzRaw(startIndx:endIndx, :) = pcMovedMtcPts;
    
    % Create pointTrack object for each pixel pair
    % ============================================
    % Save the 2D pixel points into a pointTrack class -- It takes the multiple
    % pixels from multiple views and the view-ids and creates an object.
    pxlAnch = matchPtsPxls{iImg, 1}.pixelsRGB;
    pxlMoved = matchPtsPxls{iImg, 2}.pixelsRGB;
    viewIds = [indxAnch, indxMoved];
    for iTrk = 1:numMtchPts
        pxlList(1, startIndx+iTrk-1) = pointTrack(viewIds, ...
            [pxlAnch(iTrk, :); pxlMoved(iTrk, :)]);
    end
    
    % Read R|T files
    % ==============
    % Read the rotation and translation from the text files and update the
    % "rtInfo" table
    tmpRTName = [dirName, '/', rtFolderName, '/rt_'];
    rtFullName = [tmpRTName, char(movedName), '.txt'];
    % Read the text file which will return a sturct with R(3x3), and T(3,1)
    tformMoved2Anchor = ReadRT(rtFullName);
    rtRaw(indxMoved, :) = {indxMoved, tformMoved2Anchor.R, tformMoved2Anchor.T'};
end

% Create a table for R|T
rtRaw = table(cell2mat(rtRaw(:,1)), rtRaw(:,2), rtRaw(:,3), ...
    'VariableNames', {'ViewId', 'Orientation', 'Location'});

% Load the camera intrinsic parameters and create an object
load(calibStereo, 'KK_left', 'KK_right');
KK_RGB = KK_left;
camParams = cameraParameters('IntrinsicMatrix', KK_RGB);
rtRaw.ViewId = uint32(rtRaw.ViewId);	% cameraPoses only takes IDs as uint32

% Bundle Adjustment
% =================
% Carry out the bundle adjustment using the R|T's in the form of a table,
% intrinsic parameters in the form of "cameraParameters" obj and the 3D points
% as a Nx3 matrix.
[xyzRefinedts,refinedRT] = bundleAdjustment(xyzRaw, pxlList, rtRaw, ...
    camParams, 'FixedViewIDs', 1,  'RelativeTolerance', 1e-10, 'MaxIterations', 100);

% Update R|T
% ==========
% Go through all the rt_*.txt files and update it with the current finding of
% bundle adjustments.
for iN = 1:numImgs
    % Create a file name for RT
    tmpRTName = [dirStruct.dirName, '/', dirStruct.rtFolderName, '/rt_'];
    rtNum = matchInfo.Name(iN, 1);
    rtFullName = [tmpRTName, char(rtNum), '.txt'];
    
    % Read the text file which will return a sturct with R(3x3), and T(3,1)
    tformMoved2Anchor = ReadRT(rtFullName);
    
    % Update the current RT values
    tformMoved2Anchor.R = cell2mat(refinedPoses.Orientation(iN,1));
    tformMoved2Anchor.T = cell2mat(refinedPoses.Location(iN,1))';
    
    % Write the RT values into the file
    WriteRT(tformMoved2Anchor, rtFullName); 
end
disp('Updated all the R|T values in the rt*_.txt files');

% Output:
refinedStruct = struct('xyz', xyzRefinedts, 'rt', refinedRT);
rawStruct = struct('xyz', xyzRaw, 'rt', rtRaw);
end

%% Input arguments valiating functions
function TF = validateDirStruct(dirStruct)
TF = false;
% First validate whether the structure contains the required fields or not.
if ~all(isfield(dirStruct, {'dirName', 'plyFolderName', 'rtFolderName'}))
    error('Provied the proper fields : dirName, plyFolderName and rtFolderName');
elseif ~ischar(dirStruct.dirName) && ~ischar(dirStruct.plyFolderName) && ...
        ~ischar(dirStruct.rtFolderName)
    % Then check whether the given fields are consistent with the data type that
    % is required.
    error(['The fields of the structure should be strings conaining the'...
        ' name of the folders']);
elseif ~(exist(dirStruct.dirName, 'dir')==7) || ...
        ~(exist([dirStruct.dirName, '/', dirStruct.plyFolderName], 'dir') == 7)
    % Throw an error showing telling about the missing directory
    error('The directory does not exist. Check the path onec again.');
else
    TF = true;
end

% If the subfolders doesn't exist then create the folders.
if ~(exist([dirStruct.dirName, '/', dirStruct.rtFolderName], 'dir') == 7)
    % Create the folder to save the files
    mkdir([dirStruct.dirName, '/', dirStruct.rtFolderName]);
end
end

function TF = validdateMatchInfo(matchInfo)
% This function is going to check whether the input is a table or not. And
% whether it cotains the columns "Name" and "Matched_Points" columns.
TF = false;
if ~istable(matchInfo)
    % Expecting a table
    error('This argument should be a table.');
elseif ~all(ismember({'Name', 'Matched_Points'}, matchInfo.Properties.VariableNames))
    % Expecting the table-column members as "Name" and "Matched_Points"
    error("The table should contain both the columns: 'Name' and 'Matched_Points'");
else
    TF = true;
end
end

function TF = validateMatchPtsPxls(matchPtsPxls)
% This function is going to check whether it is a cell or not, and the second
% dimention is 2 or not. It is also expecting structures as cell members. Need
% to find out a way to check each structure.
TF = false;
if ~iscell(matchPtsPxls) || ~(size(matchPtsPxls, 2) == 2)
    % Expecting a Mx2 cell containing structures
    error('It should be a Mx2 cell');
else
    TF = true;
end
end

function TF = validateRtInfo(rtInfo)
% This function is going to test whether the variable is a table or not. Also,
% it will check the fields of the table too.
TF = false;
if ~istable(rtInfo)
    error("Provide a table as an input");
elseif ~all(ismember({'ViewId', 'Orientation', 'Location'}, ...
        rtInfo.Properties.VariableNames))
    error("Provide required columns");
else
    TF = true;
end
end

function TF = validateCalibStereo(calibStereo)
TF = false;
% Is it a existing matfile or no
if exist(calibStereo, 'file') == 2
    TF = true;
else
     error('It should be a valid mat-file');
end
end
