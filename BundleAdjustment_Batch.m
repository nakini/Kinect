function [refinedStruct, rawStruct] = BundleAdjustment_Batch(dirStruct, ...
    matchInfo, varargin)
% In this function, I am going to read all the point clouds along with the RGB
% images and the pair wise matching pixels and carry out the bundle adjustment
% to optimize the transformation parameters and the 3D points. This includes the
% "loop-clouser" too.
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
% 2. matchInfo: Mx8 table
%   1) Anchor, Moved -- Image names of the anchor and moved image, respectively
%   2) Matched_Points -- Number of matched points of between two images
%   3) ICP_RMSE -- the "rmse" value from rigid registration
%   4) PtsPxls_Anch, PtsPxls_Moved -- Matched pixles info of the anchor and the
%   moved image, respectively. And each element of the column is a structure,
%   which holds the following fields.
%       a) indxPC -- Px1 vector of indices of matched 3D points of point cloud
%       b) pixelsRGB -- Px2 matrix of 2D pixels of matched RGB image
%   5) Anchor_ViewID, Moved_ViewID -- View ids of anchor and moved pc for each
%   pair, respectively.
%
% 3. ['rtRaw', rtRaw]: (M+1)x3 table
%   1) ViewId: View-id of each point cloud
%   2) Orientation: Rotation matrix corresponding to each view
%   3) Location: Translation of each view
% The last view will have 2 transformations w.r.t to the base -- One through the
% previous point cloud and the other is direct to the base pc.
%
% 4. ['calibStereo', calibStereo]: Mat-file holding stereo calibration
% parameters between IR and RGB of the Kinect that was used to collect the data.
%
% OUTPUT(s)
% =========
% 1. refinedStruct, rawStruct: These two structs hold the following the refinded
% and non-refined info, respectively. Each structure has:
%   1) xyz -- Px3 3D points from all views
%   2) rt -- Px3 table of transformation matrices for all the views
%
% Example(s)
% ==========
%
%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% Validate input arguments -----------------------------------------------------
p = inputParser;
p.StructExpand = false;                 % Accept strutcture as one element

% Calibration parameters of the Kinect sensors
defaultCalibStereo = ['~/Dropbox/PhD/Data/Calibration/Calibration_20181114/', ...
    'HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
defaultRtFalg = true;                   % Assume input RTs are not given by user

addRequired(p, 'dirStruct', @validateDirStruct);
addRequired(p, 'matchInfo', @validdateMatchInfo);
addParameter(p, 'rtRaw', defaultRtFalg, @validateRtInfo);
addParameter(p, 'calibStereo', defaultCalibStereo, @validateCalibStereo);

p.parse(dirStruct, matchInfo, varargin{:});
disp(p.Results);

% Store variales into local variables to save typing
dirName = dirStruct.dirName;
rtFolderName = dirStruct.rtFolderName;
plyFolderName = dirStruct.plyFolderName;
calibStereo = p.Results.calibStereo;
matchInfo = p.Results.matchInfo;
rtRaw = p.Results.rtRaw;
if ~istable(rtRaw)
    defaultRtFalg = false;
    disp('Reading RTs from the text files');
else
    disp('Reading RTs provided by the user');
end

% Algorithm --------------------------------------------------------------------
% Create the respective lists that will hold all the 3D and 2D points for all 
% images.
totalPts = sum(matchInfo.Matched_Points);   % Total number points from all images
xyzRaw = zeros(totalPts, 3);                % 3D point list
pxlList(1, 1:totalPts) = pointTrack;        % 2D pixel list of pointTrack objs

% Go through each pair and load the transformation matrix, the point cloud
numImgs = size(matchInfo, 1);

% Container to store R and T values along with the veiw-id of each view if it is
% not provided by the user. Also add one more view to close the loop.
if defaultRtFalg == false
    rtRaw = cell(numImgs+1, 3);
    rtRaw(1, :) = {1, eye(3,3), [0, 0, 0]};     % 1st image is the anchor
end

endIndxMatchPts = 0;
baseName = matchInfo.Anchor(1);             % Every pc is represented w.r.t 1st one
for iImg = 1:numImgs
    numMtchPts = matchInfo.Matched_Points(iImg, 1);     % Number of points
    anchName = matchInfo.Anchor(iImg);                  % Name of the pc
    movedName = matchInfo.Moved(iImg);
    startIndxMatchPts = endIndxMatchPts + 1;
    endIndxMatchPts  = endIndxMatchPts + numMtchPts;
    
    % Read the save point clouds
    % ==========================
    % Names of the point clouds
    tmpPlyName = [dirName, '/', plyFolderName, '/depthImg_'];
    pcFullNameAnch = [tmpPlyName, char(anchName), '.ply'];
    pcFullNameMoved = [tmpPlyName, char(movedName), '.ply'];
    
    % Load the pcs and segregate the matching points from each pair
    pcAnch = pcread(pcFullNameAnch);
    pcMoved = pcread(pcFullNameMoved);
    pcAnchMtcPts = pcAnch.Location(matchInfo.PtsPxls_Anch{iImg}.indxPC, :);
    pcMovedMtcPts = pcMoved.Location(matchInfo.PtsPxls_Moved{iImg}.indxPC, :);
    
	% Store the matching 3D points list
    xyzRaw(startIndxMatchPts:endIndxMatchPts, :) = pcMovedMtcPts;
    
    % Create pointTrack object for each pixel pair
    % ============================================
    % Save the 2D pixel points into a pointTrack class -- It takes the multiple
    % pixels from multiple views and the view-ids and creates an object.
    pxlAnch = matchInfo.PtsPxls_Anch{iImg}.pixelsRGB;
    pxlMoved = matchInfo.PtsPxls_Moved{iImg}.pixelsRGB;
    currViewIDs = [matchInfo.Anchor_ViewID(iImg), matchInfo.Moved_ViewID(iImg)];
    for iTrk = 1:numMtchPts
        pxlList(1, startIndxMatchPts+iTrk-1) = pointTrack(currViewIDs, ...
            [pxlAnch(iTrk, :); pxlMoved(iTrk, :)]);
    end
    
    % Read R|T files
    % ==============
    % Read the rotation and translation from the text files if the "rtRaw" table
    % is not provided by the user.
    if defaultRtFalg == false
        rtFullName = [dirName, '/', rtFolderName, '/Absolute/rt_', ...
            char(movedName), '_to_', char(baseName), '.txt'];
        % Read the text file which will return a sturct with R(3x3), and T(3,1)
        [tformMoved2Anchor, fileReadFlag] = ReadRT(rtFullName);
        if fileReadFlag == true
            rtRaw(iImg+1, :) = {matchInfo.Moved_ViewID(iImg), ...
                tformMoved2Anchor.R, tformMoved2Anchor.T'};
        else
            error("Unable to read the RT file!!!");
        end
    end
end

% Create a table for R|T if not provided by the user
if defaultRtFalg == false
    rtRaw = table(cell2mat(rtRaw(:,1)), rtRaw(:,2), rtRaw(:,3), ...
        'VariableNames', {'ViewId', 'Orientation', 'Location'});
end

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
for iN = 2:numImgs+1
    % Create a file name for RT
    rtFullName = [dirName, '/', rtFolderName, '/Absolute/rt_',  ...
        char(matchInfo.Moved(iN-1)), '_to_', char(baseName), '.txt'];
    
    % Update the current RT values
    tformMoved2Anchor.R = cell2mat(refinedRT.Orientation(iN,1));
    tformMoved2Anchor.T = cell2mat(refinedRT.Location(iN,1))';
    
    % Write the RT values into the file
    WriteRT(tformMoved2Anchor, rtFullName); 
end
disp('Updated all the R|T values in the rt*_.txt files');

% Outputs
% =======
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
elseif ~all(ismember({'Anchor', 'Moved', 'Matched_Points'}, ...
        matchInfo.Properties.VariableNames))
    % Expecting the table-column members as "Name" and "Matched_Points"
    error("The table should contain the columns: 'Anchor', 'Moved' and 'Matched_Points'");
else
    TF = true;
end
end

function TF = validateRtInfo(rtRaw)
% This function is going to test whether the variable is a table or not. Also,
% it will check the fields of the table too.
TF = false;
if ~istable(rtRaw)
    error("Provide a table as an input");
elseif ~all(ismember({'ViewId', 'Orientation', 'Location'}, ...
        rtRaw.Properties.VariableNames))
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
