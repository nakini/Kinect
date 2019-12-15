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
%   4) PtsPxls_Anch, PtsPxls_Moved -- Matched pixels info of the anchor and the
%   moved image, respectively. And each element of the column is a structure,
%   which holds the following fields.
%       a) indxPC -- Px1 vector of indices of matched 3D points of point cloud
%       b) pixelsRGB -- Px2 matrix of 2D pixels of matched RGB image
%   5) Anchor_ViewID, Moved_ViewID -- View ids of anchor and moved pc for each
%   pair, respectively.
%
% 3. rtRaw_Curr2Global: (M+1)x3 table
%   1) ViewId: View-id of each point cloud
%   2) Orientation: Rotation matrix corresponding to each view
%   3) Location: Translation of each view
% The last view will have 2 transformations w.r.t to the base -- One through the
% previous point cloud and the other is direct to the base pc. This will act as
% a loop closure. All the transformations will transform the points in current
% view to the base/global coordinate frame, i.e.,
%       X_g = R*X_curr + T;
%
% 4. ['calibStereo', calibStereo]: Mat-file holding stereo calibration
% parameters between IR and RGB of the Kinect that was used to collect the data.
%
% 5. ['viewCount', viewCount]: Scalar value -- Number of views that will be used
% w.r.t. the base point cloud
%
% 6. ['flagLC', flagLC]: true/false -- This flag will decide whether loop
% closure is needed to be done or not.
%
% OUTPUT(s)
% =========
% 1. refinedStruct, rawStruct: These two structs hold the following the refined
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
p.StructExpand = false;                 % Accept structure as one element

% Calibration parameters of the Kinect sensors
defaultCalibStereo = ['~/Dropbox/PhD/Data/Calibration/Calibration_20181114/', ...
    'HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
defaultFlagLC = true;       % Use loop closure
defaultViewCount = inf;     % Use all the views by default

addRequired(p, 'dirStruct', @validateDirStruct);
addRequired(p, 'matchInfo', @validdateMatchInfo);
addRequired(p, 'rtRaw_Curr2Global', @validateRtInfo);
addParameter(p, 'viewCount', defaultViewCount, @(x) isnumeric(x) && x>=2);
addParameter(p, 'flagLC', defaultFlagLC, @(x) islogical(x));
addParameter(p, 'calibStereo', defaultCalibStereo, @validateCalibStereo);

p.parse(dirStruct, matchInfo, varargin{:});
disp(p.Results);

% Store variales into local variables to save typing
dirName = dirStruct.dirName;
plyFolderName = dirStruct.plyFolderName;
calibStereo = p.Results.calibStereo;
matchInfo = p.Results.matchInfo;
rtRaw_Curr2Global = p.Results.rtRaw_Curr2Global;
viewCount = p.Results.viewCount;
flagLC = p.Results.flagLC;

% If not all the views are used then don't carry out the loop-closure.
if isinf(viewCount)
    disp('Using all the views for BA!');
else
    disp(['Using first ', num2str(viewCount), ' view!']);
    flagLC = false;
end
if flagLC == true
    disp('Loop closure will be carried out!');
else
    disp('No loop-closure will be used!');
end

% Algorithm --------------------------------------------------------------------
% Create the respective lists that will hold all the 3D and 2D points for all 
% images.
totalPts = 2*sum(matchInfo.Matched_Points); % Total number points from all images
xyzRaw_Global = zeros(totalPts, 3);         % 3D point list
pxlList(1, 1:totalPts) = pointTrack;        % 2D pixel list of pointTrack objs

numImgPairs = size(matchInfo, 1);           % Total number image pairs
rtRaw_Global2Curr  = rtRaw_Curr2Global;     % Transformation matrix for each pc

viewID_StartEnd_Indx = zeros(numImgPairs, 3);
endIndxMatchPts = 0;
% Go through each pair and read the anchor and moved point clouds. Then create
% pointTrack object for each pixel and 3D point point pairs from every given
% matching pair. Also, if needed, read the RT matrices and create a table.
for iImPrs = 1:numImgPairs
    % Following two variable act like pointer to starting and end index to keep
    % track of matching points for each pair in the long-list of whole matching
    % pairs.
    numMtchPts = matchInfo.Matched_Points(iImPrs, 1);	% Match point count in the pair
    startIndxMatchPts = endIndxMatchPts + 1;
    endIndxMatchPts  = endIndxMatchPts + 2*numMtchPts;
    % Store the start and end index of each range of points obtained from each
    % pair
    viewID_StartEnd_Indx(iImPrs, :) = [iImPrs, startIndxMatchPts, endIndxMatchPts];
    
    % Create pointTrack object for each pixel pair
    % ============================================
    % Save the 2D pixel points into a pointTrack class -- It takes the multiple
    % pixels from multiple views and the view-ids and creates an object.
    pxlAnch = matchInfo.PtsPxls_Anch{iImPrs}.pixelsRGB;
    pxlMoved = matchInfo.PtsPxls_Moved{iImPrs}.pixelsRGB;
    currViewIDs = [matchInfo.Anchor_ViewID(iImPrs), matchInfo.Moved_ViewID(iImPrs)];
    for iTrk = 1:numMtchPts
        pxlList(1, startIndxMatchPts+iTrk-1) = pointTrack(currViewIDs, ...
            [pxlAnch(iTrk, :); pxlMoved(iTrk, :)]);
    end
    pxlList(startIndxMatchPts+numMtchPts:endIndxMatchPts) = ...
        pxlList(startIndxMatchPts:startIndxMatchPts+numMtchPts-1);
    
    % Read and transform point clouds
    % ===============================
    % Names of the point clouds
    anchName = matchInfo.Anchor(iImPrs);                % Anchor pc name
    movedName = matchInfo.Moved(iImPrs);                % Moved pc name
    tmpPlyNameMoved = [dirName, '/', plyFolderName, '/depthImg_'];
    pcFullNameAnch = [tmpPlyNameMoved, char(anchName), '.ply'];
    pcFullNameMoved = [tmpPlyNameMoved, char(movedName), '.ply'];
    
    % Load the pcs, transform them into the GLOBAL coordinate frame and 
    % segregate the matching points from each pair
    pcAnch = pcread(pcFullNameAnch);
    R_Anch = rtRaw_Curr2Global.Orientation{iImPrs};
    T_Anch = rtRaw_Curr2Global.Location{iImPrs}';
    pcAnch_Global = TransformPointCloud(pcAnch, struct('R', R_Anch, 'T', T_Anch));
    pcAnchMtcPts_Global = pcAnch_Global.Location(matchInfo.PtsPxls_Anch{iImPrs}.indxPC, :);
    
    pcMoved = pcread(pcFullNameMoved);
    R_Moved = rtRaw_Curr2Global.Orientation{iImPrs+1};
    T_Moved = rtRaw_Curr2Global.Location{iImPrs+1}';
    pcMoved_Global = TransformPointCloud(pcMoved, struct('R', R_Moved, 'T', T_Moved));
    pcMovedMtcPts_Global = pcMoved_Global.Location(matchInfo.PtsPxls_Moved{iImPrs}.indxPC, :);
    
    % Store Structure and Motion
    % ==========================
    % Store global to current view transformation matrices. As we are taking the
    % inverse of the homogeneous transformation matrix, we should have used
    % inv(R) or R' for rotation, but due to Matlab's weird formats we have to
    % use R.
    rtRaw_Global2Curr.Orientation{iImPrs+1} = R_Moved;        % Expecting R' though
    rtRaw_Global2Curr.Location{iImPrs+1} = (-R_Moved'*T_Moved)';
	% Store the matching 3D point
    xyzRaw_Global(startIndxMatchPts:endIndxMatchPts, :) = ...
        vertcat(pcAnchMtcPts_Global, pcMovedMtcPts_Global);
end
% Prune data points if needed
if viewCount < numImgPairs
    % If fewer views are required instead of the entire data set then prune the
    % rest of the 3D points and 2D pixel from participating in BA
    rtRaw_Global2Curr(viewCount+1:end, :) = [];
    pxlList(viewID_StartEnd_Indx(viewCount, 2):end) = [];
    xyzRaw_Global(viewID_StartEnd_Indx(viewCount, 2):end, :) = [];
elseif flagLC == false
    % If loop closure is not needed then take out:
    %   1) direct transformation from the last view to the base.
    %   2) pixel pairs from last view and base image
    %   3) 3D points correspoding to the pixel pairs
    rtRaw_Global2Curr(end,:) = [];
    pxlList(viewID_StartEnd_Indx(end, 2):end) = [];
    xyzRaw_Global(viewID_StartEnd_Indx(end, 2):end, :) = [];
end
% cameraPoses only takes IDs as uint32
rtRaw_Global2Curr.ViewId = uint32(rtRaw_Global2Curr.ViewId);

% Load the camera intrinsic parameters and create an object -- Also another
% WEIRD thing of Matlab for which we have to take the transpose of the intrinsic
% matrix.
load(calibStereo, 'KK_left', 'KK_right');
KK_RGB = KK_left;
camParams = cameraParameters('IntrinsicMatrix', KK_RGB');

% Bundle Adjustment
% =================
% For the bundle adjustment we need all the following:
%   1) points in the global coordinate frame
%   2) transformation matrix of each view which evaluates the 3D point in
%   current view given the 3D points in global frame
%   3) intrinsic parameters of the camera which will transform the 3D points
%   in the current view into the image frame
%
% In the current scenario, use the R|T's in a table form, intrinsic parameters
% in the form of "cameraParameters" obj and the 3D points as a Nx3 matrix.
[xyzRefinedPts,refinedRTs] = bundleAdjustment(xyzRaw_Global, pxlList, ...
    rtRaw_Global2Curr, camParams, 'FixedViewIDs', 1,  'RelativeTolerance', 1e-10,...
    'MaxIterations', 1000, 'PointsUndistorted', true, 'Verbose', true);

% % Update R|T
% % ==========
% % Go through all the rt_*.txt files and update it with the current finding of
% % bundle adjustments.
% for iN = 2:numImgPairs+1
%     % Create a file name for RT
%     rtFullName = [dirName, '/', rtFolderName, '/Absolute/rt_',  ...
%         char(matchInfo.Moved(iN-1)), '_to_', char(baseName), '.txt'];
%     
%     % Update the current RT values
%     tformMoved2Anchor.R = cell2mat(refinedRT.Orientation(iN,1));
%     tformMoved2Anchor.T = cell2mat(refinedRT.Location(iN,1))';
%     
%     % Write the RT values into the file
%     WriteRT(tformMoved2Anchor, rtFullName); 
% end
% disp('Updated all the R|T values in the rt*_.txt files');

% Outputs
% =======
refinedStruct = struct('xyz', xyzRefinedPts, 'rt', refinedRTs);
rawStruct = struct('xyz', xyzRaw_Global, 'rt', rtRaw_Global2Curr);
end

%% Input arguments valiating functions
function TF = validateDirStruct(dirStruct)
% First validate whether the structure contains the required fields or not.
if ~all(isfield(dirStruct, {'dirName', 'plyFolderName', 'rtFolderName'}))
    error('Provied the proper fields : dirName, plyFolderName and rtFolderName');
elseif ~ischar(dirStruct.dirName) && ~ischar(dirStruct.plyFolderName) && ...
        ~ischar(dirStruct.rtFolderName)
    % Then check whether the given fields are consistent with the data type that
    % is required.
    error(['The fields of the structure should be strings containing the'...
        ' name of the folders']);
elseif ~(exist(dirStruct.dirName, 'dir')==7) || ...
        ~(exist([dirStruct.dirName, '/', dirStruct.plyFolderName], 'dir') == 7)
    % Throw an error showing telling about the missing directory
    error('The directory does not exist. Check the path once again.');
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
% whether it contains the columns "Name" and "Matched_Points" columns.
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

function TF = validateRtInfo(rtRaw_Curr2Global)
% This function is going to test whether the variable is a table or not. Also,
% it will check the fields of the table too.
TF = false;
if ~istable(rtRaw_Curr2Global)
    error("Provide a table as an input");
elseif ~all(ismember({'ViewId', 'Orientation', 'Location'}, ...
        rtRaw_Curr2Global.Properties.VariableNames))
    error("Provide required columns");
else
    TF = true;
end
end

function TF = validateCalibStereo(calibStereo)
% Is it a existing matfile or no
if exist(calibStereo, 'file') == 2
    TF = true;
else
     error('It should be a valid mat-file');
end
end
