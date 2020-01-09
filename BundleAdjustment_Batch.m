function [refinedStruct, rawStruct] = BundleAdjustment_Batch(dirStruct, ...
    matchPairWise, rtRawCurr2Global, varargin)
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
% 2. matchPairWise: Mx8 table
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
% 3. rtRawCurr2Global: Px3 table
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
% 5. ['seqViewIDsFlag', seqViewIDsFlag]: Logical value -- If it is true then all
% the view numbers will be replaced with sequential numbers.
%
% 6. ['optParamsBA', optParamsBA]: Struct -- It has all the optional parameters
% for bundleAdjustment() function. To get the valid parameters take a look at
% the Name|Value pair of bundleAdjustment().
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

% Compulsory parameters --
% Calibration parameters of the Kinect sensors
addRequired(p, 'dirStruct', @validateDirStruct);
addRequired(p, 'matchPairWise', @validdateMatchPairWise);
addRequired(p, 'rtRawCurr2Global', @validateRTCurr2Global);

% Optional Parameters --
defaultCalibStereo = ['~/Dropbox/PhD/Data/Calibration/Calibration_20181114/', ...
    'HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
defaultOptParamsBA = struct('FixedViewIDs', 1,  'RelativeTolerance', 1e-10, ...
    'MaxIterations', 1000, 'PointsUndistorted', true, 'Verbose', true, ...
    'AbsoluteTolerance', 0.1);          % Optional Bundle adjustment parameters
addParameter(p, 'calibStereo', defaultCalibStereo, @validateCalibStereo);
addParameter(p, 'seqViewIDsFlag', false, @(x) islogical(x));
addParameter(p, 'optParamsBA', defaultOptParamsBA, @validateOptParamsBA);

p.parse(dirStruct, matchPairWise, rtRawCurr2Global, varargin{:});
disp(p.Results);

% Store variales into local variables to save typing
dirName = dirStruct.dirName;
plyFolderName = dirStruct.plyFolderName;
calibStereo = p.Results.calibStereo;
matchPairWise = p.Results.matchPairWise;
rtRawCurr2Global = p.Results.rtRawCurr2Global;
optParamsBA = p.Results.optParamsBA;
seqViewIDsFlag = p.Results.seqViewIDsFlag;
% Update the Anchor and Moved view-ids with the sequential numbers. This doesn't
% improve anything in the optimization, just makes the plots little less
% cluttered and more attracting.
if seqViewIDsFlag
    imgNum = rtRawCurr2Global.ViewId;
    imgNumsUnq = unique(imgNum);
    seqNums = 1:length(imgNumsUnq);
    anchViewIDs = matchPairWise.Anchor_ViewID;
    movedViewIDs = matchPairWise.Moved_ViewID;
    for iIN = 1:length(imgNumsUnq)
        anchViewIDs(anchViewIDs == imgNumsUnq(iIN)) = seqNums(iIN);
        movedViewIDs(movedViewIDs == imgNumsUnq(iIN)) = seqNums(iIN);
        imgNum(imgNum == imgNumsUnq(iIN)) = seqNums(iIN);
    end
    matchPairWise.Anchor_ViewID = anchViewIDs;
    matchPairWise.Moved_ViewID = movedViewIDs;
end
% Algorithm --------------------------------------------------------------------
% Create the respective lists that will hold all the 3D and 2D points for all 
% images.
totalPts = 2*sum(matchPairWise.Matched_Points); % Total number points from all images
xyzRaw_Global = zeros(totalPts, 3);             % 3D point list
pxlList(1, 1:totalPts) = pointTrack;            % 2D pixel list of pointTrack objs

numImgPairs = size(matchPairWise, 1);           % Total number image pairs

viewID_StartEnd_Indx = zeros(numImgPairs, 3);
endIndxMatchPts = 0;
% Go through each pair and read the anchor and moved point clouds. Then create
% pointTrack object for each pixel and 3D point point pairs from every given
% matching pair. Also, if needed, read the RT matrices and create a table.
for iImPrs = 1:numImgPairs
    % Following two variable act like pointer to starting and end index to keep
    % track of matching points for each pair in the long-list of whole matching
    % pairs.
    numMtchPts = matchPairWise.Matched_Points(iImPrs, 1);   % Match point count in the pair
    startIndxMatchPts = endIndxMatchPts + 1;
    endIndxMatchPts  = endIndxMatchPts + 2*numMtchPts;
    % Store the start and end index of each range of points obtained from each
    % pair
    viewID_StartEnd_Indx(iImPrs, :) = [iImPrs, startIndxMatchPts, endIndxMatchPts];
    
    % Create pointTrack object for each pixel pair
    % ============================================
    % Save the 2D pixel points into a pointTrack class -- It takes the multiple
    % pixels from multiple views and the view-ids and creates an object.
    pxlAnch = matchPairWise.PtsPxls_Anch{iImPrs}.pixelsRGB;
    pxlMoved = matchPairWise.PtsPxls_Moved{iImPrs}.pixelsRGB;
    currViewIDs = [matchPairWise.Anchor_ViewID(iImPrs), ...
        matchPairWise.Moved_ViewID(iImPrs)];
    for iTrk = 1:numMtchPts
        pxlList(1, startIndxMatchPts+iTrk-1) = pointTrack(currViewIDs, ...
            [pxlAnch(iTrk, :); pxlMoved(iTrk, :)]);
    end
    % As we are including both the "moved" and "acnhor" point clouds, we need
    % the corresponding pixels too. However, the pixels are the same for both
    % the point clouds. So, we need to append "the same" pixels once again.
    pxlList(startIndxMatchPts+numMtchPts:endIndxMatchPts) = ...
        pxlList(startIndxMatchPts:startIndxMatchPts+numMtchPts-1);
    
    % Read and transform point clouds
    % ===============================
    % Names of the point clouds
    anchName = matchPairWise.Anchor{iImPrs};                % Anchor pc name
    movedName = matchPairWise.Moved{iImPrs};                % Moved pc name
    tmpPlyNameMoved = [dirName, '/', plyFolderName, '/depthImg_'];
    pcFullNameAnch = [tmpPlyNameMoved, anchName, '.ply'];
    pcFullNameMoved = [tmpPlyNameMoved, movedName, '.ply'];
    
    % Load the pcs, transform them into the GLOBAL coordinate frame and 
    % segregate the matching points from each pair
    pcAnch = pcread(pcFullNameAnch);
    anchR_indx = rtRawCurr2Global.ViewId == str2num(anchName);
    tmpR = rtRawCurr2Global.Orientation(anchR_indx, :);
    tmpT = rtRawCurr2Global.Location(anchR_indx, :);
    R_Anch = tmpR{1}';              % There could be many R|T for a view, choose 1st
    T_Anch = tmpT{1}';
    pcAnch_Global = TransformPointCloud(pcAnch, struct('R', R_Anch, 'T', T_Anch));
    validMatchIndx = matchPairWise.PtsPxls_Anch{iImPrs}.indxPC;
    pcAnchMtcPts_Global = pcAnch_Global.Location(validMatchIndx, :);
    
    pcMoved = pcread(pcFullNameMoved);
    movedR_indx = rtRawCurr2Global.ViewId == str2num(movedName);
    tmpR = rtRawCurr2Global.Orientation(movedR_indx, :);
    tmpT = rtRawCurr2Global.Location(movedR_indx, :);
    R_Moved = tmpR{1}';              % There could be many R|T for a view, choose 1st
    T_Moved = tmpT{1}';
    pcMoved_Global = TransformPointCloud(pcMoved, struct('R', R_Moved, 'T', T_Moved));
    validMatchIndx = matchPairWise.PtsPxls_Moved{iImPrs}.indxPC;
    pcMovedMtcPts_Global = pcMoved_Global.Location(validMatchIndx, :);
    
    % Store Structure and Motion
    % ==========================
    % The bundleAdjustment() function takes the 3D points in the global
    % coordinate frame and the transformation of each camera is from "Current
    % Camera view -- to -- Global view" which we already have. So, we just need
    % to store the matching 3D point.
    xyzRaw_Global(startIndxMatchPts:endIndxMatchPts, :) = ...
        vertcat(pcAnchMtcPts_Global, pcMovedMtcPts_Global);
end
if seqViewIDsFlag
    rtRawCurr2Global.ViewId = imgNum;
end
% Remove extra R|T from the table. %% TODO: need to prunes these values while
% creating them in "EstimateTform_Batch()" function.
% [~, indxB] = ismember(seqNums, imgNum);
% viewIDs = rtRawCurr2Global.ViewId(indxB);
% matRs = rtRawCurr2Global.Orientation(indxB);
% vectTs = rtRawCurr2Global.Location(indxB);
% rtRawCurr2Global = table(viewIDs, matRs, vectTs, ...
%     'VariableNames', {'ViewId', 'Orientation', 'Location'});

% cameraPoses only takes IDs as uint32
rtRawCurr2Global.ViewId = uint32(rtRawCurr2Global.ViewId);

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
    rtRawCurr2Global, camParams, optParamsBA);

% Outputs
% =======
refinedStruct = struct('xyz', xyzRefinedPts, 'rt', refinedRTs);
rawStruct = struct('xyz', xyzRaw_Global, 'rt', rtRawCurr2Global);
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

function TF = validdateMatchPairWise(matchInfo)
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

function TF = validateRTCurr2Global(rtRaw_Curr2Global)
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

function TF = validateOptParamsBA(optParamsBA)
if ~isstruct(optParamsBA)
    error('Provide a structure with bundleAdjustment() optional parameters');
else
    TF = true;
end
end
