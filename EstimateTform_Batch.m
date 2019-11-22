function [matchInfo, matchPtsPxls, rtInfo] = EstimateTform_Batch(dirStruct, ...
    imgNumStruct, varargin)
% In this function, I am going to read two images from a given folder then
% estimate the matching between the two using the RGB images. For all the
% matched pixels I will figure out the corresponding 3D points for each RGB
% image. Then, I will use these 3D points pairs as the matching pairs to figure
% out the transformation between two points clouds.
%
% For the matching part, I will have two methods to find out the corner points
% in the image -- 1) Use one of the those automatic feature detection technique 
% or 2) Project the 3D points onto the RGB image and use them as the feature
% points.
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
% 2. imgNumStruct: Structure holding the stand and end number of a sequence of 
% images which need to be processed
%	1) startIndx -- File number of 1st pc
%   2) endIndx -- Last file number
%
% 3. ['geomParamsStruct', geomParamsStruct]: Parameters for matching the two 
% images and removing outlier in pair of matching points. (For more information,
% look at the 'estimateGeometricTransform()' help document.
%   1) tformType -- It could be similarity|affine|projective
%   2) maxDist -- Maximum distance from point to projection
%
% 4. ['calibStereo', calibStereo]: Mat-file holding stereo calibration parameters 
% between IR and RGB of the Kinect that was used to collect the data.
%
% 5. ['dispFlag', dispFlag]: Flag to display or not the matched pixels in the 
% image pair and the final transformed point cloud pair
%	1) pcPair -- [0]/1 to display the final registered point clouds
%   2) matchPair -- [0]/1 to display matched pair of pixels
%
% 6. ['cornerTech', cornerTech]: Either automatic (SURF|Harris) or manual 
% (UserDefined)
%
% 7. ['regRigidStruct', regRigidParams]: Parameters need for carrying out a 
% rigid registration method on two given point cloud.
%	1) maxIter -- Maximum number of iteration
%   2) icpMethod -- This could be pointToPlane | pointToPoint | ndt3D
%
% OUTPUT(s)
% =========
% 1. matchInfo: Mx4 table
%   1) Anchor, Moved -- Image names of the anchor and moved image, respectively
%   2) Matched_Points -- Number of matched points of between two images
%   3) ICP_RMSE -- the "rmse" value from rigid registration
%   4) PtsPxls_Anch, PtsPxls_Moved -- Matched pixles info of the anchor and the
%   moved image, respectively. And each element of the column is a structure,
%   which holds the following fields.
%       a) indxPC -- Px1 vector of indices of matched 3D points of point cloud
%       b) pixelsRGB -- Px2 matrix of 2D pixels of matched RGB image
%
% 2. rtInfo: Mx3 table
%   1) ViewID -- View number (1 through number of images)
%   2) Orientation -- Rotation matrix w.r.t anchor/pc
%   3) Translation -- Translation vector w.r.t. anchor image/pc
%
% Example(s)
% ==========
%   dirStruct = struct('dirName', '~/Dropbox/PhD/Data/Data/Alvaro/2017_0825/103/
%       SampleImages/', 'plyFolderName', 'PCinPLY_woPlane_Testing', 
%       'rtFolderName', 'PCinXYZNorTri_woPlane_Testing');
%   mtchPts = EstimateTform_Batch(dirStruct, imgNumStruct);

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------

% Validate input arguments ----------------------------------------------------
p = inputParser;
p.StructExpand = false;             % Accept structure as one element

% Parameters to be used to match 2 RGB images
defaultGeomParams = struct('tformType', 'projective', 'maxDist', 3.5);
% Calibration parameters of the Kinect sensors
defaultCalibStereo = ['~/Dropbox/PhD/Data/Calibration/Calibration_20181114/', ...
    'HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
% Display pair of point clouds.
defaultFlags = struct('matchPair', false, 'pcPair', false);
% Corner detection technique --
validCornerOptions = {'SURF', 'Harris', 'UserDefined'};
defaultCorner = 'SURF';
validateCorner = @(x) any(validatestring(x, validCornerOptions));
% Regid registration parameters
defaultRegRigidParams =  struct('maxIter', 20, 'icpMethod', 'pointToPlane');

addRequired(p, 'dirStruct', @validateDirStruct);
addRequired(p, 'imgNumStruct', @validateImgNumStruct);
addParameter(p, 'geomParamsStruct', defaultGeomParams, @validateGeomParams);
addParameter(p, 'calibStereo', defaultCalibStereo, @validateCalibStereo);
addParameter(p, 'dispFlag', defaultFlags, @validateDispFlag);
addParameter(p, 'cornerTech', defaultCorner, validateCorner);
addParameter(p, 'regrigidStruct', defaultRegRigidParams, @validateRegRigidParams);

p.parse(dirStruct, imgNumStruct, varargin{:});
disp(p.Results);

% Store variables into local variables to save typing :-p
dirName = dirStruct.dirName;
rtFolderName = dirStruct.rtFolderName;
plyFolderName = dirStruct.plyFolderName;
geomParamsStruct = p.Results.geomParamsStruct;
calibStereo = p.Results.calibStereo;
dispFlag = p.Results.dispFlag;
cornerTech = p.Results.cornerTech;
regrigidStruct = p.Results.regrigidStruct;

% Algorithm -------------------------------------------------------------------
% First I am going to go through all the files in the given range and create a
% list.
fileList = [];                  % Store a list of image files with complete path
fileNumbers = [];               % Store the numbers given to the images
imgIndx = 1;                    % Counter
% Create a list of files
% ======================
% Here, we will create a list of files and in the end of the list we will append
% the 1st number to complere the loop.
while imgNumStruct.startIndx < imgNumStruct.endIndx
    imgNum = imgNumStruct.startIndx;
    rgbName = ['rgbImg_', num2str(imgNum), '.jpg'];
    rgbFullName = [dirStruct.dirName, '/', rgbName];
    if exist(rgbFullName, 'file') == 2
        fileList{imgIndx} = rgbFullName;
        fileNumbers(imgIndx) = imgNum;
        imgIndx = imgIndx + 1;
    end
    imgNumStruct.startIndx = imgNumStruct.startIndx + 1;   % Check the next one
end
fileList{imgIndx} = fileList{1};	% Add the 1st to the end of the list
fileNumbers(imgIndx) = fileNumbers(1);

numImgs = length(fileList)-1;       % Total number of images
matchPtsCount = zeros(numImgs, 1);  % Store matching points
regRigidError = zeros(numImgs, 1);  % Hold the error from ICP algorithm
imgName = cell(numImgs, 2);         % Name of matching image pair
matchPtsPxls = cell(numImgs, 2);    % Holds pair of structures

% Store R and T along with the view index. Also set the default values for the
% 1st image/pc.
rtInfo = cell(numImgs+1, 4);        % Also includes the R|T of 1st image
rtFromTo = [num2str(fileNumbers(1)), '_to_', num2str(fileNumbers(1))];
rtInfo(1, :) = {1, eye(3,3), [0, 0, 0], rtFromTo};
% Also save the same into a file
rtNameBase = ['rt_', rtFromTo, '.txt'];
rtFullNameBase = [dirName, '/', rtFolderName, '/', rtNameBase];
WriteRT(struct('R', eye(3,3), 'T', zeros(3,1)), rtFullNameBase);

% If there is only 1 image then then there is no point in finding the matches.
if numImgs < 2
    disp('Only few images found, so operation aborted');
    return
end

% Load Calibration Parameters
% ===========================
% Before going through each pair of images and finding out the correspondences,
% read the calibration parameters and create a structure to holding 
% transformation matrix, etc...
tformDepth2RGB = TformMatFromCalibration(calibStereo);

% Iterate through the list
% ========================
% File name that is going to store all the log information.
logFileName = [dirName, '/', rtFolderName, '/Log-', date, '.txt'];
logString = string(['Processings started at: ', datestr(datetime), '\n']);
LogInfo(logFileName, logString);

% Load a pair of images at a time and find the correspondence. The first image
% will be the anchor image and the 2nd will be the moved point cloud
for iNum = 1:numImgs
    % For the last image pair, make the last+1 (which is in fact the 1st images)
    % as the anchor image
    if iNum == numImgs              % Only for the LAST pair
        anchIndx = iNum+1;
        movedIndx = iNum;
    else                            % For the rest of the pairs
        anchIndx = iNum;
        movedIndx = iNum + 1;
    end

    % Read the 1st RGB image & PC
    % ===========================
    % Here, we are also going to read corresponding rt*.txt file if exists. If
    % not we are going to crate one.
    anchNum = fileNumbers(anchIndx);
    rgbFullNameAnch = fileList{anchIndx};
    rgbImgAnch = imread(rgbFullNameAnch);   % Read 1st image
    % Read teh 2st point cloud corresponding to the RGB image
    pcNameAnch = ['depthImg_', num2str(anchNum), '.ply'];
    pcFullNameAnch = [dirName, '/', plyFolderName, '/', pcNameAnch];
    pcAnch = pcread(pcFullNameAnch);        % 1st point cloud
    
    % Read the 2nd RGB image & PC
    % ===========================
    movedNum = fileNumbers(movedIndx);
    % Read the 2nd point cloud
    rgbFullNameMoved = fileList{movedIndx};
    rgbImgMoved = imread(rgbFullNameMoved); % Read 2nd image
    pcNameMoved = ['depthImg_', num2str(movedNum), '.ply'];
    pcFullNameMoved = [dirName, '/', plyFolderName, '/', pcNameMoved];
    pcMoved = pcread(pcFullNameMoved);      % 2nd point cloud
    
    % Display the image names that were supposed to be matched
    rgbNameAnch = ['rgbImg_', num2str(anchNum), '.jpg'];
    rgbNameMoved = ['rgbImg_', num2str(movedNum), '.jpg'];
    imgName{iNum, 1} = num2str(anchNum);    % Store the names
    imgName{iNum, 2} = num2str(movedNum);
    fprintf('Matching -- %s and %s\n\n', rgbNameAnch, rgbNameMoved);
    
    % Match two RGB images
    % ====================
    % Detect corners either  automatically (SURF | Harris) or provide them by 
    % projecting the point clouds onto the RGB images.
    if strcmpi(cornerTech, 'SURF') || strcmpi(cornerTech, 'Harris')
        % This method uses the automatic corner detection technique, so we don't
        % have to find out the 2D point.
        points2DAnch = [];
        points2DMoved = [];
    elseif strcmpi(cornerTech, 'UserDefined')
        % In this case, we are not using the any automatic corner detection 
        % technique. Instead, we are providing the projection of 3D point on the
        % RGB image as the corner points.
        points2DAnch = ProjectPCs2RGBImage(pcAnch, tformDepth2RGB);
        points2DMoved = ProjectPCs2RGBImage(pcMoved, tformDepth2RGB);
    else
        error('Matching type should be SURF | Harris | UserDefined');
    end
    
    % Find the matching point between two images.
    matchingStruct = struct('technique', cornerTech, 'points1', points2DAnch , ...
        'points2', points2DMoved);
    [inlierPtsAnch, inlierPtsMoved] = FindMatched2DPixels(rgbImgAnch, ...
        rgbImgMoved, matchingStruct, geomParamsStruct, dispFlag.matchPair);
    
    % Match corresponding Point Clouds
    % ================================
    % Estimate the initial guess for transformation between the two point cloud
    % using the RGB matching points.
    pcStructAnch = struct('rgbPts', inlierPtsAnch, 'pc', pcAnch, ...
        'tformDepth2RGB', tformDepth2RGB);
    pcStructMoved = struct('rgbPts', inlierPtsMoved, 'pc', pcMoved, ...
        'tformDepth2RGB', tformDepth2RGB);
    [mtchAnchStct, mtchMovedStct] = FindMatched3DPoints(pcStructAnch, ...
        pcStructMoved);
    matchPtsCount(iNum, 1) = size(mtchAnchStct.indxPC, 1);
    matchPtsPxls(iNum, :) = {mtchAnchStct, mtchMovedStct};
    
    % Find the transformation
    % =======================
    % Using the matching 3D point pairs find the coarse transformation between
    % the two point clouds and if needed do a fine registration using standard
    % registration techniques.
    pcStructAnch = struct('pc', pcAnch, 'matchIndx', mtchAnchStct.indxPC);
    pcStructMoved = struct('pc', pcMoved, 'matchIndx', mtchMovedStct.indxPC);
    [tformMoved2Anchor, rmse, regStats] = FindTransformationPC2toPC1(pcStructAnch, ...
        pcStructMoved, regrigidStruct);
    regRigidError(iNum, 1) = rmse;
    
    % Log, Save and Display
    % =====================
    LogRegistrationStatus(regStats, pcNameAnch, pcNameMoved, ...
        matchPtsCount(iNum, 1), logFileName);
    % Save the transformation matrix into a file
    rtFromTo = [num2str(movedNum), '_to_', num2str(anchNum)];
    rtNameMoved = ['rt_', rtFromTo, '.txt'];
    rtFullNameMoved = [dirName, '/', rtFolderName, '/', rtNameMoved];
    WriteRT(tformMoved2Anchor, rtFullNameMoved);
    rtInfo(iNum+1, :) = {movedIndx, tformMoved2Anchor.R, tformMoved2Anchor.T', ...
        rtFromTo};

    % If needed display the point cloud
    if dispFlag.pcPair == 1
        DisplayPCs(pcAnch, pcMoved, pcNameAnch, pcNameMoved, tformMoved2Anchor);
    end
end

% Store the total number of files processed status
logString = string(['\nPair of files processed in total: ', num2str(iNum), '\n\n']);
LogInfo(logFileName, logString);

% Outputs
% =======
% Create a table out of "matched point count" and "rmse" along with names and
% the matching pixels of anchor and moved pc
matchInfo = table(imgName(:,1), imgName(:,2), matchPtsCount, regRigidError, ...
     matchPtsPxls(:,1), matchPtsPxls(:,2), 'VariableNames', {'Anchor', ...
     'Moved', 'Matched_Points', 'ICP_RMSE', 'PtsPxls_Anch', 'PtsPxls_Moved'});

% Create a table for R|T
rtInfo = table(cell2mat(rtInfo(:,1)), rtInfo(:,2), rtInfo(:,3), rtInfo(:,4), ...
    'VariableNames', {'ViewId', 'Orientation', 'Location', 'Moved_To_Anchor'});
end

%%
function points2D = ProjectPCs2RGBImage(pc, tformDepth2RGB)
% Project the 3D points onto their corresponding RGB images using the intrinsic 
% and extrinsic matrix of the Kinect IR and RGB camera. We are just assuming
% that the given point cloud is already in the RGB frame.

% Get the UV values of those projected points on the RGB images
rgbUVs = ProjectPointsOnImage(pc.Location, tformDepth2RGB.KK_RGB);
rgbUVs = round(rgbUVs);

% Create the point cloud object so that it could be used in feature extraction
% process.
points2D = cornerPoints(rgbUVs);
end

%%
function tformDepth2RGB = TformMatFromCalibration(calibStereo)
% Load the stereo-calibration parameters. If you have Depth-to-RGB then use it
% directly or else take the inverse of RGB-% to-Depth parameters.
load(calibStereo, 'R', 'T', 'KK_left', 'KK_right');
% Create as structure that will hold R matrix & T vector and
% the intrinsic parameters of each camera.
tformDepth2RGB.R = inv(R);
% Convert into Centimeters as the PC is into Centimeters
tformDepth2RGB.T = -inv(R)*T/10;
tformDepth2RGB.KK_RGB = KK_left;
tformDepth2RGB.KK_IR = KK_right;
end

%%
function [tformPC2toPC1, rmse, regStatus] = FindTransformationPC2toPC1(pcStruct1, ...
    pcStruct2, paramsRegRigid)
% Here, we are going to find transformation from the matched 3D point pairs. For
% which, we need at least 3 points. However, to be on safe side, I have used the
% minimum matched point pair count as 5.
numMatchPts = size(pcStruct1.matchIndx, 1);
if numMatchPts < 5
    R = eye(3);
    T = ones(3,1);
    tformPC2toPC1 = struct('R', R, 'T', T);
    rmse = inf;             % No registration, so error is infinite
    regStatus = false;
    return
end

% Coarse Registration
% ===================
% Otherwise, first find the coarse transformation and then update the same
% carrying out the fine registration.
pcMatch1 = pcStruct1.pc.Location(pcStruct1.matchIndx, :);
pcMatch2 = pcStruct2.pc.Location(pcStruct2.matchIndx, :);
[R, T] = EstimateRT(pcMatch2', pcMatch1');                      % Coarse reg
regStatus = true;
tformPC2toPC1 = struct('R', R, 'T', T);

% Fine Registration
% =================
% Run the ICP or similar algorithms to do a final registration and then update 
% the current transformation matrix. Update the initial transformation field
% with the current findings and run the rigid ICP.
paramsRegRigid.initTform = tformPC2toPC1;
[tformPC2toPC1, ~, rmse] = RunRigidReg(pcStruct1.pc, pcStruct2.pc, paramsRegRigid);
end

%%
function LogRegistrationStatus(regStats, pcNameAnch, pcNameMoved, ...
    matchPtsCount, logFileName)
% Function to save the registration status into a file and also display the
% same.
if ~regStats
    statusStr = string(['FAILED to register ', pcNameAnch, ' and ', ...
        pcNameMoved, '. Number of matching points -- "', ...
        num2str(matchPtsCount), '"\n']);
else
    statusStr = string(['Succeded to register ', pcNameAnch, ' and ', ...
        pcNameMoved, '. Number of matching points -- "', ...
        num2str(matchPtsCount), '"\n']);
end
disp(statusStr);
LogInfo(logFileName, statusStr);
end

%%
function DisplayPCs(pcAnch, pcMoved, pcNameAnch, pcNameMoved, tformMoved2Anchor)
% Function to display the anchor and the transformed version of moved point
% cloud.
pcMoved_Tformed = TransformPointCloud(pcMoved, tformMoved2Anchor);
figure();
pcshowpair(pcAnch, pcMoved_Tformed);
legend({pcNameAnch, sprintf('Transformed %s', pcNameMoved)}, 'Interpreter', ...
    'none');
end

%% Input arguments validating functions
function TF = validateDirStruct(dirStruct)
TF = false;
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

function TF = validateImgNumStruct(imgNumStruct)
TF = false;
% First validate whether the structure contains the required fields or not.
if ~all(isfield(imgNumStruct, {'startIndx', 'endIndx'}))
    error('The start and end index of the file number should be given');
elseif ~isnumeric(imgNumStruct.startIndx) || ~isnumeric(imgNumStruct.endIndx)
    % Then check whether the given fields are consistent with the data type that 
    % is required.
    error('The start and end index should be numeric values')
else
    TF = true;
end
end

function TF = validateGeomParams(geomParamsStruct)
TF = false;
% First validate whether the structure contains the required fields or not.
if ~all(isfield(geomParamsStruct, {'tformType', 'maxDist'}))
    error('Provide the fields -- tformType, maxDist');
elseif ~ischar(geomParamsStruct.tformType) || ~isnumeric(geomParamsStruct.maxDist)
    % Then check whether the given fields are consistent with the data type that
    % is required.
    error('Provide proper data types for the structure');
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

function TF = validateDispFlag(dispFlag)
TF = false;
if ~all(isfield(dispFlag, {'pcPair', 'matchPair'}))
    % First validate whether the structure contains the required fields or not.
    error('Provide the fields -- pcPair, matchPair');
elseif ~islogical(dispFlag.pcPair) || ~islogical(dispFlag.matchPair)
    % Then check whether the given fields are consistent with the data type that
    % is required.
    error('The data types for the fields should be -- true/false');
else
    TF = true;
end
end

function TF = validateRegRigidParams(regRigidParams)
if ~all(isfield(regRigidParams, {'maxIter', 'icpMethod'}))
    % First validate whether the structure contains the required fields or not.
    error('Provide the fields -- pcPair, matchPair');
elseif ~isnumeric(regRigidParams.maxIter) || (regRigidParams.maxIter < 0)
    % Maximum iteration count check
    error('The maximum iteration count should be a non-negative integer');
elseif ~any(validatestring(regRigidParams.icpMethod , ...
        {'pointToPoint', 'pointToPlane', 'ndt3D'}))
    % Registration method name check
    error('The registration method should be --- pointToPoint|pointToPlane|ndt3D');
else
    TF = true;
end
end
