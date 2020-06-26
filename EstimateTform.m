function [matchPairWise, rtPairWise, matIncidenceWeight, pcPairWise] = ...
    EstimateTform(dirStruct, imgNumStruct, varargin)
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
% As far as the pairs are concerned, I will take all possible pairs. For
% example, given "n" images, we are going to get nC2 number of pairs in total.
% And for each pair, matching will be evaluated.
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
%   1) MaxDistance -- Maximum distance from point to projection
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
%	1) MaxIterations -- Maximum number of iteration
%   2) Metric -- This could be pointToPlane | pointToPoint
%
% 8. ['saveRTFlag', saveRTFlag]: Logical value used to decide whether to store
% the R|T values into corresponding text files or not.
%
% 9. ['nearbyViewsTh', nearbyViewsTh]: Interger value -- This number will define
% how many near by images are checked with the current image in clockwise and
% anti-clockwise direction. Default is 4 and max is 10.
%
% OUTPUT(s)
% =========
% 1. matchPairWise: Mx8 table
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
% 2. rtPairWise: Mx3 table
%   1) ViewID -- View number (1 through number of images)
%   2) Orientation -- Rotation matrix w.r.t anchor/pc
%   3) Translation -- Translation vector w.r.t. anchor image/pc
%
% 3. matIncidenceWeight: MxM table, it is an incidence matrix which will store
% weight for the successful matched pairs.
%
% Example(s)
% ==========
%

%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% Validate input arguments -----------------------------------------------------
p = inputParser;
p.StructExpand = false;             % Accept structure as one element

% Compulsory parameters --
addRequired(p, 'dirStruct', @validateDirStruct);
addRequired(p, 'imgNumStruct', @validateImgNumStruct);

% Optional parameters --
% Parameters to be used to match 2 RGB images
defaultGeomParams = struct('MaxDistance', 3.5);
% Calibration parameters of the Kinect sensors
defaultCalibStereo = ['~/Dropbox/PhD/Data/Calibration/Calibration_20181114/', ...
    'HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
% Display pair of point clouds.
defaultFlags = struct('matchPair', false, 'pcPair', false);
% Accepted corner detection techniques --
validCornerTechs = {'SURF', 'Harris', 'UserDefined'};
% Regid registration parameters -- More parameter options are available on
% pcregistericp()/pcregrigid() help docs.
defaultRegRigidParams =  struct('MaxIterations', 20, 'Metric', 'pointToPlane');

addParameter(p, 'geomParamsStruct', defaultGeomParams, @(x)isstruct(x));
addParameter(p, 'calibStereo', defaultCalibStereo, @validateCalibStereo);
addParameter(p, 'dispFlag', defaultFlags, @validateDispFlag);
addParameter(p, 'cornerTech', 'SURF', @(x) any(validatestring(x, validCornerTechs)));
addParameter(p, 'regrigidStruct', defaultRegRigidParams, @(x) isstruct(x));
addParameter(p, 'saveRTFlag', false, @(x) islogical(x));
addParameter(p, 'nearbyViewsTh', 4, @(x) isinteger(x) && x < 10)

p.parse(dirStruct, imgNumStruct, varargin{:});
disp('Given inputs for EstimateTform_Batch() function:');
disp(p.Results);
fprintf('\n');

% Store variables into local variables to save typing :-p
dirName = dirStruct.dirName;
rtFolderName = dirStruct.rtFolderName;
plyFolderName = dirStruct.plyFolderName;
geomParamsStruct = p.Results.geomParamsStruct;
calibStereo = p.Results.calibStereo;
dispFlag = p.Results.dispFlag;
cornerTech = p.Results.cornerTech;
regrigidStruct = p.Results.regrigidStruct;
saveRTFlag = p.Results.saveRTFlag;
nearbyViewsTh = p.Results.nearbyViewsTh;

% If the sub-folder "Relative" doesn't exist then create to store all the
% pairwise relative transformation matrices.
rtSubFolderName = [dirName, '/', rtFolderName, '/Relative'];
if ~(exist(rtSubFolderName, 'dir') == 7)
    % Create the folder to save the files
    mkdir(rtSubFolderName);
end

% Algorithm --------------------------------------------------------------------
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
numImgs = length(fileList);             % Total number of images
imgPairList = nchoosek(fileNumbers, 2); % All possible image pair combinations
viewIDSeq = 1:numImgs;                  % View numbers 1 through N
viewIDSeqPairs = nchoosek(viewIDSeq,2); % All possible pairs with seqentail numbers
numPairs = size(imgPairList, 1);        % Total number of combinations
matchPtsCount = zeros(numPairs, 1);     % Store matching points
regRigidError = zeros(numPairs, 1);     % Hold the error from ICP algorithm
imgName = cell(numPairs, 2);            % Name of matching image pair
matchPtsPxls = cell(numPairs, 2);       % Holds pair of structures
pcPairInfo = cell(numPairs, 4);         % Anchor and Moved pc and matched indices
% For Bundle adjustment, we also need the view-ids which is nothing but a
% sequential view number.
viewIDPairs = zeros(numPairs, 4);       % Pair of view IDs -- Image nums & seq nums
regPairStatus = false(numPairs,1);      % Keep track of FAILED/SUCCEEDED pairs
% Store Anch-num, Moved-num, R, T, and "from-to" name. For the 1st images we
% always assume the the rotation is identity matrix and the translation is zero-
% vector.
rtPairWise = cell(numPairs, 5);
% Incidence matrix of a graph to hold all the successful pair-wise connections.
% The row and column name of the table will be the image names. And, as this is
% an undirected graph, it will an symmetric matrix. Also, keep in mind that the
% 'VariableNames' should strar with a 'character'.
rcNames = string(num2cell(fileNumbers)).cellstr; % Needed a cell of strings
cNames = "To_" + rcNames;
cNames = cNames.cellstr;
matIncidenceWeight = array2table(zeros(numImgs, numImgs), 'VariableNames', cNames, ...
    'RowNames', rcNames);
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
logString = string(['Processing started at: ', datestr(datetime), '\n']);
LogInfo(logFileName, logString);

% Load a pair of images at a time and find the correspondence. The first image
% will be the anchor image and the 2nd will be the moved point cloud
for iIP = 1:numPairs
    % Check whether the movedNum is with in "nearbyViewsTh" images away from the
    % anchNum in clockwise and anit-clockwise direction. If the movedNum is
    % beyond the given threshold don't try to match the pairs.
    anchNumSeq = viewIDSeqPairs(iIP, 1);
    movedNumSeq = viewIDSeqPairs(iIP, 2);
    numHops = min(movedNumSeq-anchNumSeq, anchNumSeq+numImgs-movedNumSeq);
    if numHops > nearbyViewsTh
        continue;
    end
    
	anchNum = imgPairList(iIP, 1);
    movedNum = imgPairList(iIP, 2);
    % Save the view-id -- The view ids will the actual number cropped from image
    % name. %%%TODO: If the bundle adjust need sequential numbering scheme then
    % I need to update it.%%%
    viewIDPairs(iIP, :) = [anchNumSeq, movedNumSeq, anchNum, movedNum];
    
    % Read the 1st RGB image & PC
    % ===========================
    % Here, we are also going to read corresponding rt*.txt file if exists. If
    % not we are going to crate one.
    anchIndx = fileNumbers == anchNum;
    rgbFullNameAnch = fileList{anchIndx};
    rgbImgAnch = imread(rgbFullNameAnch);   % Read 1st image
    % Read teh 2st point cloud corresponding to the RGB image
    pcNameAnch = ['depthImg_', num2str(anchNum), '.ply'];
    pcFullNameAnch = [dirName, '/', plyFolderName, '/', pcNameAnch];
    pcAnch = pcread(pcFullNameAnch);        % 1st point cloud
    
    % Read the 2nd RGB image & PC
    % ===========================
    movedIndx = fileNumbers == movedNum;
    % Read the 2nd point cloud
    rgbFullNameMoved = fileList{movedIndx};
    rgbImgMoved = imread(rgbFullNameMoved); % Read 2nd image
    pcNameMoved = ['depthImg_', num2str(movedNum), '.ply'];
    pcFullNameMoved = [dirName, '/', plyFolderName, '/', pcNameMoved];
    pcMoved = pcread(pcFullNameMoved);      % 2nd point cloud
    
    % Display the image names that were supposed to be matched
    rgbNameAnch = ['rgbImg_', num2str(anchNum), '.jpg'];
    rgbNameMoved = ['rgbImg_', num2str(movedNum), '.jpg'];
    imgName{iIP, 1} = num2str(anchNum);    % Store the names
    imgName{iIP, 2} = num2str(movedNum);
    fprintf('Matching -- %s and %s\n', rgbNameAnch, rgbNameMoved);
    fprintf('========\n');
    
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
        rgbImgMoved, 'matchingParams', matchingStruct, 'geomEstParams', ...
        geomParamsStruct, 'dispFlag', dispFlag.matchPair);
    
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
    matchPtsCount(iIP, 1) = size(mtchAnchStct.indxPC, 1);
    matchPtsPxls(iIP, :) = {mtchAnchStct, mtchMovedStct};
    pcPairInfo(iIP, :) = {pcAnch, pcMoved, mtchAnchStct.indxPC, ...
        mtchMovedStct.indxPC};
    
    % Find the transformation
    % =======================
    % Using the matching 3D point pairs find the coarse transformation between
    % the two point clouds and if needed do a fine registration using standard
    % registration techniques.
    pcStructAnch = struct('pc', pcAnch, 'matchIndx', mtchAnchStct.indxPC);
    pcStructMoved = struct('pc', pcMoved, 'matchIndx', mtchMovedStct.indxPC);
    [tformMoved2Anchor, rmse, regStats] = FindTransformationPC2toPC1(pcStructAnch, ...
        pcStructMoved, regrigidStruct);
    regRigidError(iIP, 1) = rmse;
    if regStats && (rmse < 2.5)
        regPairStatus(iIP, 1) = true;
        % Incidence matrix of a directed graph -- As the transformation is from
        % moved-to-anchor, but not the other way around.
        matIncidenceWeight(num2str(movedNum), ['To_', num2str(anchNum)]) = ...
            {1/matchPtsCount(iIP, 1)};
%         matIncidenceWeight(num2str(anchNum), ['To_', num2str(movedNum)]) = {numHops^2};
    else
        regPairStatus(iIP, 1) = false;
    end
    
    % Log, Save and Display
    % =====================
    LogRegistrationStatus(regStats, pcNameAnch, pcNameMoved, ...
        matchPtsCount(iIP, 1), logFileName);
    % Save the transformation matrix into a file if needed
    rtFromTo = [num2str(movedNum), '_to_', num2str(anchNum)];
    if saveRTFlag == true
        rtNameMoved = ['rt_', rtFromTo, '.txt'];
        rtFullNameMoved = [rtSubFolderName, '/', rtNameMoved];
        WriteRT(tformMoved2Anchor, rtFullNameMoved);
    end

    % Also save the R|T values in a table -- All the CV toolbox functions use
    % the "Transpose" of the matrices and vectors, such that:
    %       [x y z] = [X Y Z]*R' + t'
    % where, R is a 3x3 matrix and t is a 3x1 vector.
    rtPairWise(iIP, :) = {anchNum, movedNum, tformMoved2Anchor.R', ...
        tformMoved2Anchor.T', rtFromTo};
    % If needed display the point cloud
    if dispFlag.pcPair && regStats
        DisplayPCs(pcAnch, pcMoved, pcNameAnch, pcNameMoved, tformMoved2Anchor);
    end
end

% Log the outcome
% ===============
% Store the total number of files processed status
numSucceededPairs = nnz(regPairStatus);         % Count number of successful pairs
logString = string(['\nOut of ', num2str(iIP), ' pairs of files, ', ...
    num2str(numSucceededPairs), ' succeeded and ', ...
    num2str(numPairs - numSucceededPairs), ' failed.', '\n\n']);
LogInfo(logFileName, logString);

% Prune the failed attempts
% =========================
% We are going to get rid of all the pairs that have failed to match with each
% other as there are fewer corresponding points that the threshold count which
% is 5 in my case.
imgName = imgName(regPairStatus, :);
matchPtsCount = matchPtsCount(regPairStatus, :);
regRigidError = regRigidError(regPairStatus, :);
matchPtsPxls = matchPtsPxls(regPairStatus, :);
pcPairInfo = pcPairInfo(regPairStatus, :);
viewIDPairs = viewIDPairs(regPairStatus, :);
rtPairWise = rtPairWise(regPairStatus, :);

% Outputs
% =======
% Create a table out of "matched point count" and "rmse" along with names and
% the matching pixels of anchor and moved pc
matchPairWise = table(imgName(:,1), imgName(:,2), matchPtsCount, regRigidError, ...
     matchPtsPxls(:,1), matchPtsPxls(:,2), viewIDPairs(:,1), viewIDPairs(:,2), ...
     viewIDPairs(:,3), viewIDPairs(:,4), 'VariableNames', {'Anchor', 'Moved', ...
     'Matched_Points', 'ICP_RMSE', 'PtsPxls_Anch', 'PtsPxls_Moved', ...
     'Anchor_ViewID_Sq', 'Moved_ViewID_Sq', 'Anchor_ViewID', 'Moved_ViewID'});

% Create a table for R|T
rtPairWise = table(cell2mat(rtPairWise(:,1)), cell2mat(rtPairWise(:,2)), rtPairWise(:,3), ...
    rtPairWise(:,4), rtPairWise(:,5), 'VariableNames', {'Anchor_Num', 'Moved_Num', ...
    'Orientation', 'Location', 'Moved_To_Anchor'});

% Create a table for "Anchor" and "Moved" point clouds.
pcPairWise = table(imgName(:,1), imgName(:,2), pcPairInfo(:,1), pcPairInfo(:,2), ...
    pcPairInfo(:,3), pcPairInfo(:,4), matchPtsCount, 'VariableNames', ...
    {'Anchor', 'Moved', 'Anch_PC', 'Moved_PC', 'Anch_PtsIndx', 'Moved_PtsIndx', ...
    'Matched_Points'});
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
if numMatchPts < 10
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

% Build a affine3D object from R and T provided as the initial rotation and
% translation.
paramsRegRigid.InitialTransform = ConvertRTtoAffine3D(tformPC2toPC1.R, tformPC2toPC1.T);
% Register the point clouds
[tformPC2toPC1, ~, rmse] = pcregrigid(pcStruct2.pc, pcStruct1.pc, paramsRegRigid);
% Convert the affine3D object into R matrix and T vector and return it as a
% structure.
[finalR, finalT] = ConvertAffine3DtoRT(tformPC2toPC1);
tformPC2toPC1 = struct('R', finalR, 'T', finalT);
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
fprintf(sprintf('%s%s', statusStr, '\n'));
LogInfo(logFileName, statusStr);
end

%%
function DisplayPCs(pcAnch, pcMoved, pcNameAnch, pcNameMoved, tformMoved2Anchor)
% Function to display the anchor and the transformed version of moved point
% cloud.
pcMoved_Tformed = TransformPointCloud(pcMoved, tformMoved2Anchor);
figure(10);
pcshowpair(pcAnch, pcMoved_Tformed);
legend({pcNameAnch, sprintf('Transformed %s', pcNameMoved)}, 'Interpreter', ...
    'none');
end

%% Input arguments validating functions
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

function TF = validateImgNumStruct(imgNumStruct)
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

function TF = validateCalibStereo(calibStereo)
% Is it a existing matfile or no
if exist(calibStereo, 'file') == 2
    TF = true;
else
    error('It should be a valid mat-file');
end
end

function TF = validateDispFlag(dispFlag)
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
