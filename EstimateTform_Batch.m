function matchPtsCount = EstimateTform_Batch(dirStruct, imgNumStruct, varargin)
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
% INPUT(s):
% 1 [dirStruct]
%   Directory structure containing the RGB/Depth images and the corresponding
%   'ply' files and the 'rt' files.
%       1) dirName      -- Name of the folder containing the depth and RGB images
%       2) plyFolderName-- Name of the folder relative to 'dirName' containing
%   the ply files.
%       3) rtFolderName -- Name of the folder relative to 'dirName' holding all
%	the text files which contain the R|T information.
%
% 2 [imgNumStruct]
%   Structure holding the stand and end number of a sequence of images which
%   need to be processed
%       1) startIndx    -- File number of 1st pc
%       2) endIndx      -- Last file number
%
% 3 ['geomParamsStruct', geomParamsStruct]
%   Parameters for matching the two images and removing outlier in pair of
%   matching points. (For more information, look at the
%   'estimateGeometricTransform()' help document.
%       1) tformType    -- It could be similarity|affine|projective
%       2) maxDist      -- Maximum distance from point to projection
%
% 4 ['calibStereo', calibStereo]
%   Mat-file holding stereo calibration parameters between IR and RGB of the
%   Kinect that was used to collect the data.
%
% 5 ['dispFlag', dispFlag]
%   Flag to display or not the matched pixels in the image pair and the final
%   transformed point cloud pair
%       1) pcPair       -- [0]/1 to display the final registered point clouds
%       2) matchPair    -- [0]/1 to display matched pair of pixels
%
% 6 ['cornerTech', cornerTech]
%   Either automatic (SURF|Harris) or manual (UserDefined)
%
% OUTPUT(s):
% 1 [matchPtsCount]
%   Count of number of matched points of each image with the previous one. The
%   count for the 1st image will be 0.
%
% Example:
%   dirStruct = struct('dirName', '~/Dropbox/PhD/Data/Data/Alvaro/2017_0825/103/
%       SampleImages/', 'plyFolderName', 'PCinPLY_woPlane_Testing', 
%       'rtFolderName', 'PCinXYZNorTri_woPlane_Testing');
%   imgNumStruct = struct('startIndx', 1250, 'endIndx',1260);
%   geomParamsStruct = struct('tformType', 'projective', 'maxDist', 3.5);
%   dispFlag = struct('pcPair', false, 'matchPair', false)
%   mtchPts = EstimateTform_Batch(dirStruct, imgNumStruct, 'geomParamsStruct', ...
%       geomParamsStruct,'dispFlag', dispFlag, 'cornerTech', 'userdefined');

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------

% Validate input arguments ----------------------------------------------------
p = inputParser;
p.StructExpand = false;             % Accept strutcture as one element

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

addRequired(p, 'dirStruct', @validateDirStruct);
addRequired(p, 'imgNumStruct', @validateImgNumStruct);
addParameter(p, 'geomParamsStruct', defaultGeomParams, @validateGeomParams);
addParameter(p, 'calibStereo', defaultCalibStereo, @validateCalibStereo);
addParameter(p, 'dispFlag', defaultFlags, @validateDispFlag);
addParameter(p, 'cornerTech', defaultCorner, validateCorner);

p.parse(dirStruct, imgNumStruct, varargin{:});
disp(p.Results);

% Store variales into local variables to save typing :-p
dirName = dirStruct.dirName;
rtFolderName = dirStruct.rtFolderName;
plyFolderName = dirStruct.plyFolderName;
geomParamsStruct = p.Results.geomParamsStruct;
calibStereo = p.Results.calibStereo;
dispFlag = p.Results.dispFlag;
cornerTech = p.Results.cornerTech;

% Algorithm -------------------------------------------------------------------
% Create a list of files
fileList = [];                  % Store a list of image files with complete path
fileNumbers = [];               % Store the numbers given to the images
imgIndx = 1;                    % Counter
while imgNumStruct.startIndx < imgNumStruct.endIndx
    iNum = imgNumStruct.startIndx;
    rgbName = ['rgbImg_', num2str(iNum), '.jpg'];
    rgbFullName = [dirStruct.dirName, '/', rgbName];
    if exist(rgbFullName, 'file') == 2
        fileList{imgIndx} = rgbFullName;
        fileNumbers(imgIndx) = iNum;
        imgIndx = imgIndx + 1;
    end
    imgNumStruct.startIndx = imgNumStruct.startIndx + 1;   % Check the next one
end

numImgs = length(fileList);         % Total number of images
matchPtsCount = zeros(numImgs, 1);  % To store mating points
% If there is only 1 image then then there is no point in finding the matches.
if numImgs < 2
    disp('Only few images found, so operation aborted');
    return
end

% Before going through each pair of images and finding out the correspondences,
% read the calibration parameters and create a structure to holding 
% transformation matrix, etc...
tformDepth2RGB = TformMatFromCalibration(calibStereo);
    
% Load a pair of images at a time and find the correspondence. The first image
% will be the anchor image and the 2nd will be the moved point cloud
for iNum = 1:numImgs-1
    % Read the 1st RGB image and the corresponding rt*.txt file
    anchNum = fileNumbers(iNum);
    rgbFullNameAnch = fileList{iNum};
    rgbImgAnch = imread(rgbFullNameAnch);   % Read 1st image
    % Read teh 2st point cloud corresponding to the RGB image
    pcNameAnch = ['depthImg_', num2str(anchNum), '.ply'];
    pcFullNameAnch = [dirName, '/', plyFolderName, '/', pcNameAnch];
    pcAnch = pcread(pcFullNameAnch);        % 1st point cloud
    
    % Check for the 1st R|T text file. If it doesn't exist the create one with 
    % default values.
    rtNameAnch = ['rt_', num2str(anchNum), '.txt'];
    rtFullNameAnch = [dirName, '/', rtFolderName, '/', rtNameAnch];
    if ~(exist(rtFullNameAnch, 'file') == 2)
        WriteRT(struct('R', eye(3,3), 'T', zeros(3,1)), rtFullNameAnch);
    end

    % Read the 2nd RGB image and its rt*.txt file
    movedIndx = iNum + 1;
    movedNum = fileNumbers(movedIndx);
    % Read the 2nd point cloud
    rgbNameFullMoved = fileList{iNum+1};
    rgbImgMoved = imread(rgbNameFullMoved); % Read 2nd image
    pcNameMoved = ['depthImg_', num2str(movedNum), '.ply'];
    pcFullNameMoved = [dirName, '/', plyFolderName, '/', pcNameMoved];
    pcMoved = pcread(pcFullNameMoved);      % 2nd point cloud
    
    % Display the image names that were supposed to be matched
    rgbNameAnch = ['rgbImg_', num2str(anchNum), '.jpg'];
    rgbNameMoved = ['rgbImg_', num2str(movedNum), '.jpg'];
    fprintf('Matching -- %s and %s\n\n', rgbNameAnch, rgbNameMoved);
    
    % Prepare data for matching two RGB images -- Detect corners either 
    % automatically (SURF | Harris) or provide them by projecting the point 
    % clouds onto the RGB images.
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
    [inlierPtsAnch, inlierPtsMoved] = FindMatchedPoints(rgbImgAnch, ...
        rgbImgMoved, matchingStruct, geomParamsStruct, dispFlag.matchPair);
    
    % Estimate the initial guess for transformation between the two point cloud
    % using the RGB matching points.
    pcStructAnch = struct('rgbPts', inlierPtsAnch, 'pc', pcAnch, ...
        'tformDepth2RGB', tformDepth2RGB);
    pcStructMoved = struct('rgbPts', inlierPtsMoved, 'pc', pcMoved, ...
        'tformDepth2RGB', tformDepth2RGB);
    [tformMoved2Anchor, matchPtsCount(movedIndx), regStats] = ...
        EstimateTformMatchingRGB(pcStructAnch, pcStructMoved);
    
    % Run the ICP or similar algorithms to do a final registration and then
    % update the current transformation matrix.
    regStruct = struct('initTform', tformMoved2Anchor, 'maxIter', 20, ...
        'icpMethod', 'pointToPlane');
    tformMoved2Anchor = RunRigidReg(pcAnch, pcMoved, regStruct);
    
    % Check the registration status, i.e., whether it succeeded or failed and
    % store the information into a file.
    logFileName = [dirName, '/', rtFolderName, '/Log-', date, '.txt'];
    LogRegistrationStatus(regStats, pcNameAnch, pcNameMoved, ...
        matchPtsCount(movedIndx), logFileName);
    
    % Save the transformation matrix into a file
    rtNameMoved = ['rt_', num2str(movedNum), '.txt'];
    rtFullNameMoved = [dirName, '/', rtFolderName, '/', rtNameMoved];
    WriteRT(tformMoved2Anchor, rtFullNameMoved);
    
    % If needed display the point cloud
    if dispFlag.pcPair == 1
        DisplayPCs(pcAnch, pcMoved, pcNameAnch, pcNameMoved,...
            tformMoved2Anchor);
    end
end
end

%%
function points2D = ProjectPCs2RGBImage(pc, tformDepth2RGB)
% Project the 3D points onto their corresponding RGB images using the intrinsic 
% and extrinsic matrix of the Kinect IR and RGB camera
pcInRGBFrame = TransformPointCloud(pc, tformDepth2RGB);

% Get the UV values of those projected points on the RGB images
rgbUVs = ProjectPointsOnImage(pcInRGBFrame.Location, tformDepth2RGB.KK_RGB);
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
% Convert into Meters as the PC is in Meters
tformDepth2RGB.T = -inv(R)*T/1000;
tformDepth2RGB.KK_RGB = KK_left;
tformDepth2RGB.KK_IR = KK_right;
end

%%
function LogRegistrationStatus(regStats, pcNameAnch, pcNameMoved, ...
    matchPtsCount, logFileName)
% Function to save the registration status into a file and also display the
% same.
if regStats ~= 0
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
% First validate whether the structure contains the required fields or not.
if ~all(isfield(dispFlag, {'pcPair', 'matchPair'}))
    error('Provide the fields -- pcPair, matchPair');
elseif ~islogical(dispFlag.pcPair) || ~islogical(dispFlag.matchPair)
    % Then check whether the given fields are consistent with the data type that
    % is required.
    error('The datatypes for the fields should be -- true/false');
else
    TF = true;
end
end
