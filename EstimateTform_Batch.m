function EstimateTform_Batch(dirStruct, imgNumStruct, geomParamsStruct, ...
    calibStereo, dispFlag, cornerTech)
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
%   dirStruct       := Directory structure containing the RGB/Depth images and
%           the corresponding 'ply' files and the 'rt' files.
%       1) dirName      -- Name of the folder containing the depth and RGB images
%       2) plyFolderName-- Name of the folder relative to 'dirName' containing
%           the ply files.
%       3) rtFolderName -- Name of the folder relative to 'dirName' holding all
%           the text files which contain the R|T information.
%   imgNumStruct    := Structure holding the stand and end number of a sequence
%           of images which need to be processed
%       1) startIndx    -- File number of 1st pc
%       2) endIndx      -- Last file number
%   geomParamsStruct:= Parameters for matching the two images and removing
%           outlier in pair of matching points. (For more information, look at
%           the 'estimateGeometricTransform()' help document.
%       1) tformType    -- It could be similarity|affine|projective
%       2) maxDist      -- Maximum distance from point to projection
%   calibStereo     := Mat-file holding stereo calibration parameters between 
%           IR and RGB of the Kinect that was used to collect the data.
%   dispFlag        := Flag to display or not the matched pixels in the image
%           pair and the final transformed point cloud pair
%       1) pcPair       -- [0]/1 to display the final registered point clouds
%       2) matchPair    -- [0]/1 to display matched pair of pixels
%   cornerTech      := Either automatic (SURF|Harris) or manual (UserDefined) 
%
% OUTPUT(s):
%
% Example:
%   dirStruct = struct('dirName', '/home/fovea/tnb88/Dropbox/PhD/Data/Data/Alvaro/
%       2017_0825/103/SampleImages/', 'plyFolderName', 'PCinPLY_woPlane_Testing', 
%       'rtFolderName', 'PCinXYZNorTri_woPlane_Testing'),
%   imgNumStruct = struct('startIndx', 1250, 'endIndx',1260);
%   geomParamsStruct = struct('tformType', 'projective', 'maxDist', 3.5);
%   dispFlag = struct('pcPair', 0, 'matchPair', 0)
%   EstimateTform_Batch(dirStruct, imgNumStruct, geomParamsStruct, [], [], 
%       'UserDefined')
%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------
% Directory containing the images
if nargin < 2
    error('Provide the directory name containing the images and number sequence');
end
% Parameters to be used to match 2 RGB images
if nargin < 3 || isempty(geomParamsStruct)
    geomParamsStruct = struct('tformType', 'projective', 'maxDist', 3.5);
end
% Calibration parameters of the Kinect sensors
if nargin < 4 || isempty(calibStereo)
    calibStereo = ['~/Dropbox/PhD/Data/Calibration/', ...
        'Calibration_20181114/HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
    disp('WARNING!!! -- Loading the handheld kinect parameters');
end
% Display pair of point clouds.
if nargin < 5 || isempty(dispFlag)
    dispFlag.matchPair = 0;     % Don't display the mated pixels in the image pair
    dispFlag.pcPair = 0;        % Don't display the final registered pair
end
% Corner detection technique --
if nargin < 6 || isempty(cornerTech)
    cornerTech = 'SURF';
end

% Load a pair of images at a time and find the correspondence.
while imgNumStruct.startIndx < imgNumStruct.endIndx
    % In the begining, make the 1st found image as the anchor image. After that,
    % however, make the 'moved' image as the anchor image.
    anchNum = imgNumStruct.startIndx;
    rgbNameAnch = ['rgbImg_', num2str(anchNum), '.jpg'];
    rgbFullNameAnch = [dirStruct.dirName, '/', rgbNameAnch];
    
    % IF the file is available read it and set it as the Anchor image and then
    % look for the Moved image.
    if exist(rgbFullNameAnch, 'file') == 2
        rgbImgAnch = imread(rgbFullNameAnch);               % Read 1st image
        
        % Check for the corresponding R|T text file. If it doesn't exist the
        % create one with default values.
        rtNameAnch = ['rt_', num2str(anchNum), '.txt'];
        rtFullNameAnch = [dirStruct.dirName, '/', dirStruct.rtFolderName, ...
            '/', rtNameAnch];
        if ~(exist(rtFullNameAnch, 'file') == 2)
            WriteRT(struct('R', eye(3,3), 'T', zeros(3,1)), rtFullNameAnch);
        end
        
        % Now, look for the next RGB image
        imgNumStruct.startIndx = imgNumStruct.startIndx + 1;
        while imgNumStruct.startIndx < imgNumStruct.endIndx
            movedNum = imgNumStruct.startIndx;
            % Read anchored and moved point cloud
            rgbNameMoved = ['rgbImg_', num2str(movedNum), '.jpg'];
            rgbNameFullMoved = [dirStruct.dirName, '/', rgbNameMoved];
            
            % IF the file is found then set it as the moved point cloud and then
            % find the correspondence between the two images
            if exist(rgbNameFullMoved, 'file') == 2
                % Display the image names that were supposed to be matched
                fprintf('Matching -- %s and %s\n\n', rgbNameAnch, rgbNameMoved);
                
                % Read the second RGB image
                rgbImgMoved = imread(rgbNameFullMoved);     % Read 2nd image
                
                % Now, read the corresponding 3D point clouds
                pcNameAnch = ['depthImg_', num2str(anchNum), '.ply'];
                pcFullNameAnch = [dirStruct.dirName, '/', ...
                    dirStruct.plyFolderName, '/', pcNameAnch];
                pcAnch = pcread(pcFullNameAnch);
                pcNameMoved = ['depthImg_', num2str(movedNum), '.ply'];
                pcFullNameMoved = [dirStruct.dirName, '/', ...
                    dirStruct.plyFolderName, '/', pcNameMoved];
                pcMoved = pcread(pcFullNameMoved);
                
                % Also, read the calibration parameters and create a structure 
                % to holding transformation matrix, etc...
                tformDepth2RGB = TformMatFromCalibration(calibStereo);

                % Prepare data for matching two RGB images -- Detect corners
                % either automatically (SURF | Harris) or provide them by
                % projecting the point clouds onto the RGB images.
                if strcmpi(cornerTech, 'SURF') || ...
                        strcmpi(cornerTech, 'Harris')
                    % This method uses the automatic corner detection technique,
                    % so we don't have to find out the 2D point.
                    points2DAnch = [];
                    points2DMoved = [];
                elseif strcmpi(cornerTech, 'UserDefined')
                    % In this case, we are not using the any automatic corner
                    % detection technique. Instead, we are providing the
                    % projection of 3D point on the RGB image as the corner 
                    % points.
                    points2DAnch = ProjectPCs2RGBImage(pcAnch, tformDepth2RGB);
                    points2DMoved = ProjectPCs2RGBImage(pcMoved, tformDepth2RGB);
                else
                    error('Matching type should be SURF | Harris | UserDefined');
                end
                
                % Find the matching point between two images.
                matchingStruct = struct('technique', cornerTech, ...
                    'points1', points2DAnch , 'points2', points2DMoved);
                [inlierPtsAnch, inlierPtsMoved] = FindMatchedPoints(rgbImgAnch, ...
                    rgbImgMoved, matchingStruct, geomParamsStruct, dispFlag.matchPair);
                
                % Estimate the initial guess for transformation between the two 
                % point cloud using the RGB matching points.
                pcStructAnch = struct('rgbPts', inlierPtsAnch, 'pc', pcAnch, ...
                    'tformDepth2RGB', tformDepth2RGB);
                pcStructMoved = struct('rgbPts', inlierPtsMoved, 'pc', pcMoved, ...
                    'tformDepth2RGB', tformDepth2RGB);
                [tformMoved2Anchor, matchPtsCount, regStats] = ...
                    EstimateTformMatchingRGB(pcStructAnch, pcStructMoved);
                
                % Run the ICP or similar algorithms to do a final registration
                % and then update the current transformation matrix.
                regStruct = struct('initTform', tformMoved2Anchor, ...
                    'icpMethod', 'pointToPlane', 'maxIter', 20);
                tformMoved2Anchor = RunRigidReg(pcAnch, pcMoved, regStruct);
                
                % Check the registration status, i.e., whether it succeeded or
                % failed and store the information into a file.
                logFileName = [dirStruct.dirName, '/', dirStruct.rtFolderName, ...
                    '/Log-', date, '.txt'];
                LogRegistrationStatus(regStats, pcNameAnch, pcNameMoved, ...
                    matchPtsCount, logFileName);
                
                % Save the transformation matrix into a file
                rtNameMoved = ['rt_', num2str(movedNum), '.txt'];
                rtFullNameMoved = [dirStruct.dirName, '/', ...
                    dirStruct.rtFolderName, '/', rtNameMoved];
                WriteRT(tformMoved2Anchor, rtFullNameMoved);
                
                % If needed display the point cloud
                if dispFlag.pcPair == 1
                    DisplayPCs(pcAnch, pcMoved, pcNameAnch, pcNameMoved,...
                        tformMoved2Anchor);
                end
                break;
            else
                % Go to the next image
                imgNumStruct.startIndx = imgNumStruct.startIndx + 1;
            end
        end
    else
        % Go to the next image
        imgNumStruct.startIndx = imgNumStruct.startIndx + 1;
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
