function EstimateTform_Batch(dirName, startIndx, endIndx, pcDirName, ...
    geomEstParams, calibStereo, dispFlag, cornerDetectionTech)
% In this function, I am going to read two images from a given folder then
% estimate the transformation between the two using the RGB images and their
% corresponding point clouds.
% INPUT(s):
%   dirName         := Directory containing the RGB/Depth images and the
%       corresponding ply files.
%   startIndx       := File number of 1st pc
%   endIndx         := Last file number
%   pcDirName       := Directory that contains the 3D point clouds in ply format
%   geomEstParams   := Parameters for matching the two images and removing
%       outlier in pair of matching points.
%   calibStereo     := Stereo calibration parameters between IR and RGB of the 
%       Kinect that was used to collect the data.
%   dispFlag        := 0/1 to display the final registered point clouds
%   cornerDetectionTech     := Either automatic (SURF|Harris) or manual (UserDefine) 
%
% OUTPUT(s):
%
% Example:
%   dirName = '~/Dropbox/PhD/Data/Data/Alvaro/2017_0825/103/SampleImages/';
%   EstimateTform_Batch(dirName, 1250, 1259, [], [], [], 1, 'UserDefined')

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------
% Directory cotaining the images
if nargin < 3               
    error('Provide the directory name containing the images');
end
% Name of the folder where point clouds will be saved
if nargin < 4 || isempty(pcDirName)
    % This is the default name I have been using
    pcDirName = 'PCinPLY';
end
% Parameters to be used to match 2 rgb images
if nargin < 5 || isempty(geomEstParams)
    geomEstParams = struct('tformType', 'projective', 'maxDist', 3.5);
end
% Calibration parameters of the Kinect sensors
if nargin < 6 || isempty(calibStereo)
    calibStereo = ['~/Dropbox/PhD/Data/Calibration/', ...
        'Calibration_20181114/HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
    disp('WARNING!!! -- Loading the handheld kinect parameters');
end
% Display pair of point clouds.
if nargin < 7 || isempty(dispFlag)
    dispFlag = 0;       % By default don't display the point clouds
end
% Corner detection technique --
if nargin < 8 || isempty(cornerDetectionTech)
    cornerDetectionTech = 'SURF';
end

% Load a pair of images at a time and find the correspondence.
while startIndx < endIndx
    % In the begining, make the 1st found image as the anchor image. After that,
    % however, make the 'moved' image as the achor image.
    anchNum = startIndx;
    rgbNameAnch = ['rgbImg_', num2str(anchNum), '.jpg'];
    rgbFullNameAnch = [dirName, '/', rgbNameAnch];
    
    % IF the file is available read it and set it as the Anchor image and then
    % look for the Moved image.
    if exist(rgbFullNameAnch, 'file') == 2
        rgbImgAnch = imread(rgbFullNameAnch);               % Read 1st image
        
        % Check for the corresponding R|T text file. If it doesn't exist the
        % create one with default values.
        rtNameAnch = ['rt_', num2str(anchNum), '.txt'];
        rtFullNameAnch = [dirName, '/', pcDirName, '/', rtNameAnch];
        if ~(exist(rtFullNameAnch, 'file') == 2)
            WriteRT(struct('R', eye(3,3), 'T', zeros(3,1)), rtFullNameAnch);
        end
        
        % Now, look for the next RGB image
        startIndx = startIndx + 1;
        while startIndx < endIndx
            movedNum = startIndx;
            % Read anchored and moved point cloud
            rgbNameMoved = ['rgbImg_', num2str(movedNum), '.jpg'];
            rgbNameFullMoved = [dirName, '/', rgbNameMoved];
            
            % IF the file is found then set it as the moved point cloud and then
            % find the correspondence between the two images
            if exist(rgbNameFullMoved, 'file') == 2
                % Display the image names that were supposed to be matched
                fprintf('Matching -- %s and %s\n\n', rgbNameAnch, rgbNameMoved);
                
                % Read the second RGB image
                rgbImgMoved = imread(rgbNameFullMoved);     % Read 2nd image
                
                % Now, read the corresponding 3D point clouds
                pcNameAnch = ['depthImg_', num2str(anchNum), '.ply'];
                pcFullNameAnch = [dirName, '/', pcDirName, '/', pcNameAnch];
                pcAnch = pcread(pcFullNameAnch);
                pcNameMoved = ['depthImg_', num2str(movedNum), '.ply'];
                pcFullNameMoved = [dirName, '/', pcDirName, '/', pcNameMoved];
                pcMoved = pcread(pcFullNameMoved);
                
                % Also, read the calibraiton parameters and create a structure 
                % to holding transformation matrix, etc...
                % Load the transformaion parameters. If you have Depth-to-
                % RGB then use it directly or else take the inverse of RGB-
                % to-Depth parameters.
                load(calibStereo, 'R', 'T', 'KK_left', 'KK_right');
                % Create as strucutre that will hold R matrix & T vector and
                % the intrinsic parameters of each camera.
                tformDepth2RGB.R = inv(R);
                % Convert into Meters as the PC is in Meters
                tformDepth2RGB.T = -inv(R)*T/1000;
                tformDepth2RGB.KK_RGB = KK_left;
                tformDepth2RGB.KK_IR = KK_right;
                    
                % Prepare data for matching two RGB images -- Detect corners
                % either automatically (SURF | Harris) or provide them by
                % projecting the point clouds onto the RGB images.
                if strcmpi(cornerDetectionTech, 'SURF') || ...
                        strcmpi(cornerDetectionTech, 'Harris')
                    % This method uses the automatic corner detection technique,
                    % so we don't have to find out the 2D point.
                    points2DAnch = [];
                    points2DMoved = [];
                elseif strcmpi(cornerDetectionTech, 'UserDefined')
                    % In this case, we are not using the any automatic corner
                    % detection technique. Instead, we are providing the
                    % projection of 3D point on the RGB image as the corner 
                    % points.
                    
                    % Project those 3D points onto their corresponding images
                    % using the intrinsic and extrinsic matrix of the Kinect IR
                    % and RGB camera
                    pcAnchInRGBFrame = TransformPointCloud(pcAnch, tformDepth2RGB);
                    pcMovedInRGBFrame = TransformPointCloud(pcMoved, tformDepth2RGB);
                    
                    % Get the UV values of those projected points on the RGB
                    % images
                    rgbUVsAnch = ProjectPointsOnImage(pcAnchInRGBFrame.Location, ...
                        tformDepth2RGB.KK_RGB);
                    rgbUVsAnch = round(rgbUVsAnch);
                    rgbUVsMoved = ProjectPointsOnImage(pcMovedInRGBFrame.Location, ...
                        tformDepth2RGB.KK_RGB);
                    rgbUVsMoved = round(rgbUVsMoved);

                    % Create the point cloud object so that it could be used in
                    % feature extraction process.
                    points2DAnch = cornerPoints(rgbUVsAnch);
                    points2DMoved = cornerPoints(rgbUVsMoved);
                else
                    error('Matching type should be SURF | Harris | UserDefined');
                end
                
                % Find the matching point between two images.
                matchingStruct = struct('technique', cornerDetectionTech, ...
                    'points1', points2DAnch , 'points2', points2DMoved);
                [inlierPtsAnch, inlierPtsMoved] = FindMatchedPoints(...
                    rgbImgAnch, rgbImgMoved, matchingStruct, geomEstParams, 1);
                
                % Estimate the transfomation from the two point cloud using the
                % RGB matching points.
                pcStructAnch = struct('rgbPts', inlierPtsAnch, 'pc', pcAnch, ...
                    'tformDepth2RGB', tformDepth2RGB);
                pcStructMoved = struct('rgbPts', inlierPtsMoved, 'pc', pcMoved, ...
                    'tformDepth2RGB', tformDepth2RGB);
                [tformMoved2Anchor, matchPtsCount, regStats] = ...
                    EstimateTformMatchingRGB(pcStructAnch, pcStructMoved);
                % Check the registration status, i.e., whether it succeeded or
                % failed.
                if regStats ~= 0
                    disp(['Unable to register pc ', pcNameAnch, 'and', ...
                        pcNameMoved, 'as number of matching points were ', ...
                        num2str(matchPtsCount)])
                else
                    disp(['Number of matched points are: ', num2str(matchPtsCount)]);
                end
                
                % Save the transformation matrix into a file
                rtNameMoved = ['rt_', num2str(movedNum), '.txt'];
                rtFullNameMoved = [dirName, '/', pcDirName, '/', rtNameMoved];
                WriteRT(tformMoved2Anchor, rtFullNameMoved);
                
                % If needed display the point cloud
                if dispFlag == 1
                    pcMoved_Tformed = TransformPointCloud(pcMoved, ...
                        tformMoved2Anchor);
                    pcshowpair(pcAnch, pcMoved_Tformed);
                    legend('Anchor PC', 'Transformed PC');
                end
                break;
            else
                % Go to the next image
                startIndx = startIndx + 1;
            end
        end
    else
        % Go to the next image
        startIndx = startIndx + 1;
    end
end
