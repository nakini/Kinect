function ConvertRawDepthRGB2ColorPly(dirName, maxDepthInMeters, KinectType, ...
    startIndx, endIndx, samplingRate, denoiseParams, calibStereo)
% This function reads the depth and the corresponding RGB images and creates a
% colored point cloud. While creating the point cloud it takes care of the noise
% using a moving window approach.
%
% INPUT:
%   dirName     : Directory name containing the raw text/image files.
%   maxDepthInMeters : Point beyond this depth will be ignored.
%   KinectType  : Either Kinect-360(v1) or Kinect-ONE(v2)
%   startIndx   : Starting number of the file which will be included in the
%               complete 3D point cloud.
%   numPCs      : Total number of point clouds from which the 3D model will be
%               created.
%   samplingRate: The difference between two consequtive images (Default = 1)
%   denoiseParams   : Parameters for denoising the point cloud.
%       1) flyWinSize   : Window size which will be used to get rid of flying pixels
%       2) flyDistTh    : Threshold to determine whether to keep/discard pixels after
%                       the flying window operation
%
% OUTPUTs:
%
% Example:
%

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------
% Set the default parameters firs.
if nargin < 8 || isempty(calibStereo)
    calibStereo = ['~/Dropbox/PhD/Data/Calibration/', ...
        'Calibration_20181114/HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
    disp('WARNING!!! -- Loading the handheld kinect parameters');
    if nargin < 7 || isempty(denoiseParams)
        disp(['WARNING!!! -- Using default values for denoising which are', ...
            'flyWinSize = 0.02 and flyDistTh = 3']);
        denoiseParams.flyWinSize = 3;
        denoiseParams.flyDistTh = 0.02;
        if (nargin < 6)
            % Sampling rate -- Using kinect we can grab a lot a images but we
            % don't need all of them to create the complete point cloud. So,
            % take only few samples from the whole data set.
            samplingRate = 1;
            if (nargin < 5)
                % First get all the depth image files inside the directory.
                if (strcmpi(KinectType, 'v1'))
                    listTxtFiles = dir([dirName, '/*.ppm']);
                elseif (strcmpi(KinectType, 'v2'))
                    listTxtFiles = dir([dirName, '/*.png']);
                end
                endIndx = length(listTxtFiles);
            end
        end
    end
else
    error(['At least provide DIRECTORY-NAME containing the images, ', ...
        'MAXIMUM-DEPTH to be captured, KINECT-TYPE and STARTING-IMAGE-NUM']);
end

% Make a directory to store the ply files.
[~, msg, ~] = mkdir([dirName, '/PCinPLY']);
if ~isempty(msg)
    disp(msg);
end
[~, msg, ~] = mkdir([dirName,'/PCinXYZNorTri']);
if ~isempty(msg)
    disp(msg);
end

% For each name given in the list read the depth image file and convert the raw
% depth into a X, Y, and Z coordinates. Also read the corresponding merged
% image file to get the R, G and B values. In the end, create a ply file from
% the coordinates with the color information.
for iNTF=startIndx:samplingRate:endIndx
    % Read the ppm files. Each pixel in the ppm file is a depth value.
    if (strcmpi(KinectType, 'v1'))          %% KINECT-V1
        ppmFileName = sprintf('depth%04d.ppm', iNTF);
        compFileName = [dirName, '/', ppmFileName];
        if (exist(compFileName, 'file') == 2)
            depthRaw = imread(compFileName);
        else
            continue;
        end
        % Convert the raw depth into depth in meters.
        depthInMeters = RawDepth2Meters_v1(depthRaw);
        % Now, get the X, Y, Z of each point in a world coordinate frame.
        [Xw, Yw, Zw] = Depth2World_v1(depthInMeters, maxDepthInMeters);
    elseif (strcmpi(KinectType, 'v2'))      %% KINECT-V2
        depthFileName = sprintf('depthImg_%04d.png', iNTF);
        rgbFileName = sprintf('rgbImg_%04d.jpg', iNTF);
        fullDepthFileName = [dirName, '/', depthFileName];
        fullRGBFileName = [dirName, '/', rgbFileName];
        % Check whether the file exists or not. If not simply go to the next
        % one. If it is available then process the same.
        if(exist(fullDepthFileName, 'file') == 2)
            depthImg = imread(fullDepthFileName);
            rgbImg = imread(fullRGBFileName);
            % Load the transformaion parameters. If you have Depth-to-RGB
            % then use it directly or else take the inverse of RGB-to-Depth
            % parameters.
            load(calibStereo, 'R', 'T', 'KK_left', 'KK_right');
            % Create as strucutre that will hold R matrix and T vector only.
            tformDepth2RGB.R = inv(R);
            % Convert into Meters as the PC is in Meters
            tformDepth2RGB.T = -inv(R)*T/1000;
            tformDepth2RGB.KK_RGB = KK_left;
            tformDepth2RGB.KK_IR = KK_right;
            
            % Now, get the X, Y, Z of each point in a world coordinate frame.
            [~, dataXYZ, dataRGB] = MapColorFrameToDepthSpace(depthImg, ...
                rgbImg, tformDepth2RGB, maxDepthInMeters,denoiseParams);
        else
            continue;
        end
    else
        error('Kinect type should either v1 or v2');
    end
    
    % Check whether the cloud is empty or not. If it is empty then don't create
    % the point cloud or else create the ply files and the corresponding NOR,
    % TRI and XYZ.
    if ~isempty(dataXYZ)
        % Create a ply file out of these point.
        % Get rid of the file extension.
        if (strcmpi(KinectType, 'v1'))
            nameWithoutExt = strrep(ppmFileName, '.ppm', '');
        elseif (strcmpi(KinectType, 'v2'))
            nameWithoutExt = strrep(depthFileName, '.png', '');
        else
            error('Kinect type should either v1 or v2');
        end
        
        fileName = [dirName, '/PCinPLY/', nameWithoutExt];
        pcwrite(pointCloud(dataXYZ, 'Color', dataRGB), fileName);
        
        % Create XYZ, Nor and Tri files for each point cloud which could be used
        % to register to with each other to create a complete 3D point cloud.
%         binDir = [dirName, '/PCinXYZNorTri/'];
%         CreateXYZTriNor(dataXYZ, binDir, iNTF);
    else
        disp("Couldn't create a point cloud because it's empty.");
    end
end
