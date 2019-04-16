function imgCounts = ConvertRawDepthRGB2ColorPly(dirName, maxDepthInMeters, KinectType, ...
    imgNumberStruct, denoiseParamsStruct, calibStereo)
% This function reads the depth and the corresponding RGB images and creates a
% colored point cloud. While creating the point cloud it takes care of the noise
% using a moving window approach.
%
% INPUT(s):
%   dirName     : Directory name containing the raw text/image files.
%   maxDepthInMeters : Point beyond this depth will be ignored.
%   KinectType  : Either Kinect-360(v1) or Kinect-ONE(v2)
%   imgNumberStruct : Structure holding the info about the images that need to
%               be processed. If this structure is empty then the all the images
%               inside the folder will be processed.
%       startIndx   : Starting number of the file which will be included in the
%                   complete 3D point cloud.
%       endIndx     : Last image number that need to be processed
%       samplingRate: The difference between two consequtive images (Default = 1)
%   denoiseParamsStruct   : Parameters for denoising the point cloud.
%       1) flyWinSize   : Window size which will be used to get rid of flying pixels
%       2) flyDistTh    : Threshold to determine whether to keep/discard pixels after
%                       the flying window operation
%   calibStereo : Calibration parameters of the Kinect that will be used in
%               processing the images.
%
% OUTPUT(s):
%   imgCounts   : Count of images that are being process
% Example(s):
%   imgCounts = ConvertRawDepthRGB2ColorPly('/media/tnb88/My Passport/PhD/
%       Data/Alvaro/2017_0825/103/SampleImages/', 2, 'v2', {},  
%       struct('flyWinSize', 3, 'flyDistTh', 1.2))

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------
if nargin < 3
    error(['At least provide DIRECTORY-NAME containing the images, ', ...
        'MAXIMUM-DEPTH to be captured, KINECT-TYPE']);
end

% Check and if needed, set the default parameters.
% Check for the calibration parameters
if nargin < 6 || isempty(calibStereo)
    calibStereo = ['~/Dropbox/PhD/Data/Calibration/', ...
        'Calibration_20181114/HandHeld/Calib_Results_stereo_rgb_to_ir.mat'];
    disp('WARNING!!! -- Loading the handheld kinect parameters');
end
% Check for image denoising parameters
if nargin < 5 || isempty(denoiseParamsStruct)
    disp(['WARNING!!! -- Using default values for denoising which are', ...
        'flyWinSize = 0.02 and flyDistTh = 3']);
    denoiseParamsStruct.flyWinSize = 3;
    denoiseParamsStruct.flyDistTh = 0.02;
end
% Check for the image numbers that need to be processed
if (nargin < 4 || isempty(imgNumberStruct))
    % First get all the depth image files inside the directory.
    if (strcmpi(KinectType, 'v1'))
        imgNumberStruct = FindImagesNumbers(dirName, 'ppm');
    elseif (strcmpi(KinectType, 'v2'))
        imgNumberStruct = FindImagesNumbers(dirName, 'png');
    end
    
    % Sampling rate -- Using kinect we can grab a lot a images but we
    % don't need all of them to create the complete point cloud. So,
    % take only few samples from the whole data set.
    imgNumberStruct = setfield(imgNumberStruct, 'samplingRate', 1);
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
imgCounts = 0;                  % Keeps track of how many images are processed
for iNTF=imgNumberStruct.startIndx:imgNumberStruct.samplingRate:imgNumberStruct.endIndx
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
                rgbImg, tformDepth2RGB, maxDepthInMeters,denoiseParamsStruct);
            
            % Increment the count
            imgCounts = imgCounts + 1;
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
