function ConvertRawDepth2Ply(dirName, maxDepthInMeters, KinectType, ...
    startIndx, endIndx, samplingRate, denoiseParams, calibIR, CenterFlag)
% This function reads the text files which contain the raw depth and converts 
% them into a ply file. It also creates XYZ, Nor, Tri files for 3D model 
% creation.
% 
% INPUT:
%   dirName     : Directory name containing the raw text/image files.
%   maxDepthInMeters : Point beyond this depth will be ignored.
%   KinectType  : Either Kinect-360(v1) or Kinect-ONE(v2)
%   startIndx   : First file number that has to be evaluated
%   endIndx     : Last file number that needs to be processed
%   samplingRate: The difference between two consequtive images (Default = 1)
%   denoiseParams   : Parameters for denoising the point cloud.
%       1) flyWinSize   : Window size which will be used to get rid of flying pixels
%       2) flyDistTh    : Threshold to determine whether to keep/discard pixels after  
%                       the flying window operation
%   calibIR     : The mat file containing the calibration parameters of the IR
%               sensor.
%   CenterFlag  : Weither or not to move the point cloud to the center 
%               (Default: No Centering)
%
% OUTPUTs:
%
% Example(s):
%   dirName = '~/Desktop/Data/Alvaro/2017_0825/';
%   1. ConvertRawDepth2Ply([dirName, '105/SampleImages/'], 2, 'v2', 10121, 10141, 1);
%   2. ConvertRawDepth2Ply([dirName, '105/SampleImages/'], 2, 'v2', 10121, ...
%       10141, 1, [], []);
%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------
% Set the default parameters firs.
if (nargin < 9)
    CenterFlag = 'No Center';
    if nargin <= 8
        if nargin < 8  || isempty(calibIR)
        calibIR = [];
        disp(['WARNING!!! -- Loading default parameters obtained from the', ...
            'libfreenect-2 API using debug mode.']);
        end
        if nargin < 7 || isempty(denoiseParams)
            disp(['WARNING!!! -- Using default values for denoising which are', ...
                'flyWinSize = 3 and flyDistTh = 0.03']);
            denoiseParams.flyWinSize = 3;
            denoiseParams.flyDistTh = 0.02;
            if (nargin < 6)
                % Sampling rate -- Using kinect we can grab a lot a images but 
                % we don't need all of them to create the complete point cloud. 
                % So, take only few samples from the whole data set.
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
    end
else
    error(['At least provide DIRECTORY-NAME containing the images, ', ...
        'MAXIMUM-DEPTH to be captured, KINECT-TYPE and STARTING-IMAGE-NUM']);
end

% Make a directory to store the ply files.
system(sprintf('mkdir %s/PCinPLY', dirName));
system(sprintf('mkdir %s/PCinXYZNorTri', dirName));

% Load the intrinsic parameters of the IR camera.
if ~isempty(calibIR)
    load(calibIR, 'KK');
else
    KK = [];
end

% For each name given in the list read the text file and convert the raw depth 
% into a X, Y, and Z coordinates and then create a ply file from the 
% coordinates. In the store the ply file in the newly created directory.
for iNTF=startIndx:samplingRate:endIndx
    % Read the ppm files. Each pixel in the ppm file is a depth value.
    if (strcmpi(KinectType, 'v1'))
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
    elseif (strcmpi(KinectType, 'v2'))
        pngFileName = sprintf('depthImg_%04d.png', iNTF);
        tmpFileName = [dirName, '/', pngFileName];
        if(exist(tmpFileName, 'file') == 2)
            depthImg = imread(tmpFileName);
            % Now, get the X, Y, Z of each point in a world coordinate frame.
            [Xw, Yw, Zw] = Depth2World_v2(depthImg, maxDepthInMeters, ...
                denoiseParams.flyWinSize, denoiseParams.flyDistTh, [], KK);
        else
            continue;
        end
    else
        error('Kinect type should either v1 or v2');
    end
    dataXYZ = cat(2, Xw, Yw, Zw);
 
    % Center the point cloud, i.e., make the mean of the pc zero.
    if (strcmpi(CenterFlag , 'Center'))
        dataXYZ = dataXYZ - mean(dataXYZ);
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
            nameWithoutExt = strrep(pngFileName, '.png', '');
        else
            error('Kinect type should either v1 or v2');
        end
        fileName = [dirName, '/PCinPLY/', nameWithoutExt];
        pcwrite(pointCloud(dataXYZ), fileName);
        
        % Create XYZ, Nor and Tri files for each point cloud which could be used
        % to register to with each other to create a complete 3D point cloud.
        binDir = [dirName, '/PCinXYZNorTri/'];
        CreateXYZTriNor(dataXYZ, binDir, iNTF);
    end
end
