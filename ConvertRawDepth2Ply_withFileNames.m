function ConvertRawDepth2Ply(dirName, maxDepthInMeters, KinectType, ...
                            startIndx, endIndx, samplingRate, Mode)
% This function reads the text files which contain the raw depth and converts 
% them into a ply file. It also creates XYZ, Nor, Tri files for 3D model 
% creation.
% 
% INPUT:
%   dirName     : Directory name containing the raw text files.
%   maxDepthInMeters : Point beyond this maximum depth will be ignored.
%   startIndx   : Starting number of the file which will be included in the 
%               complete 3D point cloud.
%   numPCs      : Total number of point clouds from which the 3D model will be 
%               created.
%   samplingRate: The difference between two consecutive images.
%   KinectType  : Either Kinect-360(v1) or Kinect-ONE(v2)
%   Mode        : Now this program can read both text and image files. 
%               1 -- text files (Not a good idea to store image as a text file.
%               2 -- PPM image (Default)
%
% OUTPUTs:
%
% Example: ConvertRawDepth2Ply_v1('~/Desktop/test_images_July11_dusk/', 
%       1.5, 1, 100, 2)

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------
% Set the default parameters firs.
if (nargin < 7)
    % Read image files.
    Mode = 2;
    if (nargin < 6)
        % Sampling rate -- Using kinect we can grab a lot a images but we don't 
        % need all of them to create the complete point cloud. So, take only 
        % few samples from the whole data set.
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
else
    error(['At least provide directory name containing the images, maximum '
        'depth to be captured, Kinect type and starting image number']);
end

% Make a directory to store the ply files.
system(sprintf('mkdir %s/PCinPLY', dirName));
system(sprintf('mkdir %s/PCinXYZNorTri', dirName));

% For each name given in the list read the text file and convert the raw depth 
% into a X, Y, and Z coordinates and then create a ply file from the 
% coordinates. In the store the ply file in the newly created directory.
for iNTF=startIndx:samplingRate:endIndx
    if Mode == 1
        % Read the text file. Each text file contains a series of float values,
        % seperated by a ','.
        txtFileName = sprintf('rawDepth%04d.txt', iNTF);
        depthRaw = textread([dirName, '/', txtFileName], '', 'delimiter', ',');
    elseif Mode == 2
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
            % Now, get the X, Y, Z of each point in a world coordinate frame.
            [Xw, Yw, Zw] = Depth2World_v2(tmpFileName, maxDepthInMeters);
        else
            error('Kinect type should either v1 or v2');
        end
        
    end
%     dataXYZ = cat(2, Xw, Yw, Zw);
    dataXYZ = cat(2, Xw, Yw-maxDepthInMeters, Zw);
    
    % Check whether the cloud is empty or not. If it is empty then don't create
    % the point cloud or else create the ply files and the corresponding NOR,
    % TRI and XYZ.
    if ~isempty(dataXYZ)
        % Create a ply file out of these point.
        % Get rid of the file extension.
        if Mode ==1
            nameWithoutExt = strrep(txtFileName, '.txt', '');
        elseif Mode ==2
            if (strcmpi(KinectType, 'v1'))
                nameWithoutExt = strrep(ppmFileName, '.ppm', '');
            elseif (strcmpi(KinectType, 'v2'))
                nameWithoutExt = strrep(pngFileName, '.png', '');
            else
                error('Kinect type should either v1 or v2');
            end
        end
        fileName = [dirName, '/PCinPLY/', nameWithoutExt];
        pcwrite(pointCloud(dataXYZ), fileName);
%         pc2ply(fileName, dataXYZ);
        
        % Create XYZ, Nor and Tri files for each point cloud which could be used 
        % to register to with each other to create a complete 3D point cloud.
        binDir = [dirName, '/PCinXYZNorTri/'];
        createXYZTriNor(dataXYZ, binDir, iNTF);
    end
end
