function ConvertRawDepth2ColorPly(dirName, maxDepthInMeters, KinectType, ...
                            startIndx, endIndx, samplingRate, CenterFlag)
% This function reads the text files which contain the raw depth and converts 
% them into a ply file. It also creates XYZ, Nor, Tri files for 3D model 
% creation. For Kinect-v2, the libfreenect2 also provides a RGB image of same 
% size as the depth image. Store the RGB values along with the XYZ in the ply
% file.
% 
% INPUT:
%   dirName     : Directory name containing the raw text/image files.
%   maxDepthInMeters : Point beyond this depth will be ignored.
%   KinectType  : Either Kinect-360(v1) or Kinect-ONE(v2)
%   startIndx   : Starting number of the file which will be included in the 
%               complete 3D point cloud.
%   numPCs      : Total number of point clouds from which the 3D model will be 
%               created.
%   samplingRate: The difference between two consecutive images (Default = 1)
%   CenterFlag  : Whether or not to move the point cloud to the center 
%               (Default: No Centering)
%
% OUTPUTs:
%
% Example: 
%

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------
% Set the default parameters firs.
if (nargin < 8)
    % Read image files.
    Mode = 2;
    if (nargin < 7)
        CenterFlag = 'No Center';
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

% % Make a directory to store the ply files.
% system(sprintf('mkdir %s/PCinPLY', dirName));
% system(sprintf('mkdir %s/PCinXYZNorTri', dirName));

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
            mergedFileName = sprintf('mergedImg_%04d.jpg', iNTF);
            fullDepthFileName = [dirName, '/', depthFileName];
            fullMergedFileName = [dirName, '/', mergedFileName];
            if(exist(fullDepthFileName, 'file') == 2)
                flyDistTh = 1.2;
                flyWinSize = 3;
                depthImg = imread(fullDepthFileName);
                mergedImg = imread(fullMergedFileName);
                % Now, get the X, Y, Z of each point in a world coordinate frame.
                [Xw, Yw, Zw, Rw, Gw, Bw] = Depth2World_v2(depthImg, ...
                    maxDepthInMeters,flyWinSize, flyDistTh, mergedImg);
            else
                continue;
            end
        else
            error('Kinect type should either v1 or v2');
        end
  
    dataXYZ = cat(2, Xw, Yw, Zw);
%     dataXYZ = cat(2, Xw, Yw-maxDepthInMeters, Zw);
    dataRGB = cat(2, Rw, Gw, Bw);
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
        if Mode ==1
            nameWithoutExt = strrep(txtFileName, '.txt', '');
        elseif Mode ==2
            if (strcmpi(KinectType, 'v1'))
                nameWithoutExt = strrep(ppmFileName, '.ppm', '');
            elseif (strcmpi(KinectType, 'v2'))
                nameWithoutExt = strrep(depthFileName, '.png', '');
            else
                error('Kinect type should either v1 or v2');
            end
        end
        fileName = [dirName, '/PCinPLY/', nameWithoutExt];
        pcwrite(pointCloud(dataXYZ, 'Color', dataRGB), fileName);
%         pc2ply(fileName, dataXYZ);
        
        % Create XYZ, Nor and Tri files for each point cloud which could be used 
        % to register to with each other to create a complete 3D point cloud.
%         binDir = [dirName, '/PCinXYZNorTri/'];
%         CreateXYZTriNor(dataXYZ, binDir, iNTF);
    else
        disp("Couldn't create a point cloud because it's empty.");
    end
end
