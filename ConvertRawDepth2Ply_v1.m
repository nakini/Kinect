function ConvertRawDepth2Ply_v1(dirName, maxDepthInMeters, startIndx, numPCs, ...
    samplingRate, Mode)
% This function reads the text files which contain the raw depth and converts 
% them into a ply file. It also creates XYZ, Nor, Tri files for 3D model 
% creation.
% 
% INPUT:
%   dirName = Directory name containing the raw text files.
%   maxDepthInMeters = Point beyond this maximum depth will
%   startIndx = Starting number of the file which will be included in the 
%       complete 3D point cloud.
%   numPCs = Total number of point clouds from which the 3D model will be created.
%   samplingRate = The difference between two consequtive images.
%   Mode = Now this program can read both text and image files. 
%       1 -- text files (Not a good idea to store image as a text file.
%       2 -- PPM image (Default)
%
%
% OUTPUTs:
%
% Example: ConvertRawDepth2Ply_v1('~/Desktop/test_images_July11_dusk/', 1.5, 1, 100, 2)

if (nargin < 4)
    % First get all the depth image files inside the directory.
    listTxtFiles = dir([dirName, '/*.ppm']);
    numPCs = length(listTxtFiles);
    
    % Sampling rate -- Using kinect we can grab a lot a images but we don't need all 
    % of them to create the complete point cloud. So, take only few samples from the 
    % whole data set.
    samplingRate = 1;
    
    % Read image files.
    Mode = 2;
else
    error(['At least provide directory name containing the images, maximum depth to be'
            ' captured and starting image number']);
end

% Make a directory to store the ply files.
system(sprintf('mkdir %s/PCinPLY', dirName));
system(sprintf('mkdir %s/PCinXYZNorTri', dirName));

% For each name given in the list read the text file and convert the raw depth 
% into a X, Y, and Z coordinates and then create a ply file from the coordinates. 
% In the store the ply file in the newly created directory.
for iNTF=startIndx:samplingRate:startIndx+numPCs-1
    if Mode == 1
        % Read the text file. Each text file contains a series of float values, 
        % seperated by a ','.
        txtFileName = sprintf('rawDepth%d.txt', iNTF);
        depthRaw = textread([dirName, '/', txtFileName], '', 'delimiter', ',');
    elseif Mode == 2
        % Read the ppm files. Each pixel in the ppm file is a depth value.
        ppmFileName = sprintf('depth%d.ppm', iNTF);
        depthRaw = imread([dirName, '/', ppmFileName]);
    end
    
    % Convert the raw depth into depth in meters.
    depthInMeters = RawDepth2Meters_v1(depthRaw);
        
    % Now, get the X, Y, Z of each point in a world coordinate frame.
    [Xw, Yw, Zw] = Depth2World_v1(depthInMeters, maxDepthInMeters);
%     dataXYZ = cat(2, Xw, Yw, Zw);
    dataXYZ = cat(2, Xw, Yw-maxDepthInMeters, Zw);
    
    if ~isempty(dataXYZ)
        % Create a ply file out of these point.
        % Get rid of the file extension.
        if Mode ==1
            nameWithoutExt = strrep(txtFileName, '.txt', '');
        elseif Mode ==2
            nameWithoutExt = strrep(ppmFileName, '.ppm', '');
        end
        fileName = [dirName, '/PCinPLY/', nameWithoutExt];
        pc2ply(fileName, dataXYZ);
        
        % Create XYZ, Nor and Tri files for each point cloud which could be used to 
        % register to with each other to create a complete 3D point cloud.
        binDir = [dirName, '/PCinXYZNorTri/'];
        createXYZTriNor(dataXYZ, binDir, iNTF);
    end
end
