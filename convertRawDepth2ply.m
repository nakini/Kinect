function convertRawDepth2ply(dirName, maxDepthInMeters, startIndx, numPCs)
% This function reads the text files which contain the raw depth and converts them into a
% ply file.
% 
% INPUT:
%   dirName = Directory name containing the raw text files.
%   maxDepthInMeters = Point beyond this maximum depth will
%   startIndx = Starting number of the file which will be included in the complete 3D point
%       cloud.
%   numPCs = Total number of point clouds from which the 3D model will be created.


% First get all the text files inside the directory.
listTxtFiles = dir([dirName, '/*.txt']);

% Make a directory to store the ply files.
system(sprintf('mkdir %s/PCinPLY', dirName));
system(sprintf('mkdir %s/PCinXYZNorTri', dirName));

% For each name given in the list read the text file and convert the raw depth into a X,
% Y, and Z coordinates and then create a ply file from the coordinates. In the store the
% ply file in the newly created directory.
for iNTF=startIndx:startIndx+numPCs
    % Read the text file. Each text file contains a series of float values, seperated by a
    % ','.
    txtFileName = sprintf('rawDepth%d.txt', iNTF);
    depthRaw = textread([dirName, '/', txtFileName], '', 'delimiter', ',');
    % Convert the raw depth into depth in meters.
    depthInMeters = rawDepth2Meters(depthRaw);
    
    % Now, get the X, Y, Z of each point in a world coordinate frame.
    [Xw Yw Zw] = depth2World(depthInMeters, maxDepthInMeters);
%     dataXYZ = cat(2, Xw, Yw, Zw);
    dataXYZ = cat(2, Xw, Yw-maxDepthInMeters, Zw);
    
    if ~isempty(dataXYZ)
        % Create a ply file out of these point.
        nameWithoutExt = strrep(txtFileName, '.txt', ''); % Get rid of the file extension.
        fileName = [dirName, '/PCinPLY/', nameWithoutExt];
        pc2ply(fileName, dataXYZ);
        
        % Create XYZ, Nor and Tri files for each point cloud which could be used to register
        % to with each other to create a complete 3D point cloud.
        binDir = [dirName, '/PCinXYZNorTri/'];
        createXYZTriNor(dataXYZ, binDir, iNTF);
    end
end
