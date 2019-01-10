function Create3DModels(dirName, startIndx, endIndx, samplingRate)
% This function reads ply files generated from each depth image and subsequently
% calls the 'RegistrationGUI.mlapp' GUI app for each consecutive pair of plys
% and merges them together. In the end, it will create a complete 3D model. The
% complete 3D model will be created based on the 1st ply file.
%
% INPUT(s):
%   dirName: Name of the folder containing all the ply files
%
% OUTPUT(s):
%

%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% Read 1st ply file
while startIndx < endIndx
    pcName1 = sprintf('%s/depthImg_%d.ply', dirName, startIndx);
    if exist(pcName1, 'file') == 2
        % Read the point cloud and save it as the final model. Later on we can
        % keep adding the incoming ply files to the model.
        ptCloud1 = pcread(pcName1);
        pcwrite(ptCloud1, 'RegisteredPC.ply');
        
        % Create a text file with rotation and translation parameter. As this is
        % 1st one, the rotation and translation will be zero;
        rtName1 = sprintf('%s/rt_%d.txt', dirName, startIndx);
        rtMat = eye(4,3);       % First 3 lines for R, last line is T
        dlmwrite(rtName1,rtMat,'delimiter','\t','precision',6);
        break;
    else
        startIndx = startIndx + 1;
    end
end

% Start reading 2nd ply file onwards and adding to the 1st one
for imgNum = startIndx+1:samplingRate:endIndx
    ptCloud1 = pcread('RegisteredPC.ply');
    pcName2 = sprintf('%s/depthImg_%d.ply', dirName, imgNum);
    if exist(pcName2, 'file') == 2
        ptCloud2 = pcread(pcName2);
        LoadPointClouds(RegistrationGUI, ptCloud1, ptCloud2);
        
        % Rename the text file containing the rotation and translation
        % parameters.
        rtName2 = sprintf('%s/rt_%d.txt', dirName, imgNum);
        if exist('RT.txt', 'file') == 2
            movefile('RT.txt', rtName2);
        end
        
        % Finish one pair at a time.
        keyboard         
    else
        continue;
    end
end
pcName2 = sprintf('%s/Final.ply', dirName);
movefile('RegisteredPC.ply', pcName2);
