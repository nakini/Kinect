function Create3DModels(dirName, startIndx, endIndx, samplingRate)
% This function reads ply files generated from each depth image and subsequently
% calls the 'RegistrationGUI.mlapp' GUI app for each consecutive pair of plys
% and merges them together. In the end, it will create a complete 3D model. The
% complete 3D model will be created based on the 1st ply file.
%
% INPUT(s):
%   dirName     := Name of the folder containing all the ply files
%   startIndx   := File number of 1st pc
%   endIndx     := Last file number
%   samplingRate:= Number of files that need to be skipped
%
% OUTPUT(s):
%
%
% Example(s):
%
%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% Read 1st ply file and the transformation matrix
while startIndx < endIndx
    pcNameAnch = ['depthImg_', num2str(startIndx), '.ply'];
    pcFullNameAnch = [dirName, '/', pcNameAnch];
    % IF the file is available read it and set it as the Anchor point cloud.
    if exist(pcFullNameAnch, 'file') == 2
        ptCloudAnch = pcread(pcFullNameAnch);
        rtFullNameAnch = [dirName, '/', 'rt_', num2str(startIndx), '.txt'];
        rtStrcutAnch = ReadTransformationFile(rtFullNameAnch);
        break;
    else
        startIndx = startIndx + 1;
    end
end

% Start reading 2nd ply file onwards and adding to the 1st one
for imgNum = startIndx+1:samplingRate:endIndx
    % Read anchored and moved point cloud
    pcNameMoved = ['depthImg_', num2str(imgNum), '.ply'];
    pcNameFullMoved = [dirName, '/', pcNameMoved];
    
    if exist(pcNameFullMoved, 'file') == 2
        % Read the second point cloud
        ptCloudMoved = pcread(pcNameFullMoved);
        
        % Also read the corresponding R & T parameters for the same.
        rtNameFullMoved = [dirName, '/', 'rt_', num2str(imgNum), '.txt'];
        rtStructMoved = ReadTransformationFile(rtNameFullMoved);
        
        % Create a final structure for the GUI
        ptAnchor = struct('data', ptCloudAnch, 'transform', rtStrcutAnch, ...
            'name', pcNameAnch);
        ptMoved = struct('data', ptCloudMoved, 'transform', rtStructMoved, ...
            'name', pcNameMoved);
        
        % Load the GUI and wait until the job is done.
        LoadPointClouds(RegistrationGUI, ptAnchor, ptMoved);
        % Finish one pair at a time.
        keyboard
        
        % If you happen to save the final pc then update the R/T values in the
        % rt_*.txt file.
        if exist('/tmp/RT.txt', 'file') == 2
            movefile('/tmp/RT.txt', rtNameFullMoved);
        end
    else
        continue;
    end
end

if exist('/tmp/RegisteredPC.ply', 'file') ==2
    pcNameFullMoved = sprintf('%s/Final.ply', dirName);
    movefile('RegisteredPC.ply', pcNameFullMoved);
end
end

function trnsForm = ReadTransformationFile(rtFileName)
% This function will read the pre-stored RT file and return the R and T info as
% a structure. If the R/T values are already available in a text file then read 
% it or else create a structure with default values.
if exist(rtFileName, 'file') == 2
    rtMat = dlmread(rtFileName);
    trnsForm.R = rtMat(1:3, 1:3);
    trnsForm.T = rtMat(4, 1:3)';
else
    trnsForm = struct('R', eye(3,3), 'T', zeros(3,1));
    dlmwrite(rtFileName, eye(4,4), 'delimiter', '\t');
end

end
