function FindTransformation_PairWise(dirName, startIndx, endIndx)
% This function reads ply files generated from each depth image and subsequently
% calls the 'RegistrationGUI.mlapp' GUI app to register two pairs of point 
% cloudsfor, say Model and Data point cloud. The transformation will be from 
% Data to Model pc.
%
% First, it will load 1st and 2nd pc in which the 1st will become the Model and 
% the 2nd will become the Data pc. Using the GUI it will find out the 
% transformation from 2nd to 1st. Then it will load 2nd and 3rd in which 2nd will
% become the Model and 3rd will become Data. Again, the transformation will be 
% calculated from 3rd to 2nd. This process will be repeated untill all pair wise
% transformation is evaluated.
%
% INPUT(s):
%   dirName         := Name of the folder containing all the ply files
%   startIndx       := File number of 1st pc
%   endIndx         := Last file number
%   samplingRate    := Number of files that need to be skipped
%
% OUTPUT(s):
%

%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% The outer while loop will look for the first existing pc and the inner loop
% will check for the 2nd pc. The 1st pc will be termed as the Anchor and the 2nd
% will be the Moved pc. Always load 2 files at a time and find the
% correspondence between them, such as at first find the corresnpondence between
%  1st and 2nd pc, then 2nd and 3rd, then 3rd and 4th, so on and so forth.
while startIndx < endIndx
    % Begin the search for the 1st point cloud
    pcNameAnch = ['depthImg_', num2str(startIndx), '.ply'];
    pcFullNameAnch = [dirName, '/', pcNameAnch];
    
    % IF the file is available read it and set it as the Anchor point cloud.
    if exist(pcFullNameAnch, 'file') == 2
        ptCloudAnch = pcread(pcFullNameAnch);
        rtFullNameAnch = [dirName, '/', 'rt_', num2str(startIndx), '.txt'];
        rtStrcutAnch = ReadTransformationFile(rtFullNameAnch);
        
        % Now, look for the 2nd point cloud.
        startIndx = startIndx + 1;
        while startIndx < endIndx
            % Read anchored and moved point cloud
            pcNameMoved = ['depthImg_', num2str(startIndx), '.ply'];
            pcNameFullMoved = [dirName, '/', pcNameMoved];
            
            if exist(pcNameFullMoved, 'file') == 2
                % Read the second point cloud
                ptCloudMoved = pcread(pcNameFullMoved);
                
                % Also read the corresponding R & T parameters for the same.
                rtFullNameMoved = [dirName, '/', 'rt_', num2str(startIndx), '.txt'];
                rtStructMoved = ReadTransformationFile(rtFullNameMoved);
                
                % Create a final structure for the GUI
                ptAnchor = struct('data', ptCloudAnch, 'transform', ...
                    rtStrcutAnch, 'name', pcNameAnch);
                ptMoved = struct('data', ptCloudMoved, 'transform', ...
                    rtStructMoved, 'name', pcNameMoved);
                
                % Load the GUI and wait until the job is done.
                LoadPointClouds(RegistrationGUI, ptAnchor, ptMoved);
                
                % Finish one pair at a time.
                keyboard;               % To get out of it use "dbcont"
                
                % If you happen to save the final pc then update the R/T values 
                % in the rt_*.txt file.
                if exist('/tmp/RT.txt', 'file') == 2
                    movefile('/tmp/RT.txt', rtFullNameMoved);
                end
                break;
            else
                startIndx = startIndx + 1;
            end
        end
    end
end
end

function trnsForm = ReadTransformationFile(rtFileName)
% This function will create text file R as 3x3 identity matrix and T a <0,0,0>
% vector. Then it will save the file with the name provided as a parameter.
trnsForm = struct('R', eye(3,3), 'T', zeros(3,1));

% If file already exists then don't create a new one.
if ~(exist(rtFileName, 'file') == 2)
    dlmwrite(rtFileName, eye(4,4), 'delimiter', '\t');
end
end
