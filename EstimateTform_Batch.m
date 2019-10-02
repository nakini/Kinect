function EstimateTformMatchingRGBs(dirName, startIndx, endIndx, pcDirName, geomEstParams)
% In this function, I am going to read two images from a given folder then
% estimate the transformation between the two using the RGB images and their
% corresponding point clouds.
% INPUT(s):
%   dirName         := Directory containing the RGB/Depth images and the
%       corresponding ply files.
%   startIndx       := File number of 1st pc
%   endIndx         := Last file number
%   pcDirName       := Directory that contains the 3D point clouds in ply format
%   geomEstParams   := Parameters for matching the two images and removing
%       outlier in pair of matching points.
%
% OUTPUT(s):
%
% Example:
%   dirName = ~/Dropbox/PhD/Data/Data/Alvaro/2017_0825/103/SampleImages/;
%   geomEstParams = struct('tformType', 'projective', 'maxDist', 50.5);
%   FindTransformation(dirName, 1250, 1262, geomEstParams)

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------

if nargin < 3
    error('Provide the directory name containing the images');
end
if nargin < 4
    % This is the default name I have been using
    pcDirName = 'PCinPLY';
end
if nargin < 5
    geomEstParams = struct('tformType', 'projective', 'maxDist', 3.5);
end

% Load a pair of images at a time and find the correspondence.
while startIndx < endIndx
    % In the begining, make the 1st found image as the anchor image. After that,
    % however, make the 'moved' image as the achor image.
    anchNum = startIndx;
    rgbNameAnch = ['rgbImg_', num2str(anchNum), '.jpg'];
    rgbFullNameAnch = [dirName, '/', rgbNameAnch];
    
    % IF the file is available read it and set it as the Anchor image and then
    % look for the Moved image.
    if exist(rgbFullNameAnch, 'file') == 2
        rgbImgAnch = imread(rgbFullNameAnch);               % Read 1st image
        
        % Now, look for the next RGB image
        startIndx = startIndx + 1;
        while startIndx < endIndx
            movedNum = startIndx;
            % Read anchored and moved point cloud
            rgbNameMoved = ['rgbImg_', num2str(movedNum), '.jpg'];
            rgbNameFullMoved = [dirName, '/', rgbNameMoved];
            
            % IF the file is found then set it as the moved point cloud and then
            % find the correspondence between the two images
            if exist(rgbNameFullMoved, 'file') == 2
                % Display the image names that were matched
                fprintf('Matching -- %s and %s\n\n', rgbNameAnch, rgbNameMoved);
                
                % Read the second point cloud
                rgbImgMoved = imread(rgbNameFullMoved);     % Read 2nd image
                
                % Find the matching point between two images.
                matchTech = 'SURF';
                [inlierPts1, inlierPts2] = FindMatchedPoints(rgbImgAnch, ...
                    rgbImgMoved, matchTech, geomEstParams, 1);
                
                % Now read the 3D point clouds
                pcNameAnch = ['depthImg_', num2str(anchNum), '.ply'];
                pcFullNameAnch = [dirName, '/', pcDirName, '/', pcNameAnch];
                pcNameMoved = ['depthImg_', num2str(movedNum), '.ply'];
                pcFullNameMoved = [dirName, '/', pcDirName, '/', pcNameMoved];
                
                break;
            else
                % Go to the next image
                startIndx = startIndx + 1;
            end
        end
    else
        % Go to the next image
        startIndx = startIndx + 1;
    end
end

        
        
        

% Now, read the corresponding two ply files along with the depth images

% Then find out the 3D points corresponding to each RGB image pixel

% Then estimate the transformation between the two point clouds.

% In the end, display the two registered point clouds.