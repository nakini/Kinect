function FindTransformation(dirName, startIndx, endIndx, geomEstParams)
% In this function, I am going to read two images from a given folder then
% estimate the transformation between the two using the RGB images and their
% corresponding point clouds.
% INPUT(s):
%   dirName         := Directory containing the RGB/Depth images and the
%       corresponding ply files.
%   startIndx       := File number of 1st pc
%   endIndx         := Last file number
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
    geomEstParams = struct('tformType', 'projective', 'maxDist', 3.5);
end

% Load a pair of images at a time and find the correspondence.
while startIndx < endIndx
    % In the begining, make the 1st found image as the anchor image. After that,
    % however, make the 'moved' image as the achor image.
    rgbNameAnch = ['rgbImg_', num2str(startIndx), '.jpg'];
    rgbFullNameAnch = [dirName, '/', rgbNameAnch];
    
    % IF the file is available read it and set it as the Anchor point cloud.
    if exist(rgbFullNameAnch, 'file') == 2
        rgbImgAnch = imread(rgbFullNameAnch);
        
        % Now, look for the next RGB image
        startIndx = startIndx + 1;
        while startIndx < endIndx
            % Read anchored and moved point cloud
            rgbNameMoved = ['rgbImg_', num2str(startIndx), '.jpg'];
            rgbNameFullMoved = [dirName, '/', rgbNameMoved];
            
            % IF the file is found then set it as the moved point cloud and then
            % find the correspondence between the two images
            if exist(rgbNameFullMoved, 'file') == 2
                % Display the image names that were matched
                fprintf('Matching -- %s and %s\n\n', rgbNameAnch, rgbNameMoved);
                
                % Read the second point cloud
                rgbImgMoved = imread(rgbNameFullMoved);
                
                % Find the matching point between two images.
                matchTech = 'SURF';
                [inlierPts1, inlierPts2] = FindMatchedPoints(rgbImgAnch, ...
                    rgbImgMoved, matchTech, geomEstParams, 1);
                
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