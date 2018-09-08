function pc_tformed = TransformPointCloud(pc, tform)
% In this function, a point cloud is goint to be tranformed using the equation:
%   pcLocation_tformed = R*pc.Location + T
% and the normals if present are transfromed using:
%   pcNormal_tformed = R*pc.Normal
%
% INPUT(s):
%   pc = Point cloud object
%   tform = Structure containing the Rotation matrix R and the Translation
%       vector T
%
% OUTPUT(s):
%   pc_tformed = Transformed point cloud object
%
% EXAMPLE:
%   

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
% First, transform the location using rotation and translation:
pcCount = size(pc.Location, 1);
pcLocation_tformed = tform.R * pc.Location' + repmat(tform.T, [1,pcCount]);
pcLocation_tformed = pcLocation_tformed';

% Transform the normals using the rotation matrices onle:
if ~isempty(pc.Normal)
    % Transform the normal
    pcNormal_tformed = tform.R * pc.Normal';
    pcNormal_tformed = pcNormal_tformed';
else
    pcNormal_tformed = [];
end

% Leave the color as it is:
pcColor_tformed = pc.Color;

% Create a final pointCloud object
pc_tformed = pointCloud(pcLocation_tformed, 'Color', pcColor_tformed, ...
    'Normal', pcNormal_tformed);
