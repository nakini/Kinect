function pcFinal = StitchTwoPointClouds(pc1, pc2, tform_PC1toPC2, ...
    downsampleType, downsampleArg)
% This function transforms point cloud 1 into the cooridinate of point cloud 2.
% The transformation function is from PC1 to PC2.
%
% INPUTs:
%   pc1     : pointCloud object that has Location, Color, Normal, etc for view-1
%   pc2     : pointCloud object for view-2
%   tform_PC1toPC2 : Structure which contains a 3x3 rotation matrix(R) and 3x1
%       translation vector(T). These are used to transforms PC1 into the view 
%       of PC2
%
% OUTPUTs:
%   pcFinal : pointCloud object, combined pc of both views

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
if nargin == 3
    downsampleType = 'random';
    downsampleArg = 0.5;
elseif nargin ~= 5
    error('ERROR!!! -- Atleast, provide two point clouds and a transformation matrix');
end

% Transform the 'Location' in pc1 into frame of pc2 such that:
% pc_tformed = R*pc1 + T
% Before using the Translation vector convert it into Meters from milli-meters
pc1Count = size(pc1.Location, 1);
pc1_tformed = tform_PC1toPC2.R * pc1.Location' + repmat(tform_PC1toPC2.T/1000, ...
    [1,pc1Count]);

% Merge the Location info:
pcFinal_Location = cat(1, pc1_tformed', pc2.Location);

% If the normal information is available for both the PCs then, transform the
% Normal in pc1 into pc2 using the same equation as the above one.
if ~isempty(pc1.Normal) && ~isempty(pc2.Normal)
    % Transform the normal
    pc1Normal_tformed = tform_PC1toPC2.R * pc1.Normal';
    
    % Merge the normal information
    pcFinal_Normal = cat(1, pc1Normal_tformed, pc2.Normal);
else
    pcFinal_Normal = [];
end

% If color informatino for both the PCs are available then add the color info to
% the merged cloud.
if ~isempty(pc1.Color) && ~isempty(pc2.Color)
    pcFinal_Color = cat(1, pc1.Color, pc2.Color);
end

% Create a final pointCloud object
pcFinal = pointCloud(pcFinal_Location, 'Color', pcFinal_Color, 'Normal', ...
    pcFinal_Normal);

% Downsample the PC if needed by the user
if strcmpi(downsampleType, 'random') || strcmpi(downsampleType, 'gridAverage') ...
        || strcmpi(downsampleType, 'nonuniformGridSample')
    pcFinal = pcdownsample(pcFinal, downsampleType, downsampleArg);
end
