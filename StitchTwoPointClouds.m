function finalPC = TransformPointCloud(pc1, pc2, tform_PC1toPC2)
% This function transforms point cloud 1 into the cooridinate of point cloud 2.
% The transformation function is from PC1 to PC2.
%
% INPUTs:
%   pc1     : Nx3 matrix containing X,Y,Z of view-1
%   pc2     : Mx3 matrix for view-2
%   tform_PC1toPC2 : Structure which contains a 3x3 rotation matrix(R) and 3x1
%       translation vector(T). These are used to transforms PC1 into the view 
%       of PC2
%
% OUTPUTs:
%   pcFinal : (N+M)x3 matrix, combined pc of both views

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------
if nargin ~= 3
    error('ERROR!!! -- Provide two point clouds and a transformation matrix');
end

% Transform the pc1 into frame of pc2 such that pc_tformed = R*pc1 + T
pc1Count = size(pc1, 1);
pc1_tformed = tform_PC1toPC2.R * pc1' + repmat(tform_PC1toPC2.T', [pc1Count,1]);

% Append the point clouds
finalPC = cat(1, pc1_tformed', pc2);