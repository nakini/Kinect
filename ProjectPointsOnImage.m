function uvPixels = ProjectPointsOnImage(xyzPoints, K)
% Given the 3D points and the intrinsic matrix, transform the 3D points in the
% camera coordinate frame into the pixel coordinates.
%
% INPUT(s):
%   xyzPoints   : Nx3 matrix having 3D points
%   K           : 3x3 Camera intrinsic parameters
%
% OUTPUT(s):
%   uvPixels    : Nx2 matrix having the pixel coordinates

%^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
%------------------------------- START -----------------------------------------

uvWithScale = K * xyzPoints';
uvWithScale = uvWithScale';

uvWithScale = uvWithScale./(repmat(uvWithScale(:,3), [1,3]));
uvPixels = uvWithScale(:, 1:2);