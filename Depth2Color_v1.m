function colorMat = Depth2Color_v1(depthMat)
% In this function, given a depth x,y coordinate, the corresponding color x,y
% coordinate will be evaluted.
%
% INPUT:
%   detphMat(N,2) = Each row represents a [x,y] coordinate in depth image.
%
% OUTPUT:
%   colorMat(N,2) = Each row will have a [x,y] coordinate in color image.