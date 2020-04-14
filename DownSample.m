function matchPairWise_DS = DownSample(matchPairWise)
% BRIEF
% =====
%
% INPUT(s)
% ========
% 1. matchPairWise (Mx8 table):
%   1) Anchor, Moved (cell) -- Image numbers of the anchor and moved image in
%   "string" format, respectively
%   2) Matched_Points (double) -- Number of matched points of between two images
%   3) ICP_RMSE (double) -- the "rmse" value from rigid registration
%   4) PtsPxls_Anch, PtsPxls_Moved -- Matched pixels info of the anchor and the
%   moved image, respectively. And each element of the column is a structure,
%   which holds the following fields.
%       a) indxPC (Px1 integer vector) -- Indices of matched 3D points of PC
%       b) pixelsRGB (Px2 integer matrix) -- 2D pixels of matched RGB image
%   5) Anchor_ViewID, Moved_ViewID (double) -- View ids of anchor and moved pc
%   for each pair, respectively.
%
% OUTPUT(s)
% =========
% 1. matchPairWise (Mx8 table): Contains the same elements as the input.
%
% Example(s)
% ==========
%

%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

numPairs = height(matchPairWise);
for inp = 1:numPairs
    numPts = matchPairWise.Matched_Points(inp);
    if (numPairs > 50)
        
    end
end