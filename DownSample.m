function matchPairWise = DownSample(matchPairWise, pcPairWise, maxNumPts)
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
% 2. pcPairWise (Mx6 table):
%   1) Anchor, Moved (cell) -- Image numbers of the anchor and moved image in
%   "string" format, respectively
%   2) Anch_PC, Moved_PC (pointCloud) -- 3D points for anchor and moved pc
%   3) Anch_PtsIndx, Moved_PtsIndx (Nx1 double) -- Indices of 3D points that 
%   have a matching pair.
%   4) Matched_Points (double) -- Number of matched points of between two images
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
    if (numPts > maxNumPts)
        % Load the full point clouds and get the one that matched in both pairs.
        pcAnch = pcPairWise.Anch_PC{inp};
        pcAnch = pcAnch.Location(pcPairWise.Anch_PtsIndx{inp}, :);
        
        % Down sample the same
        percentage = 50/numPts;
        pcAnch_DS = pcdownsample(pointCloud(pcAnch), 'random', percentage);
        
        % Find the index of the downsampled points
        Idx_AnMv = knnsearch(pcAnch, pcAnch_DS.Location);
        
        % Update the matching pixels of Anchor and Moved pc, also update the
        % count of number of matching points.
        matchPairWise.PtsPxls_Anch{inp,1}.indxPC = ...
            matchPairWise.PtsPxls_Anch{inp,1}.indxPC(Idx_AnMv, :);  % Ancho PC
        matchPairWise.PtsPxls_Anch{inp,1}.pixelsRGB = ...
            matchPairWise.PtsPxls_Anch{inp,1}.pixelsRGB(Idx_AnMv, :);
        
        matchPairWise.PtsPxls_Moved{inp,1}.indxPC = ...
            matchPairWise.PtsPxls_Moved{inp,1}.indxPC(Idx_AnMv, :); % Moved PC
        matchPairWise.PtsPxls_Moved{inp,1}.pixelsRGB = ...
            matchPairWise.PtsPxls_Moved{inp,1}.pixelsRGB(Idx_AnMv, :);
        
        matchPairWise.Matched_Points(inp) = length(Idx_AnMv);       % Total Points
    end
end
