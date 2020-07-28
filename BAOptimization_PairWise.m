function [xyzRefined, rtRefined_PairWise, resnorm, residual, exitflag, output, ...
    lambda,jacobian] = BAOptimization_PairWise(pcPairWise, rtPairWise, ...
    matchPairWise, camIntrinsic, xyzFlag)
% BRIEF
% =====
% In this function, given the pair-wise transformation matrices, the 3D points
% and the matching pixel coordinates, I will do the followings:
%   1. convert the rotation matrices to euler angles
%   2. create a vector having the all the variables that need to be optimized,
%   such as 3D points, and pairwise camera poses.
%   3. write an objective function
%   4. call "lsqnonlin" optimizer with various options and update the initial
%   guess evaluted during coarse registration
%   5. Return the updated XYZs and R|T parameters.
%
% INPUT(s)
% ========
%   1. pcPairWise (table): 3D point clouds in their camera coordinate frame
%
%   2. rtPairWise (table): It will have view-ids along with the
%   transfomation matrices which transform the from current view to global view.
%
%   3. matchPairWise (table): Holds matched pixels, view-ids, etc.
%
%   4. camIntrinsic (3x3 matrix): Camera intrinsic parameters
%
%   5. optParams (struct): Optimization parameters for the "lsqnonlin"
%   optimization function
%
% OUTPUT(s)
% =========
%   1. xyzRefined (Nx3 double): Updated 3D coordiantes
%
%   2. rtRefined (table): Refined R(3x3)|T(3x1) parameters
%
%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% Create a long vector will all the 3D points -- [x-1, y-1, z-1, x-2, y-2, z-2,
% x-3, y-3, z-3, ...]. Include both Anchor and Moved pcs.
matchPtsCount = matchPairWise.Matched_Points;
numPairs = length(matchPtsCount);
rtVect_Init = zeros(1, 6*numPairs);                 % All paired H matrices
xyzVect_Init = zeros(2*sum(matchPtsCount), 3);      % 2 Pairs of pcs for a pair of images
currIndxRT = 1;
currIndx3D = 1;
for iNP = 1:numPairs
    % Save H Matrix
    % =============
    % Convert the rotation into RPY and store it along with T in a vector form
    tmpRPY = rotm2eul(rtPairWise.Orientation{iNP, 1});
    tmpLoc = rtPairWise.Location{iNP, 1};
    rtVect_Init(1, currIndxRT:currIndxRT+5) = [tmpRPY, tmpLoc];
    currIndxRT = currIndxRT + 6;
    
    % Anchor pc
    % =========
    tmpAPC = pcPairWise.Anch_PC{iNP,1};
    xyzVect_Init(currIndx3D:currIndx3D + matchPtsCount(iNP)-1, :) = ...
        tmpAPC.Location(pcPairWise.Anch_PtsIndx{iNP,1}, :);
    currIndx3D = currIndx3D + matchPtsCount(iNP);
    
    % Moved PC
    % ========
    tmpMPC = pcPairWise.Moved_PC{iNP,1};
    xyzVect_Init(currIndx3D: currIndx3D + matchPtsCount(iNP)-1, :) = ...
        tmpMPC.Location(pcPairWise.Moved_PtsIndx{iNP,1},:);
    currIndx3D = currIndx3D + matchPtsCount(iNP);
end

xyzVect_Init = reshape(xyzVect_Init', [1, numel(xyzVect_Init)]);

% Number of matched point count per view.
matchedPtsCount = matchPairWise.Matched_Points';

% Optimzation objective function and initial guess parameters
if xyzFlag == true
    initial_guess = [rtVect_Init, xyzVect_Init]; % Has both XYZs' and RTs'
else
    initial_guess = rtVect_Init;                 % Only RTs' for experiment purpose
end

% Objective function handle

% Optimization
fun = @(x)ObjectiveFun(x, matchPairWise, camIntrinsic, xyzFlag, xyzVect_Init);

% Optimation options:
% Algorithm: trust-region-reflective or levenberg-marquardt
options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt', ...
    'Display','iter', 'StepTolerance', 1e-10, 'FunctionTolerance', 1e-10, ...
    'OptimalityTolerance', 1e-10, 'MaxIterations', 3, 'Diagnostics', 'on');

% Optimization
[final_vals, resnorm, residual, exitflag, output, lambda,jacobian] = ...
    lsqnonlin(fun, initial_guess, [], [], options);

% Convert the output vector into R|T table and XYZ matrix.
[rtRefined_PairWise, xyzRefined] = ExtractRTXYZ(final_vals, matchedPtsCount, ...
    xyzVect_Init, xyzFlag);

end

%% Objective Function
function resError = ObjectiveFun(x, matchPairWise, camIntrinsic, xyzFlag, xyzInp)

% Number of matched point count per view.
matchedPtsCount = matchPairWise.Matched_Points';
numPairs = length(matchedPtsCount);
[rtPW_Init, xyzPW_Init] = ExtractRTXYZ(x, matchedPtsCount, numPairs, ...
    xyzFlag, xyzInp);

numPoints = 2*sum(matchedPtsCount);
resError = zeros(numPoints, 2);
currIndx = 1;
for iNP = 1:numPairs
    % Point clouds
    pcAnch = xyzPW_Init(currIndx:currIndx+matchedPtsCount(iNP)-1, :);
    currIndx = currIndx+matchedPtsCount(iNP);
    pcMov = xyzPW_Init(currIndx:currIndx+matchedPtsCount(iNP)-1, :);
    currIndx = currIndx-matchedPtsCount(iNP);   % Reset the index for resError matrix
    
    % Corresponding Matching pixels
    uvAnch = matchPairWise.PtsPxls_Anch{iNP, 1};
    uvMov = matchPairWise.PtsPxls_Moved{iNP, 1};
    
    
    % Project onto image plane
    % ========================
    % 2 cases --> Case-1: Project i-th and j-th image points onto i-th and j-th
    % image, respectively. Here, the error should be ZERO. Case-2: Cross
    % projection. Poject i-th pc onto j-th image and vice versa.
%     uvAnch_Proj = ProjectPointsOnImage(pcAnch, camIntrinsic);   % Case-1
%     uvMov_Proj = ProjectPointsOnImage(pcMov, camIntrinsic);

    % % Case-2: Transform the pc from moved-to-anchor frame
    R = rtPW_Init{iNP, 1}';
    T = rtPW_Init{iNP, 2};
    tmpRTMov2Anch = struct('R', R, 'T', T);
    tmpRTAnch2Mov = struct('R', R', 'T', -R'*T);
    
    pcMov_onAnch = TransformPointCloud(pointCloud(pcMov), tmpRTMov2Anch);
    uvAnch_Proj = ProjectPointsOnImage(pcMov_onAnch.Location, camIntrinsic);   
    
    pcAnch_onMov = TransformPointCloud(pointCloud(pcAnch), tmpRTAnch2Mov);
    uvMov_Proj = ProjectPointsOnImage(pcAnch_onMov.Location, camIntrinsic);
    
    % Evaluate Error
    % ==============
    resError(currIndx:currIndx+matchedPtsCount(iNP)-1, :) = round(uvAnch_Proj) - ...
        uvAnch.pixelsRGB;
    currIndx = currIndx+matchedPtsCount(iNP);
    resError(currIndx:currIndx+matchedPtsCount(iNP)-1, :) = round(uvMov_Proj) - ...
        uvMov.pixelsRGB;
    currIndx = currIndx+matchedPtsCount(iNP);
    
end

resError = resError/numPoints;
% disp(sqrt(sum(sum(resError.^2))));
end

%% Helper Functions
function [rtCell, xyzMat] = ExtractRTXYZ(vectRTXYZ, matchedPtsCount, numPairs, ...
    xyzFlag, xyz_Inp)
% This function will only convert the 3D point represented in a long vector into
% Nx3 vector. It will also convert the RPY-XYZ stored in vector into R|T
% matrices.

numPoints = sum(matchedPtsCount);       % Total number of points in all views

% R|T parameters of all views
rtVect = vectRTXYZ(1:numPairs*6);
rtCell = cell(numPairs,2);
currIndx = 1;
for iNP = 1:numPairs
    tmpRPY = rtVect(1, currIndx:currIndx+2);
    rotMat = eul2rotm(tmpRPY);
    tmpLoc = rtVect(1, currIndx+3:currIndx+5)';
    currIndx = currIndx + 6;            % Go to next 6 parameters
    rtCell(iNP, :) = {rotMat, tmpLoc};  % As per the standards
end

% 3D points
if xyzFlag == true
    xyzMat = reshape(vectRTXYZ(6*numPairs+1:end), [3, 2*numPoints])';
else
    xyzMat = reshape(xyz_Inp, [3, 2*numPoints])';
end
end
