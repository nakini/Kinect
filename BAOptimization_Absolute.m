function [xyzRefined_G, rtRefined_C2G, resnorm, residual, exitflag, output, ...
    lambda,jacobian] = BAOptimization_Absolute(pcPairWise, rtRawCurr2Global, ...
    matchPairWise, camIntrinsic, xyzFlag)
% BRIEF
% =====
% In this function, given the transformation matrices, the 3D points and the
% matching pixel coordinates, I will do the followings:
%   1. convert the rotation matrices to euler angles
%   2. create a vector having the all the variable that need to be optimized,
%   such as 3D points, and camera poses.
%   3. write an objective function
%   4. call "lsqnonlin" optimizer with various optins and update the initial
%   guess
%   5. Return the updated XYZs and R|T parameters.
%
% INPUT(s)
% ========
%   1. xyzRaw_Global (Nx3 double): All the points are given w.r.t global 
%   coordinate frame
%
%   2. rtRawCurr2Global (table): It will have view-ids along with the
%   transfomation matrices which transform the from current view to global view.
%   JUST TO BE CONSISTENT WITH MATLAB -- I expect R' instead of R which I am
%   going to transpose back.
%
%   3. matchPairWise (table): Holds matched pixels, view-ids, etc.
%
%   4. camIntrinsic (3x3 matrix): Camera intrinsic parameters
%   JUST TO BE CONSISTENT WITH MATLAB -- I expect K' instead of K which I am
%   going to transpose back.
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

% Convert R|T to RPY-XYZ. Also, we need the transformation from
% "Global-to-Current" view frame as the 3D points are given in global coordiante
% frame.
numViews = size(rtRawCurr2Global, 1);           % View count
% Extrinsic parameters of the camera appended in a row --- [roll-1, pitch-1,
% yaw-1, x-1, y-1, z-1, roll-2, pitch-2, yaw-2, x-2, y-2, z-2, roll-3 ...]
rtG2C_Init = zeros(1, numViews*6);          % 3 for angles + 3 locations
quatG2C_Init = zeros(1, numViews*7);        % 4 quarternions + 3 locations
currIndxRT = 1;
currIndxQ = 1;
for iNV = 1:numViews                            % Not going to update the 1st H
    % Get the values of R and T and take a transpose to convert from Matlab way
    % of representation into universal standard form.
    R = rtRawCurr2Global.Orientation{iNV}';     % Standard format R
    T = rtRawCurr2Global.Location{iNV}';        % Standard format T
    
    % Take the inverse of the transformation matrix.
    R_Global2Curr = R';
    T_Global2Curr = -R'*T;
    
    % Convert the rotation into RPY and store it along with T in a vector form
    tmpRPY = rotm2eul(R_Global2Curr);
    tmpLoc = T_Global2Curr';
    tmpQ = rotm2quat(R_Global2Curr);
    % TODO: All my derivations are a w.r.t. [x y z w]. However, the standard is
    % [w x y z]. NEED TO FIX IT.
    tmpQ = [tmpQ(2:4), tmpQ(1)];
    rtG2C_Init(1, currIndxRT:currIndxRT+5) = [tmpRPY, tmpLoc];  % RPY & T
    currIndxRT = currIndxRT + 6;
    quatG2C_Init(1, currIndxQ:currIndxQ+6) = [tmpQ, tmpLoc];    % Quat & T
    currIndxQ = currIndxQ + 7;
end

% Create a long vector will all the 3D points -- [x-1, y-1, z-1, x-2, y-2, z-2,
% x-3, y-3, z-3, ...]. Include both Anchor and Moved pcs.
matchPtsCount = matchPairWise.Matched_Points;
numPairs = length(matchPtsCount);
xyzVect_Init = zeros(2*sum(matchPtsCount), 3);        % 2 Pairs of pcs for a pair of images
currIndx = 1;
for iNP = 1:numPairs
    % Anchor pc
    % =========
    anchId = matchPairWise.Anchor_ViewID(iNP);
    anchIndex = rtRawCurr2Global.ViewId == anchId;
    
    % Transform the pc to the global coordinate frame
    R = rtRawCurr2Global.Orientation{anchIndex}';
    T = rtRawCurr2Global.Location{anchIndex}';
    tmpRTCurr2Base = struct('R', R, 'T', T);
    tmpAPC = pcPairWise.Anch_PC{iNP,1};
    tmpAPC_G = TransformPointCloud(tmpAPC, tmpRTCurr2Base); % PC in Global frame
    
    xyzVect_Init(currIndx:currIndx + matchPtsCount(iNP)-1, :) = ...
        tmpAPC_G.Location(pcPairWise.Anch_PtsIndx{iNP,1}, :);
    currIndx = currIndx + matchPtsCount(iNP);
    
    % Moved PC
    % ========
    movId = matchPairWise.Moved_ViewID(iNP);
    movIndex = rtRawCurr2Global.ViewId == movId;
    
    % Transform the Moved pc to the global frame
    R = rtRawCurr2Global.Orientation{movIndex}';
    T = rtRawCurr2Global.Location{movIndex}';
    tmpRTCurr2Base = struct('R', R, 'T', T);
    tmpMPC = pcPairWise.Moved_PC{iNP,1};
    tmpMPC_G = TransformPointCloud(tmpMPC, tmpRTCurr2Base); % PC in Global frame
    
    xyzVect_Init(currIndx: currIndx + matchPtsCount(iNP)-1, :) = ...
        tmpMPC_G.Location(pcPairWise.Moved_PtsIndx{iNP,1},:);
    currIndx = currIndx + matchPtsCount(iNP);
end

xyzVect_Init = reshape(xyzVect_Init', [1, numel(xyzVect_Init)]);

% Number of matched point count per view.
matchedPtsCount = matchPairWise.Matched_Points';

% Optimzation objective function and initial guess parameters
if xyzFlag == true
    initial_guess_RPY = [rtG2C_Init, xyzVect_Init]; % Has both XYZs' and RTs'
    initial_guess_Q = [quatG2C_Init, xyzVect_Init];
else
    % for experiment purpose
    initial_guess_RPY = rtG2C_Init;                 % Only RTs'
    initial_guess_Q = quatG2C_Init;                 % Quaternions and Translations
end
rotFlag = 2;
fun = @(x)ObjectiveFun(x, matchPairWise, camIntrinsic, xyzVect_Init, ...
    xyzFlag, numViews, rotFlag);

% Optimation options:
% Algorithm: trust-region-reflective or levenberg-marquardt
jacobianFlag = 0;
if jacobianFlag == 1
    disp('User defined Jacobian is used');
    options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt', ...
        'Display','iter', 'StepTolerance', 1e-10, 'FunctionTolerance', 1e-10, ...
        'OptimalityTolerance', 1e-10, 'MaxIterations', 3, 'Diagnostics', 'on', ...
        'SpecifyObjectiveGradient',true);
else
    disp('Jacobian is evaluated by optimizer');
    options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt', ...
        'Display','iter', 'StepTolerance', 1e-10, 'FunctionTolerance', 1e-10, ...
        'OptimalityTolerance', 1e-10, 'MaxIterations', 3, 'Diagnostics', 'on');
end
% Optimization
if rotFlag == 1
    [final_vals, resnorm, residual, exitflag, output, lambda,jacobian] = ...
        lsqnonlin(fun, initial_guess_RPY, [], [], options);
    
    % Convert the output vector into R|T table and XYZ matrix.
    [rtRefined_G2C, xyzRefined_G] = ExtractRTXYZ(final_vals, matchedPtsCount, ...
        xyzVect_Init, xyzFlag, numViews);
else
    [final_vals, resnorm, residual, exitflag, output, lambda,jacobian] = ...
        lsqnonlin(fun, initial_guess_Q, [], [], options);
    
    % Convert the output vector into R|T table and XYZ matrix.
    [rtRefined_G2C, xyzRefined_G] = ExtractQuatXYZ(final_vals, matchedPtsCount, ...
        xyzVect_Init, xyzFlag, numViews);
end

% First, take the inverse and then transpose the R|T values so that they will
% match the Matlab standard.
rtRefined_C2G = rtRefined_G2C;
viewId = (1:numViews)';
for iNV = 1:numViews
    R = rtRefined_G2C{iNV, 1};
    T = rtRefined_G2C{iNV, 2};
    rtRefined_C2G{iNV, 1} = R;                  % Equal to (inv(R))'
    rtRefined_C2G{iNV, 2} = (-R'*T)';
end
rtRefined_C2G = table(viewId, rtRefined_C2G(:,1), rtRefined_C2G(:,2), ...
    'VariableNames', {'ViewId', 'Orientation', 'Location'});
end

%% Objective Function
function [resError, J] = ObjectiveFun(x, matchPairWise, camIntrinsic, ...
    xyz_G_Inp, xyzFlag, numViews, RotFlag)
% This is where the error matric will be defined given initial guess for the
% transformation parameters, 3D points and the original pixel coordinates.

% Number of matched point count per view.
matchedPtsCount = matchPairWise.Matched_Points';
if (RotFlag == 1)
    [rtG2C_Init, xyz_G_Init] = ExtractRTXYZ(x, matchedPtsCount, xyz_G_Inp, ...
        xyzFlag,numViews);
else
    [rtG2C_Init, xyz_G_Init] = ExtractQuatXYZ(x, matchedPtsCount, xyz_G_Inp, ...
        xyzFlag,numViews);
end

numUnknowns = length(x);                        % Number of unknown parameters
numPoints = 2*sum(matchedPtsCount);
resError = zeros(2*numPoints,1);
partJ_Pt = zeros(2*numViews*numPoints, numViews*7+3);   % Blockwise elements of Jacobian
currIndx = 1;                                   % Point to pixels from 1st Anchor pc
currIdxXYZ = 1;                                 % For XYZ points
% Find the error for each view pair -- I mean, project the points on both the
% matched images and find the error in the pixels. Do the same for all the view
% pairs.
numPairs = size(matchPairWise, 1);
for iVP = 1:numPairs
    % Total number of matching points in the current view pair and the 3D points
    % visible in the current view-pair
    mtchPairCount = matchedPtsCount(iVP);
    anchViewId = matchPairWise.Anchor_ViewID_Sq(iVP);
    movedViewId = matchPairWise.Moved_ViewID_Sq(iVP);
    viewIdx = [anchViewId, movedViewId];
    
    % Project Anchor PC
    % =================
    % Project the 3D points of Anchor pc in the global coordiante into the MOVED
    % view
    xyzAnch_inGlobal = xyz_G_Init(currIdxXYZ:currIdxXYZ+mtchPairCount-1, :);
%     % TODO: NEED TO CHECK WHY I USED movedViewId INSTEAD OF anchorViewID.
    tformMoved = struct('R', rtG2C_Init{movedViewId,1}, 'T', rtG2C_Init{movedViewId,2});
%     tformMoved = struct('R', rtG2C_Init{anchViewId,1}, 'T', rtG2C_Init{anchViewId,2});
    xyzAnch_inCurrView = TransformPointCloud(pointCloud(xyzAnch_inGlobal), tformMoved);
    uvAnch = ProjectPointsOnImage(xyzAnch_inCurrView.Location, camIntrinsic);
    % Residual error from Anchor PC and Moved image --
    %   tmpError = [u1^2, v1^2; u2^2, v2^2; ...]            % nx2
    %   resError = [u1^2; v1^2; u2^2; v2^2; ...]            % 2*nx1
    tmpError = (matchPairWise.PtsPxls_Moved{iVP}.pixelsRGB - uvAnch).^2; % 2xn matrix
    resError(currIndx:2:currIndx+2*mtchPairCount-1, :) = tmpError(:, 1);
    resError(currIndx+1:2:currIndx+2*mtchPairCount-1, :) = tmpError(:, 2);
    
    currIndx = currIndx+2*mtchPairCount;            % Point to pixels in moved pc
    currIdxXYZ = currIdxXYZ+mtchPairCount;          % Point to next XYZ points
    
    J_Anch = JacobianError(tmpErrorVect, movedViewId, numUnknowns, tformMoved, ...
        camIntrinsic, xyzAnch_inGlobal);
    
    % Project Moved PC
    % ================
    % Project the 3D points of Moved point clouds in the global coordiante into
    % the ANCHOR view
    xyzMoved_inGlobal = xyz_G_Init(currIdxXYZ:currIdxXYZ+mtchPairCount-1, :);
%     % TODO: NEED TO CHECK WHY I USED anchorViewID INSTEAD OF movedViewId.
    tformAnch = struct('R', rtG2C_Init{anchViewId,1}, 'T', rtG2C_Init{anchViewId,2});
%     tformAnch = struct('R', rtG2C_Init{movedViewId,1}, 'T', rtG2C_Init{movedViewId,2});
    
    xyzMoved_inCurrView = TransformPointCloud(pointCloud(xyzMoved_inGlobal), tformAnch);
    uvMoved = ProjectPointsOnImage(xyzMoved_inCurrView.Location, camIntrinsic);
    % Residual error from Moved PC and Anchor image --
    tmpError = (matchPairWise.PtsPxls_Anch{iVP}.pixelsRGB - uvMoved).^2; % 2xn matrix
    resError(currIndx:2:currIndx+2*mtchPairCount-1, :) = tmpError(:, 1);
    resError(currIndx+1:2:currIndx+2*mtchPairCount-1, :) = tmpError(:, 2);
    J_Moved = JacobianError(tmpErrorVect, anchViewId, numUnknowns, tformAnch, ...
        camIntrinsic, xyzMoved_inGlobal);
    
    % Jacobian
    % ========
%     J_OnePair = Jacobian(tformAnch, tformMoved, camIntrinsic, xyzMoved_inGlobal, ...
%         xyzAnch_inGlobal, viewIdx, numViews);
%     rowIdx = (iVP-1)*2*numViews*2*mtchPairCount+1 : iVP*2*numViews*2*mtchPairCount;
%     partJ_Pt(rowIdx, :) = J_OnePair;
    
    % Point to Anchor region of next pair
    currIndx = currIndx + 2*mtchPairCount;          % Point to next pc
    currIdxXYZ = currIdxXYZ+mtchPairCount;          % Point to next XYZ points
end
resError = resError/numPoints;
resError = sqrt(sum(resError.^2, 2));           % Sum along the rows
% disp(sqrt(sum(resError.^2)));

% % Create a sparse matrix from the given matrices. We need to create a row,
% % column and a value matrix.
% [rH, cH, vH] = find(partJ_Pt(:, 1:numViews*3));
% [r3D, c3D, v3D] = find(partJ_Pt(:,numViews*3+1:numViews*3+3));
% % c3D = reshape([1:numPoints;1:numPoints], [1, 2*numPoints]);
% spMat_R = 2*numViews*numPoints;
% spMat_C = numViews*7 + numPoints*3;
% J = sparse(horzcat(rH', r3D'), horzcat(cH', c3D'), horzcat(vH', v3D'), ...
%     spMat_R, spMat_C);
J = 0;
end

%% Helper Functions

% Extract RT and XYZ
% ~~~~~~~~~~~~~~~~~~
function [rtCell, xyzMat] = ExtractRTXYZ(vectRTXYZ, matchedPtsCount, xyz_G_Inp, ...
    xyzFlag, numViews)
% This function will only convert the 3D point represented in a long vector into
% Nx3 vector. It will also convert the RPY-XYZ stored in vector into R|T
% matrices.

% numViews = length(matchedPtsCount);     % Total number of views
numPoints = sum(matchedPtsCount);       % Total number of points in all views

% R|T parameters of all views
rtVect = vectRTXYZ(1:numViews*6);
rtCell = cell(numViews,2);
rtCell(1, :) = {eul2rotm([0,0,0]), [0,0,0]'};    % 1st view will be the reference
currIndx = 1;
for iNV = 2:numViews
    tmpRPY = rtVect(1, currIndx:currIndx+2);
    rotMat = eul2rotm(tmpRPY);
    tmpLoc = rtVect(1, currIndx+3:currIndx+5)';
    currIndx = currIndx + 6;            % Go to next 6 parameters
    rtCell(iNV, :) = {rotMat, tmpLoc};  % As per the standards
end

% 3D points
if xyzFlag == true
    xyzMat = reshape(vectRTXYZ(6*(numViews-1)+1:end), [3, 2*numPoints])';
else
    xyzMat = reshape(xyz_G_Inp, [3, 2*numPoints])';
end
end

% Extract Quaternion and XYZ
% ~~~~~~~~~~~~~~~~~~~~~~~~~~
function [quatCell, xyzMat] = ExtractQuatXYZ(vectQuatXYZ, matchedPtsCount, ...
    xyz_G_Inp, xyzFlag, numViews)
% This function will only convert the 3D point represented in a long vector into
% Nx3 vector. It will also convert the Quaternion-XYZ stored in vector into R|T
% matrices.

% numViews = length(matchedPtsCount);     % Total number of views
numPoints = sum(matchedPtsCount);       % Total number of points in all views

% R|T parameters of all views
quatVect = vectQuatXYZ(1:numViews*7);
quatCell = cell(numViews,2);
quatCell(1, :) = {eul2rotm([0,0,0]), [0,0,0]'};    % 1st view will be the reference
currIndx = 1;
for iNV = 1:numViews
    tmpQuat = quatVect(1, currIndx:currIndx+3);
    tmpQuat = [tmpQuat(4), tmpQuat(1:3)];
    rotMat = quat2rotm(tmpQuat);
    tmpLoc = quatVect(1, currIndx+4:currIndx+6)';
    currIndx = currIndx + 7;            % Go to next 6 parameters
    quatCell(iNV, :) = {rotMat, tmpLoc};  % As per the standards
end

% 3D points
if xyzFlag == true
    xyzMat = reshape(vectQuatXYZ(7*numViews+1:end), [3, 2*numPoints])';
else
    xyzMat = reshape(xyz_G_Inp, [3, 2*numPoints])';
end

end

% Jacobian
% ~~~~~~~~
function partJ = Jacobian(tform1, tform2, K, pc1, pc2, viewPair, numViews)
% BRIEF
%
% This funcition is going to evaluate the Jacobian matrix of given point could
% and it's transfromation matrix.
%
% INPUT(s):
%   1) tform1, tform2 [Structure]: 
%       a) R(3x3): Rotation matrix
%       b) T(3x1): Translation vector
%   2) K(3x3): Intrinsic matrix
%   3) pc1, pc2 (Nx3): Pair of points in global coordinate frame from two views
%   4) viewPair (1x2): View ids

numPoints = size(pc1, 1);
partJ = zeros(2*numViews*2*numPoints, numViews*7+3);    % Blockwise elements of Jacobian
vid1 = viewPair(1);                                     % View ids
vid2 = viewPair(2);

for pcNum=1:2
    if pcNum==1
        pc = pc1;
        stIdx = 0;
    else
        pc = pc2;
        stIdx = 1;
    end
    
    for ip=1:numPoints
        % Evaluate derivatives w.r.t R, T and X for each point in pc1
        df_by_dX1 = DerivativeOf_f(tform1.T, tform1.R, K, pc(ip, :)', 1);
        df_by_dT1 = -df_by_dX1;
        
        df_by_dX2 = DerivativeOf_f(tform2.T, tform2.R, K, pc(ip, :)', 1);
        df_by_dT2 = -df_by_dX2;
        
        df_by_dq_1 = DerivativeOf_f(tform1.T, tform1.R, K, pc(ip, :)', 3);
        df_by_dq_2 = DerivativeOf_f(tform2.T, tform2.R, K, pc(ip, :)', 3);
        
        % Fill up the matrix
        rowIdx = stIdx*2*numPoints + (ip-1)*2*numViews + (2*vid1-1:2*vid1);
        partJ(rowIdx, 7*vid1-6:7*vid1-3) = df_by_dq_1;
        partJ(rowIdx, 7*vid1-2:7*vid1) = df_by_dT1;
        partJ(rowIdx, 7*numViews+1:7*numViews+3) = df_by_dX1;
        
        rowIdx = stIdx*2*numPoints + (ip-1)*2*numViews + (2*vid2-1:2*vid2);
        partJ(rowIdx, 7*vid2-6:7*vid2-3) = df_by_dq_2;
        partJ(rowIdx, 7*vid2-2:7*vid2) = df_by_dT2;
        partJ(rowIdx, 7*numViews+1:7*numViews+3) = df_by_dX2;
    end
end
end

function J = JacobianError(deltaUV, viewID, unknownVectLen, tform, K, pcXYZ)
% INPUT(s):
%   deltaUV (2*nx1) -- [deltaU1; deltaV1; deltaU2; deltaV2; ...]
%   viewId (scalar) -- The view id w.r.t which all the derivatives are evaluated
%   unknownVectLen (scalar) -- Number of unknows in the given error functions
%   tform (structure) - Hold 3x1 Translation matrix and 3x3 Rotation matrix
%   K (3x3 matrix) -- Camera intrinsic parameters
%   pcXYZ (nx3) -- 3D point cloud
%
% OUTPUT(s):
%   

numErrPts = size(deltaUV, 1);
J = zeros(2*numErrPts, unknownVectLen);

numPoints = size(pc1, 1);
for ip=1:numPoints
    df_by_dT = DerivativeOf_f(tform.T, tform.R, K, pcXYZ(ip, :)', 2);	% wrt. T
    df_by_dq = DerivativeOf_f(tform.T, tform.R, K, pcXYZ(ip, :)', 3);	% wrt. q
    
    % Concatenate the derivative of q and t so that it becomes 2x7 matrix.
    df_by_dH = horzcat(df_by_dq, df_by_dT);
    
    % Update the appropriate spots in Jacobian matrix.
    %   i) The rows will be: 2*ip-1:2*ip
    %   ii) The columns will be: (viewID-1)*7+1:(viewID-1)*7+8
    J(2*ip-1:2*ip, 7*viewID-6:7*viewID+1) = deltaUV(2*ip-1:2*ip, 1) .* df_by_dH;
end
end
















