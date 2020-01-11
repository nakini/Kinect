function rtRawCurr2Base = ComputeAbsoluteRTs(rtPairWise, matIncidenceWeight, ...
    varargin)
% Here, we are going to read the pair wise transformation matrices which
% transforms the "moved" pc back to the "anchor", and find out absolute
% transformation to the "base" pc by appending subsequent transformation
% matrices. For each pair, the "moved" number will be the "ViewId". And for each
% moved-num, we will find every possible path to reach to the base first and
% then append all the R|T that fall in each path to find the transformation to
% the base from moved-num for each path.
%
% INPUT(s)
% ========
% 1. rtPairWise: Mx5 table, contains the R|T values along with view numbers
%   1) Anchor_Num, Moved_Num -- Number of the anchor and moved image, respectively
%   2) Orientation -- Rotation matrix from moved-to-anchor
%   3) Location -- Translation vector from moved-to-anchor
%   4) Moved_To_Anchor -- Extra info showing "from" numer to "to" number
%
% 2. matIncidenceWeighted: MxM table, holding the weighted incedence matrix of a
% graph, which has a weight assigned in place of successful matched pair and 0
% everywhere.
%   1) Columns -- Number of each image along with "To_" as prefix as Matlab
%   doesn't allow names starting with numbers for "VariableNames".
%   2) Rows -- Number of each image
%
% OUTPUT(s)
% =========
% 1. rtRawCurr2Base: Nx3 table, containing the view-id, R and T
%
% Example(s)
% ==========
%

%-------------------------------------------------------------------------------
%------------------------------- START -----------------------------------------

% Validate input arguments -----------------------------------------------------
p = inputParser;
p.StructExpand = false;             % Accept structure as one element

% Compulsory parameters --
addRequired(p, 'rtPairWise', @validateRTPairWise);
addRequired(p, 'matIncidenceWeight', @(x) istable(x));

% Optional parameters --
addParameter(p, 'dispGraphFlag', 'true', @(x) islogical(x));

p.parse(rtPairWise, matIncidenceWeight, varargin{:});
disp('Given inputs for EstimateTform_Batch() function:');
disp(p.Results);
fprintf('\n');

% Store variables into local variables to save typing :-p
dispGraphFlag = p.Results.dispGraphFlag;

% Algorithm --------------------------------------------------------------------
% Data Arrangement
% ================
% Get all the file/view numbers and the incidence matrix -- We might miss the
% 1st/last view number in the Moved_Num/Achor_Num column of "rtPariWise" data.
% So better to take a union of these two columns.
viewIDs = union(rtPairWise.Anchor_Num, rtPairWise.Moved_Num);
mtxIncidence = table2array(matIncidenceWeight);     % In standard matrix format
graphMtxInc = digraph(tril(mtxIncidence));          % Directional graph
shortestPathsToBase = shortestpathtree(graphMtxInc,'all', 1);

% Display the graph if needed
if dispGraphFlag
    figure(1)
    graphMtxInc.Nodes.Name = matIncidenceWeight.Row;
    plot(graphMtxInc, 'Layout', 'circle', 'EdgeLabel', graphMtxInc.Edges.Weight);
    title('All possible paths');
    
    figure(2)
    plot(shortestPathsToBase, 'Layout', 'circle');
    title('Shortest paths');
end

% The following two will be used to find the index of the the transformation
% matrix from "moved-view" to "anchor-view".
movedNums = rtPairWise.Moved_Num;
anchNums = rtPairWise.Anchor_Num;

% Shortes Paths of All Nodes
% ==========================
% Now, we use the 1st viewIDs entry as the "base" and then go through rest of
% the numbers and find the shortest path to reach the base from the current one.
allPathsCount = size(viewIDs,1)-1;                  % Total number of views
allShortestPaths = cell(allPathsCount, 2);
for iVN = 2:allPathsCount+1
    % Find the shortest paths between the current view and the base-view.
    shortPath_CurrView = shortestpath(shortestPathsToBase, iVN, 1);
    % Store the values
    allShortestPaths{iVN-1, 1} = viewIDs(iVN);
    allShortestPaths{iVN-1, 2} = shortPath_CurrView;
end
allViews_Id = zeros(allPathsCount, 1);
allViews_R = cell(allPathsCount, 1);
allViews_T = cell(allPathsCount, 1);

% Transformation for each Path
% ============================
% For each path find out the transformation matrices to the base -- To estimate
% the transformation, we will append all the tranformation matrices in an order.
% The shortes path algorithms returns the nodes the path from source-to-target.
% Base on the implementation of appending the transformations, make sure the
% order is maintained.
for iView = 1:allPathsCount
    currPathVect = allShortestPaths{iView, 2};
    currPathVect = currPathVect(end:-1:1);  % Append matrcies from base-to-current_view
    
    % Current-path will have multiple hops. So, we need to append all the
    % transformations in an order that comes under the base and current-view.
    tmpRTCurr2Base = struct('R', eye(3,3), 'T', [0, 0, 0]');
    for iCHop = 1:length(currPathVect)-1
        indxAnch = anchNums == viewIDs(currPathVect(iCHop));
        indxMoved = movedNums == viewIDs(currPathVect(iCHop+1));
        indxCurrView = find(indxAnch & indxMoved);
        
        % Keep updating the transformation matrix as we keep appending the
        % intermediate hops.
        rtMoved = struct('R', rtPairWise.Orientation{indxCurrView}', 'T', ...
            rtPairWise.Location{indxCurrView}');
        % Append the current rtMoved to the temporary "tmpRTCurr2Base".
        tmpRTCurr2Base = AppendRTs(tmpRTCurr2Base, rtMoved);
    end
    % Transformation from current-view to the base
    allViews_R{iView, 1} = tmpRTCurr2Base.R';
    allViews_T{iView, 1} = tmpRTCurr2Base.T';
    
    % Last number in the "currPathVect" is the 'current-view-id'
    allViews_Id(iView, 1) = viewIDs(currPathVect(end));
end

% Output
% ======
% Create a table for R|T
rtRawCurr2Base = table(allViews_Id, allViews_R, allViews_T, ...
    'VariableNames', {'ViewId', 'Orientation', 'Location'});
% Also add the base-to-base view-id, orientation, and location.
rtRawCurr2Base = [{viewIDs(1), eye(3,3), zeros(1,3)}; rtRawCurr2Base];
end

%% Input arguments validating functions
function TF = validateRTPairWise(rtInfo)
% Check whether the input is a table or not. If it is a table then check whether
% it has the column names consistent with the required names.
if ~istable(rtInfo)
    error('Provide information in the form of a table');
elseif ~all(ismember(["Anchor_Num", "Moved_Num", "Location", "Orientation"], ...
        rtInfo.Properties.VariableNames))
    error('The table should contain Anchor_Num|Moved_Num|Location|Orientation');
else
    TF = true;
end
end
