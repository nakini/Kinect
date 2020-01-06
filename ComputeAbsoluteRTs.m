function rtRawCurr2Base  = ComputeAbsoluteRTs(rtPairWise, matIncidence, varargin)
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
% 2. matIncidence: MxM table, holding the incedence matrix of a graph, which 
% has "1" in place of successful pair and 0 everywhere.
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
addRequired(p, 'matIncidence', @(x) istable(x));

% Optional parameters --
addParameter(p, 'dispGraphFlag', 'true', @(x) islogical(x));

p.parse(rtPairWise, matIncidence, varargin{:});
disp('Given inputs for EstimateTform_Batch() function:');
disp(p.Results);
fprintf('\n');

% Store variables into local variables to save typing :-p
dispGraphFlag = p.Results.dispGraphFlag;

% Algorithm --------------------------------------------------------------------
% Data Arrangement
% ================
% Get all the file numbers and the incidence matrix -- The file names will be
% cell array, we need to convert them into a numbers.
fileNumbers = str2num(cell2mat(matIncidence.Row));
matInc = table2array(matIncidence);         % In standard matrix format

if dispGraphFlag
    plot(digraph(matInc), 'Layout', 'circle');
end

% The following two will be used to find the index of the the transformation
% matrix from "moved-view" to "anchor-view".
movedNums = rtPairWise.Moved_Num;
anchNums = rtPairWise.Anchor_Num;

% All Possible Paths
% ==================
% Now, we use the 1st fileNumber entry as the "base" and then go through rest of
% the numbers and find every possible way to reach the base from the current
% one.
allPaths_AllViews = cell(length(fileNumbers)-1, 3);
allPathsCount = 0;
for iFN = 2:length(fileNumbers)
    % Find all possible paths between the current view and the base-view. The
    % function below will return a cell array with each member representing a
    % path.
    allPaths_CurrView = pathbetweennodes(matInc, iFN, 1);   % src = iFN; dest = 1;
    
    % Store the values
    allPaths_AllViews{iFN-1, 1} = fileNumbers(iFN);         % View number
    allPaths_AllViews{iFN-1, 2} = allPaths_CurrView;        % All possible paths for current view
    allPaths_AllViews{iFN-1, 3} = length(allPaths_CurrView);% Paths count for current view
    
    % Update the all paths count in the Graph.
    allPathsCount = allPathsCount + length(allPaths_CurrView);
end

allViewIds = zeros(allPathsCount, 1);
allPairs_R = cell(allPathsCount, 1);
allPairs_T = cell(allPathsCount, 1);

% Transformation for each Path
% ============================
% For each path find out the transformation matrices to the base -- To estimate
% the transformation, we will append all the tranformation matrices in an order.
indxAllVids = 1;
for iView = 1:length(allPaths_AllViews)
    allPaths_CurrView = allPaths_AllViews{iView, 2};
    
    % In each path, we start from the begining node which is the current
    % view-number and keep appending subsequent node until we reach the end
    % which will be the base view number.
    for iCPath = 1:length(allPaths_CurrView)
        currPathVect = allPaths_CurrView{iCPath};
        
        % Current-path will have multiple hops. So, we need to append all the
        % transformations in an order that comes under the current-view and base
        % view.
        tmpRTCurr2Base = struct('R', eye(3,3), 'T', [0, 0, 0]');
        for iCHop = 1:length(currPathVect)-1
            indxAnch = anchNums == fileNumbers(currPathVect(iCHop+1));
            indxMoved = movedNums == fileNumbers(currPathVect(iCHop));
            
            indxCurrView = indxAnch & indxMoved;
            % Keep updating the transformation matrix as we keep appending the
            % intermediate hops.
            rtMoved = struct('R', rtPairWise.Orientation{indxCurrView}', 'T', ...
                rtPairWise.Location{indxCurrView}');
            tmpRTCurr2Base = AppendRTs(tmpRTCurr2Base, rtMoved);
        end
        
        % Transformation from current-view to the base
        allPairs_R{indxAllVids, 1} = tmpRTCurr2Base.R';
        allPairs_T{indxAllVids, 1} = tmpRTCurr2Base.T';
        
        % 1st number is the 'current-view'
        allViewIds(indxAllVids, 1) = fileNumbers(currPathVect(1));
            
        % Go to the next index
        indxAllVids = indxAllVids + 1;
    end
end

% Output
% ======
% Create a table for R|T
rtRawCurr2Base = table(allViewIds, allPairs_R, allPairs_T, ...
    'VariableNames', {'ViewId', 'Orientation', 'Location'});
% Also add the base-to-base view-id, orientation, and location.
rtRawCurr2Base = [{fileNumbers(1), eye(3,3), zeros(1,3)}; rtRawCurr2Base];
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