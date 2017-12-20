dirName = '/Users/tushar/Desktop/Data/20171027/18/';
scanNames = {'1969_12_31-18_02', '1969_12_31-18_21', '1969_12_31-18_34' ...
    '1969_12_31-18_43', '1969_12_31-18_53'};
scanStart = [53, 2268, 3605, 4504, 5521];
scanEnd = [1061, 3040, 4281, 5236, 6577];
for i=1:5
    ConvertRawDepth2Ply([dirName, scanNames{i}, '/', 'SampleImages/'], ...
        2.5, 'v2', scanStart(i), scanEnd(i), 4);
end