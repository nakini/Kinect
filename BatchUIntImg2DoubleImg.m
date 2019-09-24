function BatchUIntImg2DoubleImg(dirName, fileNum, imgName, hist_eq_flag)
% This function converts all the images in a folder. The original images 
% are of type uint16 and the converted images are of type double.
%
% INPUTs:
%   dirName = Directory containing all the IR images
%   fileNum = The image sequence numbers that are to be processed
%   imgName = Name of the image file without sequence numbers
%   hist_eq_flag = if set all the images will be passed through histogram
%           equalization process (default is false)
%
% EXAMPLE:
%   dirName = ['/home/fovea/tnb88/Documents/ViGIR_Research/PhD/Calibration/',...
%           'Kinect_v2_20170107/IR2'];
%   batchUIntImg2DoubleImg(dirName, 1:10, 'ir_11-', false)

% Do a error check for the arguments.
if nargin < 4
    hist_eq_flag = 'false';
    if nargin < 3 || nargin > 4
        error('Must provide 1st 3 arguments');
    end
end

totalFiles = length(fileNum);
for iTF = 1:totalFiles
    fileName = sprintf('%s/%s%d.pgm', dirName, imgName, fileNum(iTF));
    imgUInt16 = imread(fileName);
    imgDouble = im2double(imgUInt16);
    if strcmp(hist_eq_flag, 'true')
        imgDouble = histeq(imgDouble);
    end
    imwrite(imgDouble, fileName)
end
