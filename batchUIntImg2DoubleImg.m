function batchUIntImg2DoubleImg(dirName, fileNum)
% This function convertes all the images in a floder. The original images 
% are of type uint16 and the converted images are of type double.
%
% EXAMPLE:
%   dirName = '/home/fovea/tnb88/Documents/ViGIR_Research/PhD/Calibration/Kinect_v2_20170107/IR2';
%   batchUIntImg2DoubleImg(dirName, 1:10)

totalFiles = length(fileNum);
for iTF = 1:totalFiles
    fileName = sprintf('%s/irImg_%d.pgm', dirName, fileNum(iTF));
    imgUInt16 = imread(fileName);
    imgDouble = im2double(imgUInt16);
    imwrite(imgDouble, fileName)
end