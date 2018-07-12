function BatchUIntImg2DoubleImg(dirName, fileNum, imgName)
% This function convertes all the images in a floder. The original images 
% are of type uint16 and the converted images are of type double.
%
% INPUTs:
%   dirName = Directory containing all the IR images
%   fileNum = The image sequence numbers that are to be processed
%   imgName = Name of the image file without sequence numbers
%
% EXAMPLE:
%   dirName = '/home/fovea/tnb88/Documents/ViGIR_Research/PhD/Calibration/Kinect_v2_20170107/IR2';
%   batchUIntImg2DoubleImg(dirName, 1:10, 'ir_11-')


totalFiles = length(fileNum);
for iTF = 1:totalFiles
    fileName = sprintf('%s/%s%d.pgm', dirName, imgName, fileNum(iTF));
    imgUInt16 = imread(fileName);
    imgDouble = im2double(imgUInt16);
    imwrite(imgDouble, fileName)
end
