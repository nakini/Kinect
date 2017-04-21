function depthInMeters = RawDepth2Meters_v1(depthValue)
% This function takes the depth data capctured from the kinect and convets them into depth
% in meters. The depth data should be a MxN matrix and the output will also be a MxN
% matrix. Additionally, make sure the depth data is the raw-data, i.e, it has not been
% preprocessed and the values range from 0 through 2047 (i.e., 11 bits).

depthInMeters = 1 ./ (double(depthValue)*(-0.0030711016)+3.3309495161);
