function rgbMapped = Depth2ColorMap_v2(rgbRaw)
% In this function, I am going to find out the RGB map(512x424) using the depth
% image(512x424) and the rgb image(1920x1080). For that, I need some parameters
% which I am going to get from online forum for the time being and later on I
% will calculate on my own.
%
% Copied IR and RGB camera parameters from website: 
%   http://answers.opencv.org/question/78429/kinect-one-stereo-calibration-and-
%   overlay-rgb-with-depth-map/
%
% IR: 
%   fx=365.481  fy=365.481 
%   cx=257.346  cy=210.347
%   k1=0.089026 k2=-0.271706    k3=0.0982151 
%   p1=0        p2=365.481
% Color: 
%   fx=1081.37  fy=1081.37 
%   cx=959.5    cy=539.5 
%   shift_d=863 shift_m=52 
%   mx_x3y0=0.000449294     mx_x0y3=1.91656e-05     mx_x2y1=4.82909e-05 
%   mx_x1y2=0.000353673     mx_x2y0=-2.44043e-05    mx_x0y2=-1.19426e-05 
%   mx_x1y1=0.000988431     mx_x1y0=0.642474        mx_x0y1=0.00500649 
%   mx_x0y0=0.142021        my_x3y0=4.42793e-06     my_x0y3=0.000724863
%   my_x2y1=0.000398557     my_x1y2=4.90383e-05     my_x2y0=0.000136024 
%   my_x0y2=0.00107291      my_x1y1=-1.75465e-05    my_x1y0=-0.00554263 
%   my_x0y1=0.641807        my_x0y0=0.0180811
%
% TODO:
%   I need to find out these parameters using libfreenet2 library. The functions
%   which provide these parameters are: (Freenect2Device::getIrCameraParams and 
%   getColorCameraParams)
%
% INPUTs:
%   rgbRaw --> RGB image matrix of size 1920x1080
% OUTPUTs:
%   rgbMapped --> Mapped RGB image matrix of size 512x424

%------------------------------------------------------------------------------
%------------------------------- START ----------------------------------------

% Initilize the parameters and matrices
depth = struct('fx',365.481, 'fy',365.481, 'cx',257.346, 'cy',210.347, ...
    'k1',0.089026, 'k2',-0.271706, 'k3',0.0982151, 'p1',0, 'p2',365.481);
color = struct('fx',1081.37, 'fy',1081.37, 'cx',959.5, 'cy',539.5, ...
    'shift_d',863, 'shift_m',52, 'mx_x3y0',0.000449294, 'mx_x0y3',1.91656e-05, ...
    'mx_x2y1',4.82909e-05, 'mx_x1y2',0.000353673, 'mx_x2y0',-2.44043e-05, ...
    'mx_x0y2',-1.19426e-05, 'mx_x1y1',0.000988431, 'mx_x1y0',0.642474, ...
    'mx_x0y1',0.00500649, 'mx_x0y0',0.142021, 'my_x3y0',4.42793e-06, ...
    'my_x0y3',0.000724863, 'my_x2y1',0.000398557, 'my_x1y2',4.90383e-05, ...
    'my_x2y0',0.000136024, 'my_x0y2',0.00107291, 'my_x1y1',-1.75465e-05, ...
    'my_x1y0',-0.00554263, 'my_x0y1',0.641807, 'my_x0y0',0.0180811);

rgbMapped = zeros(424,512,3);

% Hardcoded in the original SDK
depth_q = 0.01;
color_q = 0.002199;
for y=1:424
    for x=1:512
        % Get the distorted coordinate
        [my, mx] = distort(y, x, depth);
        % Depth to Color mapping
        [ry, rx] = depth_to_color(y, x, depth, color, depth_q, color_q);
        rgbMapped(y, x, :) = rgbRaw(ry, rx, :);
    end
end
end

%% Function to find the distorted position of a pixel.
function [y, x] = distort(my, mx, depth)
dx = (mx - depth.cx) / depth.fx;
dy = (my - depth.cy) / depth.fy;
dx2 = dx * dx;
dy2 = dy * dy;
r2 = dx2 + dy2;
dxdy2 = 2 * dx * dy;
kr = 1 + ((depth.k3 * r2 + depth.k2) * r2 + depth.k1) * r2;
x = depth.fx * (dx * kr + depth.p2 * (r2 + 2 * dx2) + depth.p1 * dxdy2) ...
    + depth.cx;
y = depth.fy * (dy * kr + depth.p1 * (r2 + 2 * dy2) + depth.p2 * dxdy2) ...
    + depth.cy;
end

%% Function to map a depth pixel to the rgb pixel
function [rx, ry]=depth_to_color(my, mx, depth, color, depth_q, color_q)
mx = (mx - depth.cx) * depth_q;
my = (my - depth.cy) * depth_q;

wx = (mx * mx * mx * color.mx_x3y0) + (my * my * my * color.mx_x0y3) ...
    + (mx * mx * my * color.mx_x2y1) + (my * my * mx * color.mx_x1y2)...
    + (mx * mx * color.mx_x2y0) + (my * my * color.mx_x0y2) ...
    + (mx * my * color.mx_x1y1) ...
    + (mx * color.mx_x1y0) + (my * color.mx_x0y1) + (color.mx_x0y0);

wy = (mx * mx * mx * color.my_x3y0) + (my * my * my * color.my_x0y3) ...
    + (mx * mx * my * color.my_x2y1) + (my * my * mx * color.my_x1y2)...
    + (mx * mx * color.my_x2y0) + (my * my * color.my_x0y2) ...
    + (mx * my * color.my_x1y1) ...
    + (mx * color.my_x1y0) + (my * color.my_x0y1) + (color.my_x0y0);

rx = (wx / (color.fx * color_q)) - (color.shift_m / color.shift_d);
ry = (wy / color_q) + color.cy;
end