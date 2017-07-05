%% Create Surface
t = hgtransform;
surf(peaks(40),'Parent',t)
view(-20,30)
axis manual

%% Create Transform
ry_angle = -15*pi/180; 
Ry = makehgtform('yrotate',ry_angle);
t.Matrix = Ry;

%% Translate the Surface and Rotate
Tx1 = makehgtform('translate',[-20 0 0]);
Tx2 = makehgtform('translate',[20 0 0]);
t.Matrix = Tx2*Ry*Tx1;
