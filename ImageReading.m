
%rosinit
global rgbBuffer;
global depthBuffer;
while 1
getMsg 
end
%% Get RGB and Depth Messages from ROS
function getMsg 
global rgbBuffer;
global depthBuffer;
rgb = rossubscriber('/camera/rgb/image_raw'); %
depth = rossubscriber('/camera/depth/image_raw'); %
img= receive(rgb,10); 
[ima,alpha] = readImage(img);
imshow(ima)
%pause(0.01);
end