
clear all
close all
set(0,'DefaultFigureWindowStyle','docked')
rosshutdown
rosinit
warning('off')
% while 1
% getMsg
% pause(0.01);
% end
% %% Get RGB and Depth Messages from ROS
%
% function getMsg
clear('pub','sub','node')
node = ros.Node('/driving');
qrCode = rgb2gray(imread('FollowMeQr.png'));
rgb = rossubscriber('/camera/rgb/image_raw'); %
depth = rossubscriber('/camera/depth/image_raw'); %
odom = rossubscriber('/odom');
drive = ros.Publisher(node,'/cmd_vel','geometry_msgs/Twist');
msg = rosmessage('geometry_msgs/Twist');
AnVel = pi/12;
TranVel = 0.15;
ToleTime = 1.8;%each computer has different toletime for Tim's Computer is 0.5s
kp_An = 1;
kd_An = 5;
ki_An = 0.05;
kp_Li = 0.01;
kd_Li = 0.1;
ki_Li = 0.0003;%0.00009;
PID_i=0;
angle_previous_error = 0;
%distance_previous_error = 0;
msg.Linear.X = TranVel;%0.5
msg.Angular.Z = AnVel;%pi/18 pi/12
trvec1 = [0.064 -0.065 0.104];%base footprint to camera link
tform1 = trvec2tform(trvec1);
trvec2 = [0.005 0.018 0.013];%camera link to camera rgb
tform2 = trvec2tform(trvec2);
Quat = quaternion([0.500398163355 -0.499999841466 0.499601836645 -0.499999841466]);%Constant
rotmRF2OF = rotmat(Quat,'point');%camera rgb to rgb optical frame
tform3 = rotm2tform(rotmRF2OF);
TF = tform1*tform2*tform3;%basefootprint to RGB optical frame
K = [1206.8897719532354 0.0 960.5; 0.0 1206.8897719532354 540.5; 0.0 0.0 1.0 ];
%%
while 1
disp('Start navigation... ')
msgOdo = receive(odom,10);
qaWbot = msgOdo.Pose.Pose.Orientation.W;
qaXbot = msgOdo.Pose.Pose.Orientation.X;
qaYbot = msgOdo.Pose.Pose.Orientation.Y;
qaZbot = msgOdo.Pose.Pose.Orientation.Z;
Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
rotmXYZ = rotmat(Quatbot,'point');
trform2 = rotm2tform(rotmXYZ);
Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
trform = trvec2tform(Tranbot);
TFmean2world = trform*trform2; %%%CHANGE EVERTIME ROBOT MOVE

img= receive(rgb,10);
dept= receive(depth,10);
[rgbImage,~] = readImage(img);
[depthImage,~] = readImage(dept);
cam = rgb2gray(rgbImage);
ptsQrCode = detectSURFFeatures(qrCode);
ptsCam = detectSURFFeatures(cam);
[featureQr,validPtsQr]=extractFeatures(qrCode,ptsQrCode);
[featureCam,validPtsCam]=extractFeatures(cam,ptsCam);

indexPairs = matchFeatures(featureQr,featureCam);%include outliers as well
matchedQr = validPtsQr(indexPairs(:,1));
matchedCam = validPtsCam(indexPairs(:,2));

msg.Linear.X = 0;

%%
%     img= receive(rgb,10);
%     dept= receive(depth,10);
%     [rgbImage,~] = readImage(img);
%     [depthImage,~] = readImage(dept);
%     cam = rgb2gray(rgbImage);
%     ptsQrCode = detectSURFFeatures(qrCode);
%     ptsCam = detectSURFFeatures(cam);
%     [featureQr,validPtsQr]=extractFeatures(qrCode,ptsQrCode);
%     [featureCam,validPtsCam]=extractFeatures(cam,ptsCam);
%
%     indexPairs = matchFeatures(featureQr,featureCam);
%     matchedQr = validPtsQr(indexPairs(:,1));
%     matchedCam = validPtsCam(indexPairs(:,2));
while size(indexPairs,1) < 4
    tic
    msg.Angular.Z = pi/3;
    while(1)
        send(drive,msg);
        if toc >= (0.5) %(9 seconds ---> 8.236375 or .236373 or =9-0.763627)
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        end
    end
    img= receive(rgb,10);
    dept= receive(depth,10);
    [rgbImage,~] = readImage(img);
    [depthImage,~] = readImage(dept);
    cam = rgb2gray(rgbImage);
    ptsQrCode = detectSURFFeatures(qrCode);
    ptsCam = detectSURFFeatures(cam);
    [featureQr,validPtsQr]=extractFeatures(qrCode,ptsQrCode);
    [featureCam,validPtsCam]=extractFeatures(cam,ptsCam);
    
    indexPairs = matchFeatures(featureQr,featureCam);
    matchedQr = validPtsQr(indexPairs(:,1));
    matchedCam = validPtsCam(indexPairs(:,2));
    
    if size(indexPairs,1) >= 4
        break;
    end
end
[~,inlierCam,inlierQr] = estimateGeometricTransform(matchedCam,matchedQr,'similarity');


showMatchedFeatures(qrCode,cam,inlierQr,inlierCam);
depthCam = nan(inlierCam.Count,1);
CP = nan(3,inlierCam.Count);
locsCamFrame = nan(3,inlierCam.Count);
locsBaseFrame = nan(4,inlierCam.Count);

for i=1 : inlierCam.Count
    depthCam(i) = depthImage(round(inlierCam.Location(i,2)),round(inlierCam.Location(i,1)));
    CP(:,i) = [(depthCam(i)* inlierCam.Location(i,1)) ; (depthCam(i)* inlierCam.Location(i,2)) ; depthCam(i)];
    locsCamFrame(:,i) = K\CP(:,i);
    locsBaseFrame(:,i) =  TF*[locsCamFrame(:,i);1];
end
disp('No. Point: ')
size(locsBaseFrame,2)% can not make a plane with 2 pts
%%
while size(locsBaseFrame,2) < 3
    tic
    msg.Angular.Z = pi/40;
    while(1)
        send(drive,msg);
        if toc >= (1) %(9 seconds ---> 8.236375 or .236373 or =9-0.763627)
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        end
    end
    img= receive(rgb,10);
    dept= receive(depth,10);
    [rgbImage,~] = readImage(img);
    [depthImage,~] = readImage(dept);
    cam = rgb2gray(rgbImage);
    ptsQrCode = detectSURFFeatures(qrCode);
    ptsCam = detectSURFFeatures(cam);
    [featureQr,validPtsQr]=extractFeatures(qrCode,ptsQrCode);
    [featureCam,validPtsCam]=extractFeatures(cam,ptsCam);
    indexPairs = matchFeatures(featureQr,featureCam);
    matchedQr = validPtsQr(indexPairs(:,1));
    matchedCam = validPtsCam(indexPairs(:,2));
    while size(indexPairs,1) < 4
        tic
        msg.Angular.Z = pi/40;
        while(1)
            send(drive,msg);
            if toc >= (1) %(9 seconds ---> 8.236375 or .236373 or =9-0.763627)
                msg.Angular.Z = 0;
                send(drive,msg);
                break;
            end
        end
        img= receive(rgb,10);
        dept= receive(depth,10);
        [rgbImage,~] = readImage(img);
        [depthImage,~] = readImage(dept);
        cam = rgb2gray(rgbImage);
        ptsQrCode = detectSURFFeatures(qrCode);
        ptsCam = detectSURFFeatures(cam);
        [featureQr,validPtsQr]=extractFeatures(qrCode,ptsQrCode);
        [featureCam,validPtsCam]=extractFeatures(cam,ptsCam);
        
        indexPairs = matchFeatures(featureQr,featureCam);
        matchedQr = validPtsQr(indexPairs(:,1));
        matchedCam = validPtsCam(indexPairs(:,2));
        
        if size(indexPairs,1) >= 4
            break;
        end
    end
    [~,inlierCam,inlierQr] = estimateGeometricTransform(matchedCam,matchedQr,'similarity');
    showMatchedFeatures(qrCode,cam,inlierQr,inlierCam);
    depthCam = nan(inlierCam.Count,1);
    CP = nan(3,inlierCam.Count);
    locsCamFrame = nan(3,inlierCam.Count);
    locsBaseFrame = nan(4,inlierCam.Count);
    
    for i=1 : inlierCam.Count
        depthCam(i) = depthImage(round(inlierCam.Location(i,2)),round(inlierCam.Location(i,1)));
        CP(:,i) = [(depthCam(i)* inlierCam.Location(i,1)) ; (depthCam(i)* inlierCam.Location(i,2)) ; depthCam(i)];
        locsCamFrame(:,i) = K\CP(:,i);
        locsBaseFrame(:,i) =  TF*[locsCamFrame(:,i);1];
    end
    if size(locsBaseFrame,2) >= 3
        break;
    end
end
meanLoc = mean(locsBaseFrame,2);
msgOdo = receive(odom,10);
qaWbot = msgOdo.Pose.Pose.Orientation.W;
qaXbot = msgOdo.Pose.Pose.Orientation.X;
qaYbot = msgOdo.Pose.Pose.Orientation.Y;
qaZbot = msgOdo.Pose.Pose.Orientation.Z;
Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
rotmXYZ = rotmat(Quatbot,'point');
trform2 = rotm2tform(rotmXYZ);
Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
trform = trvec2tform(Tranbot);
TFmean2world = trform*trform2; %%%CHANGE EVERTIME ROBOT MOVE
mean2world = TFmean2world*meanLoc;
%% Finding plane
ptCloud = pointCloud(locsBaseFrame(1:3,:)');
plane = pcfitplane(ptCloud,1);
planeN = TFmean2world*[(plane.Normal)';1];
planeInfo = [plane.Normal(1) plane.Normal(2) meanLoc(1) meanLoc(2)];
robotInfo = [-plane.Normal(2) plane.Normal(1) 0 0];

try
    %% Align with intersectionpoint
    IntersectionPoint = TwoDSolver(planeInfo,robotInfo,1);
    IntersectionPoint = IntersectionPoint.IntersectPoint();
    IntersectionPointW = TFmean2world*[IntersectionPoint.xIn;IntersectionPoint.yIn;0;1];
    IntersectionPointW = IntersectionPointW(1:3);
    VecTar =  IntersectionPointW' - Tranbot;
    Rot = Rotation(Quatbot,VecTar,AnVel);
    Rot = Rot.TimeDeter();
    msg.Linear.X = 0;
    % send(drive,msg);
    while (1)
        msg.Linear.X = 0;
        msgOdo = receive(odom,10);
        qaWbot = msgOdo.Pose.Pose.Orientation.W;
        qaXbot = msgOdo.Pose.Pose.Orientation.X;
        qaYbot = msgOdo.Pose.Pose.Orientation.Y;
        qaZbot = msgOdo.Pose.Pose.Orientation.Z;
        Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
        Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
        VecTar =   IntersectionPointW' - Tranbot;
        Rot = Rotation(Quatbot,VecTar,pi/12);
        Rot = Rot.TimeDeter();
        angle_error = Rot.ThetainRad;
        if abs(angle_error) < 0.05
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        end
        PID_p = kp_An*angle_error;
        angle_previous_error = 0;
        PID_d = kd_An*((angle_error - angle_previous_error)/20);
        if(-0.1 < angle_error && angle_error < 0.1)
            PID_i = PID_i + (ki_An * angle_error);
        else
            PID_i = 0;
        end
        PID_total = PID_p + PID_i ;%+ PID_d;
        msg.Linear.X = 0;
        msg.Angular.Z = PID_total;
        send(drive,msg);
        angle_previous_error = angle_error;
    end
  
    
    %% Start going straight to InterSection Point
    PID_i = 0;
    PID_p = 0;
    TransDistance = sqrt((Tranbot(1)-IntersectionPointW(1))^2 + (Tranbot(2)-IntersectionPointW(2))^2);
    distance_error =  sqrt((Tranbot(1)-IntersectionPointW(1))^2 + (Tranbot(2)-IntersectionPointW(2))^2);
    distance_error_original = distance_error;
    if TransDistance < 1
        %small distance
        kd_Li = 0.1;
        kp_Li = 0.1;
        ki_Li = 0.0009;%0.0003;%0.00009;
        Tole = 0.2;
    else
        kd_Li = 0.1;
        kp_Li = 0.1/distance_error;%0.005
        ki_Li = 0.0001;%0.00003
        Tole = 0.2;
    end
         
    if IntersectionPointW(1)-Tranbot(1) > 0
        sign = 1;
    end
    
    
    if IntersectionPointW(1)-Tranbot(1) < 0
        sign = -1;
    end

    while (1)
        msgOdo = receive(odom,10);
        Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];
        distance_error =  sqrt((Tranbot(1)-IntersectionPointW(1))^2 + (Tranbot(2)-IntersectionPointW(2))^2);
        
        try
            if sign == 1
                if IntersectionPointW(1)-Tranbot(1) < 0
                    msg.Linear.X = 0;
                    msg.Angular.Z = 0;
                    send(drive,msg);
                    break;
                end
            end
            if sign == -1
                if IntersectionPointW(1)-Tranbot(1) > 0
                    msg.Linear.X = 0;
                    msg.Angular.Z = 0;
                    send(drive,msg);
                    break;
                end
            end
        end
        if (abs(distance_error) <= 0.08)
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        elseif ((abs(Tranbot(1)-IntersectionPointW(1)) < 0.08) && (abs(Tranbot(2)-IntersectionPointW(2)) < 0.08))
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        end
        PID_p = kp_Li*distance_error;
        %         PID_d = kd_Li*((distance_error - distance_previous_error)/20);
        if(abs(distance_error) > distance_error_original/2)
            PID_i = PID_i + (ki_Li * distance_error);
            
        else
            PID_i = PID_i - (ki_Li * distance_error);
            if PID_i < 0
                PID_i = 0;
            end
        end
        PID_total = PID_p + PID_i;
        msg.Linear.X = PID_total;
        send(drive,msg);
        distance_previous_error = distance_error;
    end
    
    %% Rotation from Intersection point to goal
    PID_i = 0;
    PID_d = 0;
    PID_p = 0;
    msg.Linear.X = 0;
    msgOdo = receive(odom,10);
    qaWbot = msgOdo.Pose.Pose.Orientation.W;
    qaXbot = msgOdo.Pose.Pose.Orientation.X;
    qaYbot = msgOdo.Pose.Pose.Orientation.Y;
    qaZbot = msgOdo.Pose.Pose.Orientation.Z;
    Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
    Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
    VecTar =  mean2world(1:3)' - Tranbot;
    Rot = Rotation(Quatbot,VecTar,AnVel);
    Rot = Rot.TimeDeter();
    msg.Linear.X = 0;
    while (1)
        msg.Linear.X = 0;
        msgOdo = receive(odom,10);
        qaWbot = msgOdo.Pose.Pose.Orientation.W;
        qaXbot = msgOdo.Pose.Pose.Orientation.X;
        qaYbot = msgOdo.Pose.Pose.Orientation.Y;
        qaZbot = msgOdo.Pose.Pose.Orientation.Z;
        Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
        Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
        VecTar =  mean2world(1:3)' - Tranbot;
        Rot = Rotation(Quatbot,VecTar,pi/12);
        Rot = Rot.TimeDeter();
        angle_error = Rot.ThetainRad;
        if abs(angle_error) < 0.05
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        end
        PID_p = kp_An*angle_error;
        PID_d = kd_An*((angle_error - angle_previous_error)/20);
        if(-0.1 < angle_error && angle_error < 0.1)
            PID_i = PID_i + (ki_An * angle_error);
        else
            PID_i = 0;
        end
        PID_total = PID_p + PID_i + PID_d;
        msg.Linear.X = 0;
        msg.Angular.Z = PID_total;
        send(drive,msg)
        angle_previous_error = angle_error;
    end
    
    
    %% Scan again
    
    img= receive(rgb,10);
    dept= receive(depth,10);
    [rgbImage,~] = readImage(img);
    [depthImage,~] = readImage(dept);
    cam = rgb2gray(rgbImage);
    msgOdo = receive(odom,10);
    qaWbot = msgOdo.Pose.Pose.Orientation.W;
    qaXbot = msgOdo.Pose.Pose.Orientation.X;
    qaYbot = msgOdo.Pose.Pose.Orientation.Y;
    qaZbot = msgOdo.Pose.Pose.Orientation.Z;
    Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
    rotmXYZ = rotmat(Quatbot,'point');
    trform2 = rotm2tform(rotmXYZ);
    Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
    trform = trvec2tform(Tranbot);
    TFmean2world = trform*trform2;
    
    ptsQrCode = detectSURFFeatures(qrCode);
    ptsCam = detectSURFFeatures(cam);
    [featureQr,validPtsQr]=extractFeatures(qrCode,ptsQrCode);
    [featureCam,validPtsCam]=extractFeatures(cam,ptsCam);
    
    indexPairs = matchFeatures(featureQr,featureCam);
    matchedQr = validPtsQr(indexPairs(:,1));
    matchedCam = validPtsCam(indexPairs(:,2));
    
    [~,inlierCam,inlierQr] = estimateGeometricTransform(matchedCam,matchedQr,'similarity');
    
    showMatchedFeatures(qrCode,cam,inlierQr,inlierCam);
    
    depthCam = nan(inlierCam.Count,1);
    CP = nan(3,inlierCam.Count);
    locsCamFrame = nan(3,inlierCam.Count);
    locsBaseFrame = nan(4,inlierCam.Count);
    
    for i=1 : inlierCam.Count
        depthCam(i) = depthImage(round(inlierCam.Location(i,2)),round(inlierCam.Location(i,1)));
        CP(:,i) = [(depthCam(i)* inlierCam.Location(i,1)) ; (depthCam(i)* inlierCam.Location(i,2)) ; depthCam(i)];
        locsCamFrame(:,i) = K\CP(:,i);
        locsBaseFrame(:,i) =  TF*[locsCamFrame(:,i);1];
    end
    disp('No. Point: ')
    size(locsBaseFrame,2)
    meanLoc = mean(locsBaseFrame,2);
    %meanLoc = [meanLoc;1];
    mean2world = TFmean2world*meanLoc
    %% Finding plane
    ptCloud = pointCloud(locsBaseFrame(1:3,:)');
    plane = pcfitplane(ptCloud,1);
    planeN = TFmean2world*[(plane.Normal)';1]
    planeInfo = [plane.Normal(1) plane.Normal(2) meanLoc(1) meanLoc(2)];
    robotInfo = [-plane.Normal(2) plane.Normal(1) 0 0];
    
    msgOdo = receive(odom,10);
    qaWbot = msgOdo.Pose.Pose.Orientation.W;
    qaXbot = msgOdo.Pose.Pose.Orientation.X;
    qaYbot = msgOdo.Pose.Pose.Orientation.Y;
    qaZbot = msgOdo.Pose.Pose.Orientation.Z;
    Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
    Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
    tempAnVel=pi/90;
    VecTar =  mean2world(1:3)' - Tranbot;
    eulXYZ = quat2eul(Quatbot,'XYZ');
    Vecbot = [cos(eulXYZ(1,3)) sin(eulXYZ(1,3)) 0];
    
    %% Check offset
    PID_i = 0;
    PID_d = 0;
    PID_p = 0;
    u = [Vecbot(1) Vecbot(2)];
    v = [VecTar(1) VecTar(2)];
    
    
    CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
    ThetaInDegrees = real(acosd(CosTheta))
    Rot = Rotation(Quatbot,VecTar,AnVel);
    Rot = Rot.TimeDeter();
    msg.Linear.X = 0;
    % send(drive,msg);
    while (1)
        msg.Linear.X = 0;
        msgOdo = receive(odom,10);
        qaWbot = msgOdo.Pose.Pose.Orientation.W;
        qaXbot = msgOdo.Pose.Pose.Orientation.X;
        qaYbot = msgOdo.Pose.Pose.Orientation.Y;
        qaZbot = msgOdo.Pose.Pose.Orientation.Z;
        Quatbot = quaternion([qaWbot qaXbot qaYbot qaZbot]);
        Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];%current position world
        VecTar =  mean2world(1:3)' - Tranbot;
        Rot = Rotation(Quatbot,VecTar,pi/12);
        Rot = Rot.TimeDeter();
        angle_error = Rot.ThetainRad;
        if abs(angle_error) < 0.001
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        end
        PID_p = kp_An*angle_error;
        angle_previous_error = 0;
        PID_d = kd_An*((angle_error - angle_previous_error)/20);
        if(-0.1 < angle_error && angle_error < 0.1)
            PID_i = PID_i + (ki_An * angle_error);
        else
            PID_i = 0;
        end
        PID_total = PID_p + PID_i ;%+ PID_d;
        msg.Linear.X = 0;
        msg.Angular.Z = PID_total;
        send(drive,msg)
        angle_previous_error = angle_error;
    end
    
    
    %% Going Straight to Goal point
    PID_i = 0;
    PID_d = 0;
    PID_p = 0;
    msg.Angular.Z = 0;
    msgOdo = receive(odom,10);
    Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];
    TransDistance = sqrt((Tranbot(1)-mean2world(1))^2 + (Tranbot(2)-mean2world(2))^2);
    distance_error =  sqrt((Tranbot(1)-mean2world(1))^2 + (Tranbot(2)-mean2world(2))^2);
    distance_error_original = distance_error;
    
    if TransDistance < 1
        %small distance
        kd_Li = 0.1;
        kp_Li = 0.1;
        ki_Li = 0.0009;%0.0003;%0.00009;
        Tole = 0.2;
    else
        kd_Li = 0.1;
        kp_Li = 0.1/distance_error;%0.005
        ki_Li = 0.0001;%0.00003
        Tole = 0.2;
    end
    
    if mean2world(1)-Tranbot(1) > 0
        sign = 1;
    end
    
    
    if mean2world(1)-Tranbot(1) < 0
        sign = -1;
    end
    while (1)
        msgOdo = receive(odom,10);
        Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];
        distance_error =  sqrt((Tranbot(1)-mean2world(1))^2 + (Tranbot(2)-mean2world(2))^2);
        try
            if sign == 1
                if mean2world(1)-Tranbot(1) < 0
                    msg.Linear.X = 0;
                    msg.Angular.Z = 0;
                    send(drive,msg);
                    break;
                end
            end
            if sign == -1
                if mean2world(1)-Tranbot(1) > 0
                    msg.Linear.X = 0;
                    msg.Angular.Z = 0;
                    send(drive,msg);
                    break;
                end
            end
        end
        if (abs(distance_error) <= 0.2)
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        elseif ((abs(Tranbot(1)-mean2world(1)) < 0.2) && (abs(Tranbot(2)-mean2world(2)) < 0.2))
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(drive,msg);
            break;
        end
        PID_p = kp_Li*distance_error;
        %         PID_d = kd_Li*((distance_error - distance_previous_error)/20);
        if(abs(distance_error) > distance_error_original/2)
            PID_i = PID_i + (ki_Li * distance_error);
            
        else
            PID_i = PID_i - (ki_Li * distance_error);
            if PID_i < 0
                PID_i = 0;
            end
        end
        PID_total = PID_p + PID_i;%+ PID_d;
        msg.Linear.X = PID_total;
        msg.Angular.Z = 0;
        send(drive,msg)
        distance_previous_error = distance_error;
    end
    %         TransDistance = sqrt((Tranbot(1)-IntersectionPointW(1))^2 + (Tranbot(2)-IntersectionPointW(2))^2);
    %     if(IntersectionPointW(1)- Tranbot(1)) > 0
    %         sign = 1;
    %     else
    %         sign = -1;
    %     end
    %     distance_error =  sqrt((Tranbot(1)-IntersectionPointW(1))^2 + (Tranbot(2)-IntersectionPointW(2))^2);
    %     distance_error_original = distance_error;
    %     if TransDistance < 1
    %         %small distance
    %         kp_Li = 0.1;
    %         ki_Li = 0.0009;%0.0003;%0.00009;
    %         Tole = 0.2;
    %     else
    %         kp_Li = 0.01/distance_error;%0.005
    %         ki_Li = 0.00001;%0.00003
    %         Tole = 0.2;
    %     end
    %     while (1)
    %         msgOdo = receive(odom,10);
    %         Tranbot = [msgOdo.Pose.Pose.Position.X msgOdo.Pose.Pose.Position.Y msgOdo.Pose.Pose.Position.Z];
    %         distance_error =  sqrt((Tranbot(1)-IntersectionPointW(1))^2 + (Tranbot(2)-IntersectionPointW(2))^2)
    %         if sign == 1
    %             if Tranbot(1) > IntersectionPointW(1)
    %                 distance_error = -distance_error;
    %             end
    %         end
    %         if sign == -1
    %             if Tranbot(1)-IntersectionPointW(1) < 0
    %                 distance_error = -distance_error;
    %             end
    %         end
    %         if (abs(distance_error) <= 0.1)
    %             msg.Linear.X = 0;
    %             msg.Angular.Z = 0;
    %             send(drive,msg);
    %             break;
    %         elseif ((abs(Tranbot(1)-IntersectionPointW(1)) < 0.1) && (abs(Tranbot(2)-IntersectionPointW(2)) < 0.1))
    %             msg.Linear.X = 0;
    %             msg.Angular.Z = 0;
    %             send(drive,msg);
    %             break;
    %         end
    %         PID_p = kp_Li*distance_error;
    % %         PID_d = kd_Li*((distance_error - distance_previous_error)/20);
    %         if(abs(distance_error) > distance_error_original/2)
    %             PID_i = PID_i + (ki_Li * distance_error);
    %
    %         else
    %             PID_i = PID_i - (ki_Li * distance_error);
    %             if PID_i < 0
    %                 PID_i = 0;
    %             end
    %         end
    %         PID_total = PID_p + PID_i;
    %         msg.Linear.X = PID_total;
    %         send(drive,msg)
    %         distance_previous_error = distance_error;
    %     end
catch
    disp('Unable to Solve for IP')
end


disp('Finished navigation')
pause(2)

 end
% end
