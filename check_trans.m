function [info1, info2, info3] = check_trans(ratio,imageNoise,depthNoise)
% close all
% clear
% clc

fx = 615;
fy = 615;
cx = 320;
cy = 240;
K = [fx,0,cx;0,fy,cy;0,0,1];

XYZ = [7*rand(3000,1)-3.5,7*rand(3000,1)-3.5,4+0.5+4*rand(3000,1)];
% ptCloud1 = pointCloud(XYZ); 
% figure,pcshow(ptCloud1),axis equal
dpt = zeros(480,640);
obj_fix = [];
img_fix = [];

for i = 1:length(XYZ)
    px = (fx*XYZ(i,1)/XYZ(i,3)+320);
    py = (fx*XYZ(i,2)/XYZ(i,3)+240);
    pz = XYZ(i,3);
    if (round(px)<640 && round(py)<480 && round(px)>0 && round(py)>0 )
        if ( dpt(round(py),round(px)) == 0)
            dpt(round(py),round(px)) = pz;
            p2d = [px,py];
            img_fix = [img_fix;p2d];
            obj_fix = [obj_fix;XYZ(i,1:3),1];
        end

    end
end
% [id_i,id_j,v] = find(dpt);
% figure,imagesc(dpt)
ll = length(obj_fix);

%%
T1= eye(4);
% qut = rand(1,4);
% qut = qut/norm(qut);
% T1(1:3,1:3) = quat2rotm(qut);
% T1(1:3,1:3) = eul2rotm([20,0,0]*pi/180);
%  T1(1:3,4) = 1.5 * ([0.5,0.5,0.5]');
% rand(1,3)-
T1(1:3,1:3) = eul2rotm(([5,7,13])*pi/180);
T1(1:3,4) = ([0.05,0.05,0.05]');

obj_mv = (T1*obj_fix')';
% ptCloud1 = pointCloud(obj_mv(:,1:3)); 
% figure,pcshow(ptCloud1)
%%

obj_mv_valid = [];
img_mv_valid = [];

obj_fix_valid = [];
img_fix_valid = [];
for i = 1:ll
    px = (fx*obj_mv(i,1)/obj_mv(i,3)+320);
    py = (fx*obj_mv(i,2)/obj_mv(i,3)+240);

    if (px<640 && py<480 && px>0 && py>0 )
        obj_mv_valid = [obj_mv_valid;obj_mv(i,1:3)];
        obj_fix_valid = [obj_fix_valid;obj_fix(i,1:3)];
        p2d = [px,py];
        img_mv_valid = [img_mv_valid;p2d];
        img_fix_valid = [img_fix_valid;img_fix(i,:)];
    end
end

%%
dnoise = depthNoise*wgn(length(img_fix_valid),1,0);
dnoise = obj_mv_valid(:,3).*dnoise;
obj_input_mv = obj_mv_valid + [dnoise/fx,dnoise/fx,dnoise];
dnoise = depthNoise*wgn(length(img_fix_valid),1,0);
obj_input_fix = obj_fix_valid + [dnoise/fx,dnoise/fx,dnoise];

img_input_mv = img_mv_valid + imageNoise*wgn(length(img_fix_valid),2,0);
img_input_fix = img_fix_valid + imageNoise*wgn(length(img_fix_valid),2,0);


%%
% data = load('worldToImageCorrespondences.mat');
intrinsics = cameraIntrinsics( fx , [cx,cy] , [640,480] );

% length(fff)/ll2
fff2 = randperm(length(img_fix_valid),300);
% fff2 = randperm(ll2,length(fff));
obj_input_mv = obj_input_mv(fff2,:);
obj_input_fix = obj_input_fix(fff2,:);
img_input_mv = img_input_mv(fff2,:);
img_input_fix = img_input_fix(fff2,:);


density = mean(vecnorm((img_input_mv' - mean(img_input_mv'))')) ;
% [worldOrientation,worldLocation] = estimateWorldCameraPose(imageIN,objIN,intrinsics);
% t_error = norm(T1(1:3,4)+worldLocation');
t_error = 0;
% % rotm2eul(worldOrientation)*180/pi
% r_error = rotm2axang((T1(1:3,1:3)) * (worldOrientation)');
% info2 = [density;t_error;r_error(4);length(imageIN)];

%%
[rvec, tvec, success, inliers] = cv.solvePnPRansac(obj_input_fix, img_input_mv, K);
Rcv = cv.Rodrigues(rvec);
tvec;
rotm2eul(Rcv)*180/pi;
r_error = rotm2axang((T1(1:3,1:3))*Rcv');
if(r_error(4) >2)
    r_error(4) = pi-r_error(4);
end
info1 = [density;norm(T1(1:3,4) - tvec);r_error(4);length(obj_input_fix)];

%%

[F, mask] = cv.findFundamentalMat(img_input_fix,img_input_mv, 'Method','Ransac','RansacReprojThreshold',1);
E = K' * F * K;
[R, t, good, mask, triangulatedPoints] = cv.recoverPose(E,img_input_fix,img_input_mv);
rotm2eul(R)*180/pi;
r_error = rotm2axang((T1(1:3,1:3)) * R');
if(r_error(4) >2)
    r_error(4) = pi-r_error(4);
end
info2 = [density;t_error;r_error(4);length(img_input_fix)];

%%
% obj_input_mv = (T1(1:3,4)+ T1(1:3,1:3)* obj_input_fix')';
[R,t] = Ransac3d3d(obj_input_mv, obj_input_fix);
rotm2eul(R)*180/pi;
r_error = rotm2axang((T1(1:3,1:3)) * R');
if(r_error(4) >2)
    r_error(4) = pi-r_error(4);
end
info3 = [density;norm(T1(1:3,4) - t);r_error(4);length(obj_input_fix)];
% disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
end

