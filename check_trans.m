function [t_gt, info1, info2] = check_trans(ratio)
% close all
% clear
% clc

fx = 615;
fy = 615;
cx = 320;
cy = 240;

XYZ = [7*rand(3000,1)-3.5,7*rand(3000,1)-3.5,0.5+4*rand(3000,1)];
% ptCloud1 = pointCloud(XYZ); 
% figure,pcshow(ptCloud1),axis equal
dpt = zeros(480,640);
valid_idx=[];
id_i = [];
id_j = [];
v = [];
for i = 1:length(XYZ)
    px = round(fx*XYZ(i,1)/XYZ(i,3)+320);
    py = round(fx*XYZ(i,2)/XYZ(i,3)+240);
    pz = XYZ(i,3);
    if (px<640 && py<480 && px>0 && py>0 )
        if ( dpt(py,px) == 0)
            dpt(py,px) = pz;
            valid_idx = [valid_idx,i];
            id_i = [id_i,py];
            id_j = [id_j,px];
            v = [v,pz];
        end

    end
end
% [id_i,id_j,v] = find(dpt);
% figure,imagesc(dpt)
ll = length(id_i);

pt2d_fix = [id_i;id_j];

obj_fix = ones(ll,4);
for i = 1:ll
    obj_fix(i,1) = v(i)*(id_j(i)-320)/fx;
    obj_fix(i,2) = v(i)*(id_i(i)-240)/fx;
    obj_fix(i,3) = v(i);
end
% ([XYZ(valid_idx,:),ones(ll,1)]\obj_fix)'
% ptCloud1 = pointCloud(obj_fix); 
% figure,pcshow(ptCloud1)
%%
T1= eye(4);
% qut = rand(1,4);
% qut = qut/norm(qut);
% T1(1:3,1:3) = quat2rotm(qut);
% T1(1:3,1:3) = eul2rotm([20,0,0]*pi/180);
%  T1(1:3,4) = 1.5 * ([0.5,0.5,0.5]');
% rand(1,3)-
T1(1:3,1:3) = eul2rotm(([0.5,0.5,0.5])*10*pi/180);
T1(1:3,4) = 1.5 * ([0.5,0.5,0.5]');
t_gt = norm(T1(1:3,4));
obj_mv = (T1*obj_fix')';
% ptCloud1 = pointCloud(obj_mv(:,1:3)); 
% figure,pcshow(ptCloud1)
%%

img_mv = zeros(480,640);
valid_match = zeros(1,ll);
valid_idx=[];
id_i = [];
id_j = [];
v = [];
for i = 1:ll
    px = (fx*obj_mv(i,1)/obj_mv(i,3)+320);
    py = (fx*obj_mv(i,2)/obj_mv(i,3)+240);
    pz = obj_mv(i,3);
    if (px<640 && py<480 && px>0 && py>0 )
%         if (img_mv(py,px)==0)
%             img_mv(py,px) = pz;
            valid_match(i) = 1;
            valid_idx = [valid_idx,i];
            id_i = [id_i,py];
            id_j = [id_j,px];
            v = [v,pz];
%         end
    end
end
% [id_i,id_j,v] = find(img_mv);
% valid_obs_index = find(valid_match);

pt2d_mv = [id_j;id_i];
% figure,imagesc(img_mv)
ll2 = length(id_i);
% 
obj_mv_valid = ones(ll2,4);
for i = 1:ll2
    obj_mv_valid(i,1) = v(i)*(id_j(i)-320)/fx;
    obj_mv_valid(i,2) = v(i)*(id_i(i)-240)/fx;
    obj_mv_valid(i,3) = v(i);
end

%%

% find3dto3d = (obj_fix(valid_idx,:)\obj_mv_valid)';
% % t_error = norm(T1(1:3,4)-find3dto3d(1:3,4));
% rotm2eul(find3dto3d(1:3,1:3))*180/pi
%%

obj_input = obj_fix(valid_idx,1:3);
img_input = pt2d_mv';
img_input = img_input+3*(rand(length(pt2d_mv),2)-0.5*ones(length(pt2d_mv),2));
%%
data = load('worldToImageCorrespondences.mat');
intrinsics = cameraIntrinsics( fx , [cx,cy] , [640,480] );
% [worldOrientation,worldLocation] = estimateWorldCameraPose(img_input,obj_input,data.cameraParams);
% t_error = norm(T1(1:3,4)+worldLocation');
%%
region_idx = (pt2d_mv(1,:)>640*ratio&pt2d_mv(2,:)>480*ratio);
% ll2

fff = find(region_idx);
% length(fff)
imageIN = img_input(fff,:);
objIN = obj_input(fff,:);
density = mean(vecnorm((imageIN' - mean(imageIN'))')); %/ ((1-ratio)*(1-ratio));

% [worldOrientation,worldLocation] = estimateWorldCameraPose(imageIN,obj_input(fff,:),data.cameraParams)
[worldOrientation,worldLocation] = estimateWorldCameraPose(imageIN,objIN,intrinsics)
% [worldOrientation,worldLocation] = estimateWorldCameraPose(imageIN,objIN,data.cameraParams)
% img_inputNoise = img_input+3*(rand(length(pt2d_mv),2)-0.5*ones(length(pt2d_mv),2));
% [worldOrientation,worldLocation] = estimateWorldCameraPose(img_inputNoise,obj_input,data.cameraParams);

t_error = norm(T1(1:3,4)+worldLocation');
r_error = norm(rotm2eul(T1(1:3,1:3))*180/pi - rotm2eul(worldOrientation)*180/pi);
info1 = [density;t_error;r_error;length(imageIN)];
% ll2
% min(imageIN(:,1))
% min(imageIN(:,1))
% t_error_2 
%%
% length(fff)/ll2
fff2 = randperm(ll2,length(fff));
imageIN = img_input(fff2,:);
objIN = obj_input(fff2,:);
density = mean(vecnorm((imageIN' - mean(imageIN'))')) ;
[worldOrientation,worldLocation] = estimateWorldCameraPose(imageIN,objIN,intrinsics);
t_error = norm(T1(1:3,4)+worldLocation');

% rotm2eul(worldOrientation)*180/pi
r_error = norm(rotm2eul(T1(1:3,1:3))*180/pi - rotm2eul(worldOrientation)*180/pi);
info2 = [density;t_error;r_error;length(imageIN)];
end

