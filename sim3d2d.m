clear
close all
format short g
clc
addpath('/Users/jin/Q_Mac/mexopencv');

testNum = 100;
samples = 800;
stdmm = zeros(40,8);
memm = zeros(40,8);
depthm = zeros(40,1);
% A_depth
for j = 1:40
    j
    A_depth = 1+0.1*j;
    depthm(j) = A_depth;
    results = zeros(testNum,4);
    results_pnp = zeros(testNum,4);
    pixelNoise = zeros(testNum,2);
    for testId = 1:testNum
        fixImg = [640*rand(samples,1),480*rand(samples,1)];
        NsImg = fixImg;
        NsImg = NsImg + 0.02*wgn(samples,2,0) ;
        fixImg(:,1) = (fixImg(:,1)-320)/520;
        fixImg(:,2) = (fixImg(:,2)-240)/520;

        NsImg(:,1) = (NsImg(:,1)-320)/520;
        NsImg(:,2) = (NsImg(:,2)-240)/520;

        dR = eul2rotm([0.1,0.2,0.3]*pi/180);
        dT = [.001,0.002,0.003]';

        d_base = .5*rand(samples,1)+A_depth;
        d  = d_base;
%         for i = 1:samples
%             d(i) = d(i) + d(i)*wgn(1,1,0)/2.;
%         end
        fixObj = [fixImg.*d,d];


        d2  = d_base;
        for i = 1:samples
            d2(i) = d2(i) + d2(i)*wgn(1,1,0)/2.;
        end
        mvObj = [fixImg.*d2,d2];

    %     fixObj = fixObj*dR+dT';

        K = [1,0,0;0,1,0;0,0,1];

    %     [R_out,t_out] = Ransac3d3d(fixObj,mvObj)

        [rvec, tvec, success, inliers] = cv.solvePnPRansac(fixObj, NsImg, K);
        rotm = cv.Rodrigues(rvec);

        results_pnp(testId,1:3) = 1000*tvec;%rotm2eul(rotm)*180/pi;
        axang = rotm2axang(rotm);
        results_pnp(testId,4) = 1000*norm(tvec);%axang(4)*180/pi;
        results_pnp(testId,5:7) = rotm2eul(rotm)*180/pi;
        results_pnp(testId,8) = axang(4)*180/pi;

    end

    memm(j,:) = mean(abs(results_pnp));
    stdmm(j,:) = std(results_pnp);
end

%%
figure,
titleSize =50
labelSize =20
xyzsize = 30
set(gcf,'color','w');
subplot(1,2,1)
plot(depthm,memm(:,4),'.','MarkerSize',20,'LineWidth',10)
xlabel("mean depth (m)",'FontSize',labelSize,'FontWeight','bold')
y=ylabel("mean error (mm)",'FontSize',1.3*labelSize,'FontWeight','bold');
set(y, 'position', get(y,'position')-[0.01,0,0]); 
grid minor
set(gca,'FontSize',xyzsize)

p1=fit(depthm,memm(:,4),'poly1')
hold on, pt2 = plot(depthm,p1(depthm),'r','LineWidth',5)

subplot(1,2,2)
plot(depthm,stdmm(:,4),'.','MarkerSize',20,'LineWidth',10)
xlabel("mean depth (m)",'FontSize',labelSize,'FontWeight','bold')
y=ylabel("std (mm)",'FontSize',1.3*labelSize,'FontWeight','bold');
set(y, 'position', get(y,'position')-[0.01,0,0]); 
grid minor
set(gca,'FontSize',xyzsize)

p2=fit(depthm,stdmm(:,4),'poly1')
hold on, pt2 = plot(depthm,p2(depthm),'r','LineWidth',5)


