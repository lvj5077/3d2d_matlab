% clc
clear
trailNum = 200;

detailNum= 1;

info1 = zeros(detailNum,trailNum,4);
info2 = zeros(detailNum,trailNum,4);
info3 = zeros(detailNum,trailNum,4);
addpath('/Users/jin/Q_Mac/mexopencv');
ratio = linspace(0.2,1,10);
imageNoises = linspace(0.2,1,detailNum); % pixel
depthNoise = 1/100; % cm
for jj = 1:detailNum
%     jj
    imageNoise = imageNoises(jj);
    for i = 1:trailNum
%         i
        [info1(jj,i,:),info2(jj,i,:),info3(jj,i,:)] = check_trans(1,imageNoise,depthNoise);
    end
end
save('myresults.mat')

% clc
disp("3d2d: "+num2str(mean(info1(1,:,3))*180/pi) )
disp("2d2d: "+num2str(mean(info2(1,:,3))*180/pi) )
disp("3d3d: "+num2str(mean(info3(1,:,3))*180/pi) )

% clc
% disp("~~~~~~~~~~~~~~~~~~~~~~~~~~~")
% mean(info1(1,:,2))*100
% % mean(info2(1,:,2))*100
% mean(info3(1,:,2))*100
% std(info1(1,:,3))
% std(info2(1,:,3))
% std(info3(1,:,3))
% %%
% figure,
% hold on,grid minor
% 
% %%
% figure,
% for jj = 1:rationNum
%     j = ratio(jj);
% %     hold on, plot(info2(jj,:,1),info2(jj,:,2),'.')
% %     hold on, plot(mean(info2(jj,:,1)),mean(info2(jj,:,2)),'r*')
% %     
%     hold on, plot(info1(jj,:,4),info1(jj,:,2),'.')
%     hold on, plot(mean(info1(jj,:,4)),mean(info1(jj,:,2)),'r*','MarkerSize',15,'LineWidth',5)
%     
% end
% hold on,grid minor
% labelSize = 20;
% xlabel("feature numbers",'FontSize',labelSize,'FontWeight','bold')
% y=ylabel("RPE-Translation (m)",'FontSize',1.3*labelSize,'FontWeight','bold');
% ylim([0 0.06])
% set(gcf,'color','w');
% set(gca,'FontSize',xyzsize)
% % mean(info1)
% % mean(info2)
% % mean(errorT1./gtT)
% % mean(errorT2./gtT)
% % plot(errorT),hold on, plot(errorTn),grid minor
% 
% % %%
% % trailNum = 500;
% % dst = zeros(trailNum,1);
% % for i = 1:trailNum
% % %     i
% %     imageIN = 0.25*[480*rand(1,30000);640*rand(1,30000)];
% %     density = mean(vecnorm((imageIN' - mean(imageIN'))'));
% %     dst(i) = density;
% % end
% % 
% % mean(dst)
