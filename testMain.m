clc
clear
trailNum = 1;
rationNum = 1;
gtT = zeros(rationNum,trailNum,1);
info1 = zeros(rationNum,trailNum,4);
info2 = zeros(rationNum,trailNum,4);

for jj = 1:rationNum
    j = 0.05*jj

    for i = 1:trailNum
%         i
        [gtT(jj,i), info1(jj,i,:),info2(jj,i,:)] = check_trans(j);
    end
% mean(gtT)
%     hold on, plot(info1(jj,:,1),info1(jj,:,4),'.')
%     hold on, plot(mean(info1(jj,:,1)),mean(info1(jj,:,4)),'r*')
%     hold on, plot(info2(:,1),info2(:,2),'.')
%     hold on, plot(mean(info2(:,1)),mean(info2(:,2)),'r*')
end
save('myresults.mat')
%%
figure,
hold on,grid minor

%%
figure,
for jj = 1:rationNum
    j = 0.05*jj
%     hold on, plot(info2(jj,:,1),info2(jj,:,2),'.')
%     hold on, plot(mean(info2(jj,:,1)),mean(info2(jj,:,2)),'r*')
%     
    hold on, plot(info1(jj,:,1),info1(jj,:,3),'.')
    hold on, plot(mean(info1(jj,:,1)),mean(info1(jj,:,3)),'r*')
    
end
hold on,grid minor

% mean(info1)
% mean(info2)
% mean(errorT1./gtT)
% mean(errorT2./gtT)
% plot(errorT),hold on, plot(errorTn),grid minor

%%
trailNum = 500;
dst = zeros(trailNum,1);
for i = 1:trailNum
%     i
    imageIN = 0.25*[480*rand(1,30000);640*rand(1,30000)];
    density = mean(vecnorm((imageIN' - mean(imageIN'))'));
    dst(i) = density;
end

mean(dst)
