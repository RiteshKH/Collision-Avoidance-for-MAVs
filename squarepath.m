
clc;
clear all;
close all;
wp = [2,2;2,8;8,8;8,2;2,2];
obs_x1 = [1;2.3;2.3;1;1];
obs_y1 = [4;4;5;5;4];

% obs_x2 = [4;6;6;4;4];
% obs_y2 = [7;7;8;8;7];
[obs_x2,obs_y2] = scircle1(5,8,1);

obs_x3 = [7;7;8.2;8.2;7];
obs_y3 = [4;6;6;4;4];


obs_x4 = [4;4;5;5;4];
obs_y4 = [1;2.8;2.8;1;1];

patch(obs_x1,obs_y1,'b.')
patch(obs_x2,obs_y2,'b.')
patch(obs_x3,obs_y3,'b.')
patch(obs_x4,obs_y4,'b.')

% patch(xc2,yc2,'b.')
hold on,  axis([0 10 0 10])
xlabel('metres'), ylabel('metres')
format compact
plot(wp(:,1),wp(:,2),'r');
xtrue = [wp(1,1);wp(1,2);atan2((wp(2,2)-wp(1,2)),(wp(2,1)-wp(1,1)))];
% obs = [obs_x1,obs_x2,obs_x3,obs_x4,obs_y1,obs_y2,obs_y3,obs_y4];
% for i = 1:4
%     xtrue = bearing_only(obs(:,i),obs(:,i+4),wp(i,:),wp(i+1,:),xtrue);
% end 

xtrue = bearing_only(obs_x1,obs_y1,wp(1,:),wp(2,:),xtrue);
xtrue = bearing_only(obs_x2,obs_y2,wp(2,:),wp(3,:),xtrue);
xtrue = bearing_only(obs_x3,obs_y3,wp(3,:),wp(4,:),xtrue);
xtrue = bearing_only(obs_x4,obs_y4,wp(4,:),wp(5,:),xtrue);