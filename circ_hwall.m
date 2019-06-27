clc;
clear all;
close all;
S = warning('OFF')
[xc1,yc1] = scircle1(3,3,1.5);
%[xc1,yc1] = scircle1([3;NaN;7],[3;NaN;5],[1;NaN;1]);
yc2 = [6;6;6.5;6.5;6]  ;
xc2 = [8;4;4;8;8]  ;
% cir = [xc,yc] - 0.2.*ones(size([xc yc]));
xc = [xc1;NaN;xc2];
yc = [yc1;NaN;yc2];
sp = [0 ; 0];
ep = [9 ; 9];
i = 0;
j=0;
eta_d = 50*pi/180;

fig=figure;
patch(xc1,yc1,'b.')
patch(xc2,yc2,'b.')
hold on,  axis([0 10 0 10])
xlabel('metres'), ylabel('metres')
format compact

configfile;
xfv = 0;yfv = 0;

vehicle= patch(0,0,'r','erasemode','xor'); % vehicle true
path= plot(0,0,'g-','markersize',2); % vehicle path estimate
veh=[0 -0.5 -0.5; 0 0.2 -0.2]; % vehicle animation % [0 90 90; 0 -50 50]
xtrue= [sp(1);sp(2);0];

dt= DT_CONTROLS;
G = 0; % initial steer angle
output = initialise_store(xtrue);
 % Distance between robot and end point
 d = 1;
 flag = 0;
 g_vec = [];
while(d > 0.5)
% xtrue(3)
    psi_d = xtrue(3)*180/pi;
    [xfv yfv] = scircle1(xtrue(1),xtrue(2),1,[psi_d-60 psi_d+60]);
    xfv(1) = xtrue(1);yfv(1) = xtrue(2);
    xfv(end) = xtrue(1);yfv(end) = xtrue(2);

    [x_i y_i] = polyxpoly(xfv,yfv,xc,yc);
    if((isempty([x_i y_i])~=1)&&(flag~=1))
        g_vec = [g_vec;xtrue(3)];
        if(numel(g_vec)>2)
                if ((sign(g_vec(1)-g_vec(2)))==1)
                            if xtrue(3)>g_vec(1)
                                flag = 1;
                                g_vec = [];
                            end
                else
                            if xtrue(3)<g_vec(1)
                                flag = 1;
                                g_vec = [];
                            end
                end
        end
        ang1 = pi_to_pi(atan2((y_i(1) - xtrue(2)),(x_i(1) - xtrue(1))));
        ang2 = pi_to_pi(atan2((y_i(2) - xtrue(2)),(x_i(2) - xtrue(1))));
        if (ang1<ang2)
            temp = ang1;
            ang1 = ang2;
            ang2 = temp;
        end
        eta_d = 50*pi/180;
        psi = xtrue(3);

        a = ang1-psi;
        b = ang2-psi;

        if((ang2 > psi))
            G = -(eta_d - abs(b));
        elseif((ang1 < psi))
            G = (eta_d - abs(a));
        elseif(abs(b)<abs(a))
            G = -(eta_d + abs(b));
        else
            G = (eta_d + abs(a));
        end
        deltaG = G/dt;
        
        i = 0;
    else
        
        d= (ep(1)- xtrue(1))^2 + (ep(2) - xtrue(2))^2;
        deltaG= pi_to_pi(atan2(ep(2)-xtrue(2), ep(1)-xtrue(1)) - xtrue(3));
        [x_i2 y_i2] = polyxpoly([xtrue(1);ep(1)],[xtrue(2);ep(2)],xc,yc);
        if(isempty([x_i2 y_i2])~=1)
            if(i<50)
                i = i+1;
            else
                i=0;
                flag = 0;
            end
        end
    end

maxDelta= RATEG*dt;
if abs(deltaG) > maxDelta
    deltaG= sign(deltaG)*maxDelta;
end
G = deltaG;
if(numel(g_vec)>0)
end
xtrue= vehicle_model(xtrue, V,G,dt);
xt= TransformToGlobal(veh,xtrue); 
output= store_data(output,xtrue);  

set(vehicle, 'xdata', xt(1,:), 'ydata', xt(2,:))
    set(path, 'xdata', output.true(1,1:output.i), 'ydata', output.true(2,1:output.i))
drawnow 

end