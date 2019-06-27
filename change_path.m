% Credits : Tim Bailey and Juan Nieto 2004. 
function xtrue = change_path(x,y,xtrue,rateG,dt,output,g)
configfile;
sp = [0 ; 0 ; 0];
ep = [9 ; 9 ; 0];
% cp = [xtrue(1);xtrue(2);0];
% ds = norm(cross(cross(ep-cp,sp-cp),ep-sp))/norm(ep-sp)^2;


if(xtrue(3)==g)
    return
end

ang1 = pi_to_pi(atan2((y(1) - xtrue(2)),(x(1) - xtrue(1))));
ang2 = pi_to_pi(atan2((y(2) - xtrue(2)),(x(2) - xtrue(1))));

vehicle= patch(0,0,'r','erasemode','xor'); % vehicle true
path= plot(0,0,'g-','markersize',2); % vehicle path estimate
veh=[0 -0.5 -0.5; 0 0.5 -0.5];
[xc,yc] = scircle1(6,4,2);
eta_d = 50*pi/180;

psi = xtrue(3);

a = ang1-psi;
b = ang2-psi;

if((ang2 > psi))
    G = -(eta_d - abs(b));
elseif((ang1 < psi))
    G = (eta_d - abs(a));
elseif(abs(b)<abs(a))
    G = -(eta_d - abs(b));
else
    G = (eta_d - abs(a))
end

maxDelta= rateG;
if abs(G) > maxDelta
    G= sign(G)*maxDelta;
end

xtrue= vehicle_model(xtrue, V,G,dt);

xt= TransformToGlobal(veh,xtrue); 
output= store_data(output,xtrue); 


set(vehicle, 'xdata', xt(1,:), 'ydata', xt(2,:))
    set(path, 'xdata', output.true(1,1:output.i), 'ydata', output.true(2,1:output.i))
drawnow

psi_d = xtrue(3)*180/pi;
[xfv yfv] = scircle1(xtrue(1),xtrue(2),1,[psi_d-60 psi_d+60]);
xfv(1) = xtrue(1);yfv(1) = xtrue(2);
xfv(end) = xtrue(1);yfv(end) = xtrue(2);

[x_i y_i] = polyxpoly(xfv,yfv,xc,yc);
if(isempty([x_i y_i])~=1)
   xtrue = change_path(x_i,y_i,xtrue,RATEG,dt,output,g);
end
end