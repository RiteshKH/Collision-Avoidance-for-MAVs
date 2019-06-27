% Credits : Tim Bailey and Juan Nieto 2004. 
function angle = pi_to_pi(angle)
% Credits : Tim Bailey and Juan Nieto 2004. 
% Input: array of angles.
 

angle = mod(angle, 2*pi);

i=find(angle>pi);
angle(i)=angle(i)-2*pi;

i=find(angle<-pi);
angle(i)=angle(i)+2*pi;
