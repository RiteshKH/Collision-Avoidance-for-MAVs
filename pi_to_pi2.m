function angle = pi_to_pi2(angle)

% Input: array of angles.
 

angle = mod(angle, 2*pi);

% i=find(angle>pi);
% angle(i)=angle(i)-2*pi;
% 
% i=find(angle<-pi);
% angle(i)=angle(i)+2*pi;
