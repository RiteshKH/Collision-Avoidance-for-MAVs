% Credits : Tim Bailey and Juan Nieto 2004. 
%%% Configuration file
%%% Permits various adjustments to parameters of the filter.
 
% Control Parameters

V=0.3; % Velocity of the robot in 'meters/second'.
MAXG= 60*pi/180;  % Maximum steering angle (orientation) of robot in 'radians' (-MAXG < g < MAXG)
RATEG= 40*pi/180; % Maximum rate of change of steer angle in 'radians/second'
WHEELBASE=  4;     % Robot wheel-base in 'meters'
DT_CONTROLS=0.1;% Time interval between control signals in 'seconds'
  


% waypoint proximity
AT_WAYPOINT= 1.0; % Minimum distance treshold to switch to next waypoint.
NUMBER_LOOPS= 2; % number of loops through the waypoint list


