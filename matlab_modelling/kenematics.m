function [T03,JV] = kenematics(t1,t2,t3,h,A)
para = 0.08889*25.4*360/(2*pi);
% syms t1 t2 t3
% syms h d A
% 
% assume([t1 t2 t3 t1_dot t2_dot t3_dot h d A],'real');

% th = [t1 t2 t3];  
% t_dot = [t1_dot t2_dot t3_dot];
%hoekens linkage 
% x_trans = 2*A +2*A*(sqrt(5-sin(t3))*cos(t3))/sqrt(5+4*sin(t3));
% z_trans = 2*A*(sqrt(5-sin(t3))*(2+sin(t3)))/sqrt(5+4*sin(t3));

T01 = [cos(t1+pi/2),0, sin(t1+pi/2), 0;
       sin(t1+pi/2), 0, -cos(t1+pi/2) , 0;
       0,      1,        0, h;
       0,       0,        0, 1];
T12 = [ cos(t2-pi/2),0,sin(t2-pi/2),(4*A)*cos(t2-pi/2);
        sin(t2-pi/2),0,-cos(t2-pi/2),(4*A)*sin(t2-pi/2);
        0,1,0,0;
        0,0,0,1];

T23 = [ 0, 0, 0,0;
        0, 0, 0, 0;
        0, 0, 0, para*t3;
        0,0,0,1];
T01 = (T01);
T02 = (T01*T12);
T03 = (T02*T23);


%

O0 = [0;0;0];
O1 = T01(1:3,4);
O2 = T02(1:3,4);
O3 = T03(1:3,4);

Z0 = [0;0;1];
Z1 = T01(1:3,3);
Z2 = T02(1:3,3);
Z3 = T03(1:3,3);

%% Problem 1(2)
Jv1 = cross(Z0,O3-O0);
Jv2 = cross(Z1,O3-O1);
Jv3 = cross(Z2,O3-O2);
JV = [Jv1 Jv2 Jv3];
JV = (JV);



