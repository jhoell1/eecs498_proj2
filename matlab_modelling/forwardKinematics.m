
function gst=forwardKinematics(theta1,theta2,theta3)
A = 0.1;
d = 0.01;
h = 0.1;
w1 = [0;0;1];
q1 = [0;0;0];
v1 = [0;0;0];
zeta1_h = [0 -1 0 0;1 0 0 0;0 0 0 0;0 0 0 0];
g1 = expm(zeta1_h*theta1);

w2 = [1;0;0];
q2 = [0;0;h];
v2 = [0;h;0];
zeta2_h = [0 0 0 0;0 0 1 -h;0 -1 0 0;0 0 0 0];
g2 = expm(zeta2_h*theta2);

%hoekens linkage 
z_trans = 2*A +2*A*(sqrt(5-sin(theta3))*cos(theta3))/sqrt(5+4*sin(theta3));
y_trans = 2*A*(sqrt(5-sin(theta3))*(2+sin(theta3)))/sqrt(5+4*sin(theta3));

zeta3_h = [0 0 0 0;0 0 0 1;0 0 0 0;0 0 0 0];
g3 = expm(zeta3_h*(d+y_trans));

zeta4_h = [0 0 0 0;0 0 0 0;0 0 0 1;0 0 0 0];
g4 = expm(zeta4_h*(z_trans));

gst_ini = [0; 0; h;1];

gst = g1*g2*g3*g4*gst_ini;
end


            