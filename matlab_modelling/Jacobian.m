
function J=Jacobian(theta1,theta2)

h = 0.1;
w1 = [0;0;1];
q1 = [0;0;0];
v1 = [0;0;0];
zeta1 = [v1;w1];


w2 = [1;0;0];
q2 = [0;0;h];
v2 = [0;h;0];
zeta2 = [v2;w2];
zeta2_p = [h*sin(theta1);-h*cos(theta1);0;-cos(theta1);-sin(theta1);0];


v3 = [0;1;0];
w3 = [0;0;0];
zeta3 = [v3;w3];
zeta3_p = [-sin(theta1)*cos(theta2);cos(theta1)*cos(theta2);-sin(theta2);0;0;0];

J = [zeta1 zeta2_p zeta3_p];


