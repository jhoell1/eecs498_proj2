clear all
%paper rotation 
t1 = 0;
t2 =pi;
d=11*25.4/2;
a = 8*25.4/2;
o0 = [0;0;0;1];
o1 = [a;0;d;1];
o2 = [a;0;-d;1];
o3 = [-a;0;-d;1];
o4 = [-a;0;d;1];
x = linspace(-2*25.4,-2*25.4,20);
y = linspace(0,0,20);
z = linspace(-2*25.4,2*25.4,20);

R1 = [1 0 0 0; 0 cos(t2) -sin(t2) 0;0 sin(t2) cos(t2) 0; 0 0 0 1];
R2 = [cos(t1) -sin(t1) 0 0;sin(t1) cos(t1) 0 0;0 0 1 0; 0 0 0 1];

o1 = R2*R1*o1;
o2 = R2*R1*o2;
o3 = R2*R1*o3;
o4 = R2*R1*o4;

P = [x;y;z;ones(1,20)];
P_rotated = R2*R1*P;
% figure;hold on
% plot3([o1(1),o3(1)],[o1(2),o3(2)],[o1(3),o3(3)])
% plot3([o1(1),o2(1)],[o1(2),o2(2)],[o1(3),o2(3)])
% plot3([o2(1),o4(1)],[o2(2),o4(2)],[o2(3),o4(3)])
% plot3([o4(1),o3(1)],[o4(2),o3(2)],[o4(3),o3(3)])
% hold off
% line(o2(1:3),o4(1:3));
% line(o4(1:3),o3(1:3));
% % line(o3(1:3),o1(1:3));
% X = [o1(1);o2(1);o3(1);o4(1)];
% Y = [o1(2);o2(2);o3(2);o4(2)];
% Z = [o1(3);o2(3);o3(3);o4(3)];
% figure
% mesh(X,Y,Z)
X = [o1(1) o1(1) o2(1) o4(1); o4(1) o2(1) o3(1) o3(1); 0 0 0 0];
Y = [o1(2) o1(2) o2(2) o4(2); o4(2) o2(2) o3(2) o3(2); 0 0 0 0];
Z = [o1(3) o1(3) o2(3) o4(3); o4(3) o2(3) o3(3) o3(3); 0 0 0 0];
C = [0.5000 0.5000 0.5000 0.5000 ;
     0.5000 0.5000 0.5000 0.5000 ;
     0.5000 0.5000 0.5000 0.5000];

figure
fill3(X,Y,Z,C)
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')
hold on
plot3(P(1,:),P(2,:),P(3,:),'r')
hold on
axis([-150, 150,-150 ,150,-150, 150]);

A = 4*25.4;
d = 0.01;
h = 4*A;
g0 = [0;d+4*A;h-4*A;1];

[T,J] = kenematics(0,pi/4,-pi,h,A);


gc(:,1) = g0;
q = [0;pi/4;-pi];
n = 1;
err = P_rotated(:,1) - gc(:,1);
nerr = norm(err);
for i = 1:20
    err = P_rotated(:,i) - gc(:,i);
    nerr = norm(err);
    while  nerr > 300
        err = P_rotated(:,i) - gc(:,i);
        nerr = norm(err);
        q = q+transpose(J)*err(1:3);
        gc(:,i) = kenematics(q(1),q(2),q(3),h,A)*gc(:,1);
        [T,J] = kenematics(q(1),q(2),q(3),h,A);
       n=n+1;
    end
    gc(:,i+1) = gc(:,i);
        
end

plot3(gc(1,:),gc(2,:),gc(3,:),'b')
        



