clear all
close all

N = 50;
theta1=linspace(0,2*pi,N);
theta2=linspace(0,pi/2,N);
d =linspace(0.05,0.1,N);

for k=1:N
    
    gst(:,k)=forwardKinematics(theta1(k),theta2(k),d(k));

    
end

X = gst(1,:);
Y = gst(2,:);
Z = gst(3,:);
figure
plot3(X,Y,Z)

