clear all
close all

N = 50;
theta1=linspace(0,2*pi,N);
theta2=linspace(0,pi/2,N);
d =linspace(0.01,0.1,N);

%initial frame
figure(1);hold on
axis([-0.13 0.13 -0.13 0.13 -0.13, 0.13]);
view([60,30])

for k=1:N
    
    gst(:,k)=forwardKinematics(theta1(k),theta2(k),d(k));
    l1 = plot3([0 0],[0,0],[0,0.1]);
    
    l2 = plot3([0 gst(1,k)],[0,gst(2,k)],[0.1,gst(3,k)],'-o');
    
    drawnow
    pause(0.1)
    
end

X = gst(1,:);
Y = gst(2,:);
Z = gst(3,:);
figure
comet3(X,Y,Z)





 
