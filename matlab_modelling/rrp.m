clear all
close all

N = 10;
theta1=linspace(0,2*pi,N);
theta2=linspace(0,pi/2,N);
theta3=linspace(0,pi*2,N);



% %initial frame
figure(1);
axis([-0.13 0.13 -0.13 0.13 -0.13, 0.13]);
view([60,30])
%gst_init = [0;0;0.1;1];
for k=1:N
    for i = 1:N
        for j = 1:N
            gst(:,k)=forwardKinematics(theta1(k),theta2(i),theta3(j));
        end
        X = gst(1,:);
        Y = gst(2,:);
        Z = gst(3,:);
        
        comet3(X,Y,Z)
        hold on

    end
    
end

X = gst(1,:);
Y = gst(2,:);
Z = gst(3,:);
figure
comet3(X,Y,Z)





 