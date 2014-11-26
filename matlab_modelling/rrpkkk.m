clear all
close all

N = 20;
t1=linspace(0,0,N);
t2=linspace(pi/2,pi/2,N);
t3=linspace(-pi,pi,N);
A = 2*25.4;
d = 0.01;
h = 4*A;


% %initial frame
% figure(1);hold on
% axis([-30, 30,-30 ,30,-30, 30]);
% view([60,30])
gst(:,1) = [0;d+4*A;h-4*A;1];
n = 1;
for k=1:N
     for i = 1:N
         for j = 1:N
            gst(:,n+1) = kenematics(t1(k),t2(i),t3(j),h,A)*gst(:,1);
%             l1 = line([0 0],[0 0],[0 h]);
%             l2 = line([0 gst(1,n)],[0 gst(2,n)],[h gst(3,n)]);
%             pause(1)
            n= n+1;
         end
         j=1;
     end
     i = 1;
end
        figure
        X = gst(1,:);
        Y = gst(2,:);
        Z = gst(3,:);
        
        plot3(X,Y,Z)
       axis([-500, 500,-500 ,500,-500, 500]);

%     end
%     
% end

% X = gst(1,:);
% Y = gst(2,:);
% Z = gst(3,:);
% figure
% comet3(X,Y,Z)
