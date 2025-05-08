clear all;
close all;
clc;

%输入已知数据
L1=27.33;L2=59.07;L3=61.29;L4=46.55;L5=62.89;L6=71.95;L7=45.19;L8x=98.2;L8y=13.39;epsilon=21.89;
omega1=1;%rad/s
alpha1=0;%rad/s^2
hd=pi/180;du=180/pi;%度*hd=弧度 弧度*du=度

for n1=1:361
    theta1=(n1-1)*hd;
    [theta,omega,alpha]=solution(theta1,omega1,alpha1,L1,L2,L3,L4,L5,L6,L7);
    theta2(n1)=theta(1);theta3(n1)=theta(2);theta6(n1)=theta(3);theta7(n1)=theta(4);theta5(n1)=theta(5);
    omega2(n1)=omega(1);omega3(n1)=omega(2);omega6(n1)=omega(3);omega7(n1)=omega(4);
    alpha2(n1)=alpha(1);alpha3(n1)=alpha(2);alpha6(n1)=alpha(3);alpha7(n1)=alpha(4);
end


%输出线图

figure;
n1=1:2*361;
theta20=[theta2,theta2];
omega20=[omega2,omega2];
alpha20=[alpha2,alpha2];
theta30=[theta3,theta3];
omega30=[omega3,omega3];
alpha30=[alpha3,alpha3];
theta60=[theta6,theta6];
omega60=[omega6,omega6];
alpha60=[alpha6,alpha6];
theta70=[theta7,theta7];
omega70=[omega7,omega7];
alpha70=[alpha7,alpha7];
t=n1/omega1;
subplot(4,3,1);%位移线图2
plot(t,theta20,'k')
title('角位移线图');
xlabel('时间/s')
ylabel('杆件2角位移/rad')
grid on;hold on;
subplot(4,3,2);
plot(t,omega20,'b')
title('角速度线图');
xlabel('时间/s')
ylabel('杆件2角速度/rad/s')
grid on;hold on;
subplot(4,3,3);
plot(t,alpha20,'r')
title('角加速度线图');
xlabel('时间/s')
ylabel('杆件2角加速度/rad/s^2')
grid on;hold on;
subplot(4,3,4);%位移线图3
plot(t,theta30,'k')
title('角位移线图');
xlabel('时间/s')
ylabel('杆件3角位移/rad')
grid on;hold on;
subplot(4,3,5);
plot(t,omega30,'b')
title('角速度线图');
xlabel('时间/s')
ylabel('杆件3角速度/rad/s')
grid on;hold on;
subplot(4,3,6);
plot(t,alpha30,'r')
title('角加速度线图');
xlabel('时间/s')
ylabel('杆件3角加速度/rad/s^2')
grid on;hold on;
subplot(4,3,7);%位移线图4
plot(t,theta60,'k')
title('角位移线图');
xlabel('时间/s')
ylabel('杆件4角位移/rad')
grid on;hold on;
subplot(4,3,8);
plot(t,omega60,'b')
title('角速度线图');
xlabel('时间/s')
ylabel('杆件4角速度/rad/s')
grid on;hold on;
subplot(4,3,9);
plot(t,alpha60,'r')
title('角加速度线图');
xlabel('时间/s')
ylabel('杆件4角加速度/rad/s^2')
grid on;hold on;
subplot(4,3,10);%位移线图5
plot(t,theta70,'k')
title('角位移线图');
xlabel('时间/s')
ylabel('杆件5角位移/rad')
grid on;hold on;
subplot(4,3,11);
plot(t,omega70,'b')
title('角速度线图');
xlabel('时间/s')
ylabel('杆件5角速度/rad/s')
grid on;hold on;
subplot(4,3,12);
plot(t,alpha70,'r')
title('角加速度线图');
xlabel('时间/s')
ylabel('杆件5角加速度/rad/s^2')
grid on;hold on;
figure(3);
subplot(1,2,1);
n1=1:361    
x3=L1* cos((n1 - 1)*hd)+L2*cos(theta2(n1));
 y3=L1* sin((n1 - 1)*hd)+L2*sin(theta2(n1));
plot(x3,y3,'k')
title('C点轨迹');
xlabel('x')
ylabel('y')
axis equal;
grid on;hold on;
subplot(1,2,2);
    x6=L1* cos((n1 - 1)*hd)+L5*cos(theta5(n1));
    y6=L1* sin((n1 - 1)*hd)+L5*sin(theta5(n1));
plot(x6,y6,'k')
title('E点轨迹');
xlabel('x')
ylabel('y')
axis equal;
grid on;hold on;
figure(2)
m=moviein(15);
j=0;
for n1=1:5:360
    j=j+1;
    clf;
    x(1)=0;                 
    y(1)=0;
    x(2)=L1*cos((n1-1)*hd);
    y(2)=L1*sin((n1-1)*hd);
    x(3)=L1* cos((n1 - 1)*hd)+L2*cos(theta2(n1));
    y(3)=L1* sin((n1 - 1)*hd)+L2*sin(theta2(n1));
    x(4)=L4;
    y(4)=0;
    x(5)=L1* cos((n1 - 1)*hd)+L2*cos(theta2(n1));
    y(5)=L1* sin((n1 - 1)*hd)+L2*sin(theta2(n1));
    x(6)=L1* cos((n1 - 1)*hd)+L5*cos(theta5(n1));
    y(6)=L1* sin((n1 - 1)*hd)+L5*sin(theta5(n1));
    x(7)=L1*cos((n1-1)*hd);
    y(7)=L1*sin((n1-1)*hd);
    x(8)=77.51+L7*cos(theta7(n1))+L6*cos(theta6(n1));
    y(8)=31.26+L7*sin(theta7(n1))+L6*sin(theta6(n1));
    x(9)=77.51+L7*cos(theta7(n1));
    y(9)=31.26+L7*sin(theta7(n1));
    x(10)=77.51;
    y(10)=31.26;
    plot(x,y);
    grid on;hold on;
    plot(x(1),y(1),'o');
    plot(x(2),y(2),'o');
    plot(x(3),y(3),'o');
    plot(x(4),y(4),'o');
    plot(x(6),y(6),'o');
    plot(x(9),y(9),'o');
    plot(x(10),y(10),'o');
    axis([-50 100 -50 100]);
    axis equal;
    title('运动仿真');xlabel('mm');ylabel('mm');
    m(j)=getframe;
    
end
movie(m);

%%%%%%%%%%%%%%%%%%%%%%%%子函数solution%%%%%%%%%%%%%%%%%%%%
function[theta,omega,alpha]=solution(theta1,omega1,alpha1,L1,L2,L3,L4,L5,L6,L7)
%1.计算从动件角位移
L=sqrt(L4*L4+L1*L1-2*L1*L4*cos(theta1));
phi=asin((L1*sin(theta1))/L);
beta=acos((-L2*L2+L3*L3+L*L)/(2*L3*L));
% if beta<0
%     beta=beta+pi;
% end
theta3=pi-phi-beta;
gamma=acos((L2*L2+L*L-L3*L3)/(2*L*L2));
epcl=acos((L4*L4+L*L-L1*L1)/(2*L*L4));
if theta1>pi
   epcl=-epcl; 
end
theta2=gamma-epcl;
theta5=theta2+(21.89*pi/180);
L8=sqrt((77.51-L1*cos(theta1))*(77.51-L1*cos(theta1))+(31.26-L1*sin(theta1))*(31.26-L1*sin(theta1)));
theta0=asin((31.26-L1*sin(theta1))/L8);
L0=sqrt((L5*L5)+L8*L8-2*L5*L8*cos(theta5-theta0));
phi0=asin((L5*sin(theta5-theta0))/L0);
beta0=acos((L7*L7+L0*L0-L6*L6)/(2*L7*L0));
theta7=pi+beta0-phi0+theta0;
theta6=(-pi)+theta7+acos((L6*L6+L7*L7-L0*L0)/(2*L6*L7));
theta=[theta2;theta3;theta6;theta7;theta5];


%2.计算从动件角速度
A=[-L2*sin(theta2),L3*sin(theta3);L2*cos(theta2),-L3*cos(theta3)];
B=[L1*sin(theta1);-L1*cos(theta1)];
omega=A\(omega1*B);
omega2=omega(1);omega3=omega(2);%机构从动件的速度列阵
C=[L6*sin(theta6),L7*sin(theta7);L6*cos(theta6),L7*cos(theta7)];
D=[omega1*L1*sin(theta1)+omega2*L5*sin(theta5);omega1*L1*cos(theta1)+omega2*L5*cos(theta5)];
omega0=C\D;
omega6=omega0(1);omega7=omega0(2);
if omega2<0
   omega2=-omega2;
end
if omega3<0
   omega3=-omega3;
end
if omega6<0
   omega6=-omega6;
end
if omega7<0
   omega7=-omega7;
end
omega=[omega2;omega3;omega6;omega7;omega2];

%3.计算从动件的角加速度
A=[-L2*sin(theta2),L3*sin(theta3);L2*cos(theta2),-L3*cos(theta3)];
At=[-omega2*L2*cos(theta2),omega3*L3*cos(theta3);-omega2*L2*sin(theta2),omega3*L3*sin(theta3)];
B=[L1*sin(theta1);-L1*cos(theta1)];
Bt=[omega1*L1*cos(theta1);omega1*L1*sin(theta1)];
alpha=A\(-At*omega1+alpha1*B+omega1*Bt);%对线性方程组 Ax = B 求解 x %机构从动件的加速度列阵
alpha2=alpha(1);
alpha3=alpha(2);
C=[-L6*sin(theta6),-L7*sin(theta7);L6*cos(theta6),L7*cos(theta7)];
D=[-omega1*omega1*L1*cos(theta1)-L1*alpha1*sin(theta1)-omega2*omega2*L5*cos(theta5)-alpha2*L5*sin(theta5)+omega6*omega6*L6*cos(theta6)+omega7*omega7*L7*cos(theta7);
   -omega1*omega1*L1*sin(theta1)-L1*alpha1*cos(theta1)-omega2*omega2*L5*sin(theta5)+alpha2*L5*cos(theta5)-omega6*omega6*L6*sin(theta6)+omega7*omega7*L7*sin(theta7) ];
alpha0=C\D;
alpha6=alpha0(1);
alpha7=alpha0(2);
if alpha2<0
   alpha2=-alpha2;
end
if alpha3<0
   alpha3=-alpha3;
end
if alpha6<0
   alpha6=-alpha6;
end
if alpha7<0
   alpha7=-alpha7;
end
alpha=[alpha2;alpha3;alpha6;alpha7;alpha2];
end