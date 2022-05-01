
syms m1 m2 r1 r2 t1 t2 q1 q2 'real'
syms I1 I2 l1 l2 g 'real'
syms dq1 dq2 'real'
syms d2q1 d2q2 'real'
syms v1 v2 'real'
syms t_v 'real'
syms p vr  'real'
%suffix 1 is for the first link and suffix 2 is for the second link
v=[v1;v2];
%m: Nominal Mass
%r: radius
%t: torque
%q: angle theta
%dq: rate of change of angle
%d2q: Angular acceleration
%t1 and t2: torque on arm one and arm 2
%I: Nominal angular momentum
%g: acceleration due to gravity

q=[q1;q2];

% 
% m1 = 1; %kg
% m2 = 1;
% l1 = 1; %m
% l2 = 1;
% r1 = 0.45; %m
% r2 = 0.45;
% I1 = 0.084; %kgm^2
% I2 = 0.084;
% g = 9.81;

dq=[dq1;dq2];
% where


%We shall now find the virtual control input

A=[0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B=[0,0;0,0;1,0;0,1];
eigs=[-3,-3,-4,-4];
K=place(A,B,eigs);
Kp=K(:,1:2);
Kd=K(:,3:4);

Acl=[0,0,1,0;0,0,0,1;-Kp,-Kd];

Q=eye(4).*0.7;

P=lyap(Acl',Q);

P=zeros(4,4);

qdes=[(63*t_v^3)/10000 - (471*t_v^2)/5000 + pi;  (31*t_v^3)/10000 - (59*t_v^2)/1250 + pi/2];
    qdesdot=[(189*t_v^2)/10000 - (471*t_v)/2500; (93*t_v^2)/10000 - (59*t_v)/625];
    vdes=[(189*t_v)/5000 - 471/2500; (93*t_v)/5000 - 59/625];







qdes1=[];
qdes2=[];
qddes1=[];
qddes2=[];


torque1=[];
torque2=[];


%Subsitute parameters

% m11=0.75;m21=0.75;l11=1;l21=1;I11=0.063;I21=0.063;g=9.8;
% r11=0.45;r21=0.45;


m11 = 1; %kg
m21 = 1;
l11 = 1; %m
l21 = 1;
r11 = 0.45; %m
r21 = 0.45;
I11 = 0.084; %kgm^2
I21 = 0.084;
g1 = 9.81;

alpha = [m2*l1^2+m1*r1^2+m2*r2^2 + I1 + I2; m2*l1*r2; m2*r2^2 + I2;m1*r1 + m2*l1;m2*r2];


alpha0=double(subs(alpha,[m1,m2,l1,l2,r1,r2,I1,I2,g],[m11,m21,l11,l21,r11,r21,I11,I21,g1]));

%Deriving system response for 10 seconds.
T=0:0.01:10;
y0=[deg2rad(200),0, deg2rad(125), 0, 0.75*alpha0(1), 0.75*alpha0(2),0.75*alpha0(3),0.75*alpha0(4),0.75*alpha0(5)];
%y0=[deg2rad(200),0, deg2rad(125), 0]
[t,y] = ode45(@ode_dof,T,y0);




qdes1=[];
qdes2=[];
qddes1=[];
qddes2=[];
torque1=[];
torque2=[];
tor=[];
vdes1=0;
vdes2=0;
Gmat1=0;

%Sampling the various variables in order to plot
for i=1:size(t)
    qdes1(end+1)=double(subs(qdes(1,:),t(i,:)));
    qdes2(end+1)=double(subs(qdes(2,:),t(i,:)));
    qddes1(end+1)=double(subs(qdesdot(1,:),t(i,:)));
    qddes2(end+1)=double(subs(qdesdot(2,:),t(i,:)));
    
    %We calculated the required virtual force
    vdes1=double(subs(vdes(1),t(i,:)));
    vdes2=double(subs(vdes(2),t(i,:)));
    v12=[vdes1;vdes2];
    
    %Calculate the error
    err = [y(i,1)-qdes1(i);y(i,2)-qddes1(i);y(i,3)-qdes2(i);y(i,4)-qddes2(i)];
  
    dq = [y(i,3);y(i,4)];

    %Calculate the virtual force
    V_t = ((-K) * err) + v12 ;
   
    %Calculating the adaptive parameters
    a = y(i, 5);
    b = y(i, 6);
    d = y(i, 7);
    e = y(i, 8);
    f = y(i, 9);
    M_mat= [a+2*b*cos(y(i,3)), d+b*cos(y(i,3)); d+b*cos(y(i,3)), d];
    C_mat= [-b*sin(y(i,3))*y(i,4), -b*sin(y(i,3))*(y(i,2)+y(i,4)); b*sin(y(i,3))*y(i,2),0];
    G_mat= [-e*g*sin(y(i,1))-f*g*sin(y(i,3)+y(i,1)); -f*g*sin(y(i,3)+y(i,1))];
    Gmat1=double(subs(G_mat,[g],[g1]));
    

    %Sampling the torque for plotting
    tor(:,i) = M_mat * V_t + C_mat * dq + Gmat1;
    torque1(end+1)=tor(1,i);
    torque2(end+1)=tor(2,i);
    


end



%Plotting the required trajectories
subplot(6,2,1);

plot(t,y(:,1));
title('theta1 vs time');
hold on;

plot(t,qdes1');


subplot(6,2,2);
plot(t,y(:,2));
title('dtheta1 vs time');

hold on;

plot(t,qddes1');


subplot(6,2,3);

plot(t,y(:,3));
title('theta2 vs time');
hold on;
plot(t,qdes2');


subplot(6,2,4);

plot(t,y(:,4));
title('dtheta2 vs time')

hold on;

plot(t,qddes2');

subplot(6,2,5);

plot(t,torque1');
title('torque 1 vs time')


% 
subplot(6,2,6);

plot(t,torque2');
title('torque 2 vs time')

subplot(6,2,7);

plot(t,y(:,5));
title('alpha(1) vs time')


% 
subplot(6,2,8);

plot(t,y(:,6));
title('alpha(2) vs time')


subplot(6,2,9);

plot(t,y(:,7));
title('alpha(3) vs time')


% 
subplot(6,2,10);

plot(t,y(:,8));
title('alpha(4) vs time')


subplot(6,2,11);

plot(t,y(:,9));
title('alpha(5) vs time')













