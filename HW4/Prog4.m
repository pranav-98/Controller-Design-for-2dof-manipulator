
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

dq=[dq1;dq2];
% where
a = I1 + I2 + m1*r1^2 + m2*(l1^2 + r2^2);
b = m2*l1*r2;
d = I2 + m2*r2^2;



Mmat= [a+2*b*cos(q2), d+b*cos(q2); d+b*cos(q2), d];
Cmat= [-b*sin(q2)*dq2, -b*sin(q2)*(dq1+dq2); b*sin(q2)*dq1,0];
Gmat= [-m1*g*r1*sin(q1)-m2*g*(l1*sin(q1)+r2*sin(q1+q2)); -m2*g*r2*sin(q1+q2)];


%We shall now find the virtual control input

A=[0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B=[0,0;0,0;1,0;0,1];
eigs=[-3,-3,-4,-4];
K=place(A,B,eigs);
Kp=K(:,1:2);
Kd=K(:,3:4);

Acl=[0,0,1,0;0,0,0,1;-Kp,-Kd];

Q=eye(4).*1;

P=lyap(Acl',Q);



qdes=[(63*t_v^3)/10000 - (471*t_v^2)/5000 + pi;  (31*t_v^3)/10000 - (59*t_v^2)/1250 + pi/2];
    qdesdot=[(189*t_v^2)/10000 - (471*t_v)/2500; (93*t_v^2)/10000 - (59*t_v)/625];
    vdes=[(189*t_v)/5000 - 471/2500; (93*t_v)/5000 - 59/625];






ro=2.5;
%phi=0.1;
phi=0;

x=[q-qdes;dq-qdesdot]




qdes1=[];
qdes2=[];
qddes1=[];
qddes2=[];


torque1=[];
torque2=[];


%Subsitute parameters

m11=0.75;m21=0.75;l11=1;l21=1;I11=0.063;I21=0.063;g=9.8;
r11=0.45;r21=0.45;

%Deriving system response for 10 seconds.
T=0:0.01:10;
y0=[deg2rad(200),0, deg2rad(125), 0];

[t,y]=ode45(@ode_2dof,T, y0);

size(y)
size(t)



%Sampling the various variables in order to plot
for i=1:size(t)
    qdes1(end+1)=double(subs(qdes(1,:),t(i,:)));
    qdes2(end+1)=double(subs(qdes(2,:),t(i,:)));
    qddes1(end+1)=double(subs(qdesdot(1,:),t(i,:)));
    qddes2(end+1)=double(subs(qdesdot(2,:),t(i,:)));
   
%     x=[y(i,1)-qdes1(i);y(i,3)-qdes2(i);y(i,2)-qddes1(i);y(i,4)-qddes2(i)];
%     x1=norm(x'*P*B);
% 
% if phi>0
%     disp('Exec 2')
%     if x1>phi
%         vr=-ro*(x'*P*B)/x1;
%     else
%         vr=-ro*(x'*P*B)/phi;
%     end
% else
%     disp("Exec1")
%     if x1~=0
%         vr=-ro*(x'*P*B)/x1;
%     else
%         vr=[0,0];
%     end
%     
% end

vr=[0,0];

v=vdes-Kp*(q-qdes)-Kd*(dq-qdesdot)+vr';

To=Mmat*v+Cmat*dq+Gmat;

To(1)=subs(To(1));
To(2)=subs(To(2));
To(1)=subs(To(1),[m1,m2,r1,r2,I1,I2,l1,l2,p],[m11,m21,r11,r21,I11,I21,l11,l21,ro]);
To(2)=subs(To(2),[m1,m2,r1,r2,I1,I2,l1,l2,p],[m11,m21,r11,r21,I11,I21,l11,l21,ro]);



   torque1(end+1) =double(subs(To(1),[q1,q2,dq1,dq2,t_v],[y(i,1),y(i,3),y(i,2),y(i,4),t(i)]));
   torque2(end+1) =double(subs(To(2),[q1,q2,dq1,dq2,t_v],[y(i,1),y(i,3),y(i,2),y(i,4),t(i)]));
%     


end



%Plotting the required trajectories
subplot(3,2,1);

plot(t,y(:,1));
title('theta1 vs time');
hold on;

plot(t,qdes1');


subplot(3,2,2);
plot(t,y(:,2));
title('dtheta1 vs time');

hold on;

plot(t,qddes1');


subplot(3,2,3);

plot(t,y(:,3));
title('theta2 vs time');
hold on;
plot(t,qdes2');


subplot(3,2,4);

plot(t,y(:,4));
title('dtheta2 vs time')

hold on;

plot(t,qddes2');

subplot(3,2,5);

plot(t,torque1');
title('torque 1 vs time')


% 
subplot(3,2,6);

plot(t,torque2');
title('torque 2 vs time')











