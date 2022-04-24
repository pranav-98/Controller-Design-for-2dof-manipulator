
syms m1 m2 r1 r2 t1 t2 q1 q2 'real'
syms I1 I2 l1 l2 g 'real'
syms dq1 dq2 'real'
syms d2q1 d2q2 'real'
syms v1 v2 'real'
syms t 'real'
%suffix 1 is for the first link and suffix 2 is for the second link
v=[v1;v2];
%m: Mass
%r: radius
%t: torque
%q: angle theta
%dq: rate of change of angle
%d2q: Angular acceleration
%t1 and t2: torque on arm one and arm 2
%I: angular momentum
%g: acceleration due to gravity



%The Kinetic and potential energy of the first mass
K1=(1/2*m1*(r1^2)*(dq1)^2)+(1/2*I1*(dq1)^2);
V1=m1*g*r1*cos(q1);




%The velocity of the second mass
vel2=((l1^2*dq1^2)+(r2^2*(dq1+dq2)^2)+(2*l1*r2*dq1*(dq2+dq1)*cos(q2)));





%Kinetic and potential energy of the second mass
K2=(1/2*m2*(vel2))+(1/2*I2*(dq1+dq2)^2);
V2=m2*g*((l1*cos(q1))+(r2*cos(q1+q2)));

%Lagrangian function
L=K1+K2-V1-V2;


%Finding dl/dq1,dl/ddq1, dl/dq2, dl/ddq2
dL_dq1=jacobian(L, [q1; dq1]);
dL_dq2=jacobian(L,[q2; dq2]);


%Finding dl/dtddq1 and dl/dtddq2
dL_dtdq1=jacobian(dL_dq1(2),[q1;dq1;q2;dq2])*[dq1;d2q1;dq2;d2q2];
dL_dtdq2=jacobian(dL_dq2(2),[q1;dq1;q2;dq2])*[dq1;d2q1;dq2;d2q2];

%Defining the two equations
e1= dL_dtdq1-dL_dq1(1)-t1;
e2= dL_dtdq2-dL_dq2(1)-t2;




eqn=[e1;e2];


%eqn_t=subs(eqn,[dq1,dq2,d2q1,d2q2,t1,t2],[0,0,0,0,0,0])

sol=solve(eqn==0,[t1,t2])
display(sol.t1)
display(sol.t2)

%Part b Displaying the Equation in form of the Manipulator eqn
M=[(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(q2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(q2)*r2))/2);(I2 + (m2*(2*r2^2 + 2*l1*cos(q2)*r2))/2), (m2*r2^2 + I2) ];
qdd=[d2q1;d2q2];
C=[-(dq2*m2*(2*l1*r2*sin(q2)*(dq1 + dq2) + 2*dq1*l1*r2*sin(q2)))/2; dq1*l1*m2*r2*sin(q2)*(dq1 + dq2) - dq1*dq2*l1*m2*r2*sin(q2) ];
gq=[-g*m2*(r2*sin(q1 + q2) + l1*sin(q1)) - g*m1*r1*sin(q1); - g*m2*r2*sin(q1 + q2)];


% tau=[sol.t1;sol.t2]


%Eigen value placement
A=[0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B=[0,0;0,0;1,0;0,1];
lambda=[-3-0.1i,-3+0.1i, -5,-5.3];

K=place(A,B,lambda);

Kp=K(:,1:2);
Kd=K(:,3:4);


tau=[t1;t2];
tau=M*qdd+C+gq;

tau1=M*v+C+gq;
T=tau1-tau;
v=solve(T==0,v)

v=[v.v1;v.v2];


%Declaring the  trajectory related variables
qdes=[0.0063,-0.0942,0, pi; 0.0031,-0.0472,0, pi/2]*[t^3;t^2;t;1]
qdesdot=jacobian(qdes,[t])
vdes=jacobian(qdesdot,[t])
qdes1=[];
qdes2=[];
qddes1=[];
qddes2=[];
% qdes(end+1)=double(subs(qdes(1,:),0))
% qdes(end+1)=double(subs(qdes(2,:),0))

torque1=[]
torque2=[]

dtorque1=[]
dtorque2=[]

%Deriving system response for 10 seconds.
T=10;
y0=[deg2rad(200),0, deg2rad(125), 0];

[t,y]=ode45(@ode_2dof,[0,T], y0);

size(t)

%Sampling the various variables in order to plot
for i=1:size(t)
    qdes1(end+1)=double(subs(qdes(1,:),t(i,:)));
    qdes2(end+1)=double(subs(qdes(2,:),t(i,:)));
    qddes1(end+1)=double(subs(qdesdot(1,:),t(i,:)));
    qddes2(end+1)=double(subs(qdesdot(2,:),t(i,:)));
    [~,T_act]=ode_2dof(t(i),y(i));
    torque1(end+1)=T_act(1);
    torque2(end+1)=T_act(2);
    


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



subplot(3,2,6);

plot(t,torque2');
title('torque 2 vs time')











