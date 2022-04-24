
syms m1 m2 r1 r2 t1 t2 q1 q2 'real'
syms I1 I2 l1 l2 g 'real'
syms dq1 dq2 'real'
syms d2q1 d2q2 'real'
%suffix 1 is for the first link and suffix 2 is for the second link

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
eqn1= dL_dtdq1-dL_dq1(1)-t1
eqn2= dL_dtdq2-dL_dq2(1)-t2


%Solving the equations
sol=solve([eqn1==0,eqn2==0],[d2q1,d2q2]);


%Displaying the equations found


disp(sol.d2q1)
disp(sol.d2q2)

%Defining state space variable
% X=sym('X',[4,1]);
% X(1)=q1;
% X(2)=dq1;
% X(3)=q2;
% X(4)=dq2;
% 
% 
% 
% %State space equation for the 2dof revolute joint is given by
% dX(1,1)= dq1;
% dX(2,1)= sol.d2q1;
% dX(3,1)= dq2;
% dX(4,1)= sol.d2q2;


%Plotting the response
% T=10;
% y0=[deg2rad(200),0, deg2rad(125), 0];

% [t,y]=ode45(@ode_2dof,[0,T], y0);
% 
% 
% subplot(2,2,1);
% 
% plot(t,y(:,1));
% title('theta1 vs time');
% 
% subplot(2,2,2);
% plot(t,y(:,2));
% title('dtheta1 vs time');
% 
% subplot(2,2,3);
% 
% plot(t,y(:,3));
% title('thet
%suffix 1 is for the first link and suffix 2 is for the second link

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
eqn1= dL_dtdq1-dL_dq1(1)-t1
eqn2= dL_dtdq2-dL_dq2(1)-t2


%Solving the equations
sol=solve([eqn1==0,eqn2==0],[d2q1,d2q2]);


%Displaying the equations found


disp(sol.d2q1)
disp(sol.d2q2)

%Defining state space variable
% X=sym('X',[4,1]);
% X(1)=q1;
% X(2)=dq1;
% X(3)=q2;
% X(4)=dq2;
% 
% 
% 
% %State space equation for the 2dof revolute joint is given by
% dX(1,1)= dq1;
% dX(2,1)= sol.d2q1;
% dX(3,1)= dq2;
% dX(4,1)= sol.d2q2;


%Plotting the response
T=10;
y0=[deg2rad(30),deg2rad(45), 0, 0];
K=[195,35,30,15;50,12.23,18,4.12]
[t,y]=ode45(@ode_2dof,[0,T], y0);

U=-K*y'
subplot(3,2,1);

plot(t,y(:,1));
title('theta1 vs time');

subplot(3,2,2);
plot(t,y(:,2));
title('dtheta1 vs time');

subplot(3,2,3);

plot(t,y(:,3));
title('theta2 vs time');

subplot(3,2,4);

plot(t,y(:,4));
title('dtheta2 vs time');

subplot(3,2,4);

plot(t,y(:,4));
title('dtheta2 vs time');

subplot(3,2,5)
plot(t, U(1,:))

subplot(3,2,6)
plot(t, U(1,2))