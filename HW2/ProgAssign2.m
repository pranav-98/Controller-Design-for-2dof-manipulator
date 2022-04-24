
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
e1= dL_dtdq1-dL_dq1(1)-t1
e2= dL_dtdq2-dL_dq2(1)-t2




eqn=[e1;e2];


eqn_t=subs(eqn,[dq1,dq2,d2q1,d2q2,t1,t2],[0,0,0,0,0,0])

sol=solve(eqn_t==0,[q1,q2])
display(sol.q1);
display(sol.q2);



%Part(b)

dX(1)=dq1;
dX(2)=(I2*t1 - I2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) - l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(3)= dq2;
dX(4)= -(I2*t1 - I1*t2 - I2*t2 - l1^2*m2*t2 - m1*r1^2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^3*m2^2*r2*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) - g*l1^2*m2^2*r2*sin(q1 + q2) - I1*g*m2*r2*sin(q1 + q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) + l1*m2*r2*t1*cos(q2) - 2*l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + 2*dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq2^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + g*l1^2*m2^2*r2*cos(q2)*sin(q1) - g*m1*m2*r1^2*r2*sin(q1 + q2) + I1*dq1^2*l1*m2*r2*sin(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*dq1*dq2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq1^2*l1*m1*m2*r1^2*r2*sin(q2) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2) + g*l1*m1*m2*r1*r2*cos(q2)*sin(q1))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
 
eq=[dX(1);dX(2);dX(3);dX(4)]


A=jacobian(eq,[q1;dq1;q2;dq2]);
A=subs(A,[dq1,dq2,d2q1,d2q2],[0,0,0,0]);
A=subs(A,[m1,m2,r1,r2,I1,I2,l1,l2,g],[1,1,0.45,0.45,0.084,0.084,1,1,9.8]);
A1=subs(A,[q1,q2],[sol.q1(2),sol.q2(2)]);
A2=subs(A,[q1,q2],[sol.q1(3),sol.q2(3)]);
A=subs(A,[q1,q2],[sol.q1(1),sol.q2(1)]);


A=double(A)


 
B=jacobian(eq,[t1;t2]);
B=subs(B,[dq1,dq2,d2q1,d2q2],[0,0,0,0]);
B=subs(B,[m1,m2,r1,r2,I1,I2,l1,l2,g],[1,1,0.45,0.45,0.084,0.084,1,1,9.8]);
B=subs(B,[q1,q2],[sol.q1(1),sol.q2(1)]);

B=double(B)



%Part d
CO=rank(ctrb(A,B))





%part e
eigen=[-2,-2.005-3i, -2.005+3i,-1.8];

K=place(A,B,eigen)


%Plotting the ode
T=10;
y0=[deg2rad(30),0, deg2rad(45), 0];

[t,y]=ode45(@ode_2dof,[0,T], y0);


U=-K*y';

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
title('dtheta2 vs time')

subplot(3,2,5);

plot(t,U(1,:));
title('U1 vs time')

subplot(3,2,6);

plot(t,U(2,:));
title('U2 vs time')




%part c

disp('The equilibrium points are')
disp([sol.q1,sol.q2])

%A matrix for [0,0]
disp('A matrix for [0,0]')
disp(A)

disp('eigen values are')
disp(eig(A))

%A matrix for [pi,0]
disp('A matrix for [pi,0]')
disp(double(A1))

disp('eigen values are')
disp(eig(double(A1)))

%A matrix for [0,pi]
disp('A matrix for [0,pi]')
disp(double(A2))

disp('eigen values are')
disp(eig(double(A2)))







