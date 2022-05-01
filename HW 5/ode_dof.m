function dX=ode_dof(t,X)
%Substituting the required values actual values.


m1=1;
m2=1;
l1=1;
l2=1;
I1=0.084;
I2=0.084;
g=9.8;
t1=0;
t2=0;
r1=0.45;
r2=0.45;


dX=zeros(9,1);
X=num2cell(X);


[q1,dq1,q2,dq2,a,b,d,e,f]=deal(X{:});
%Generating the feedback linearization
%Nominal Values
M_mat= [a+2*b*cos(q2), d+b*cos(q2); d+b*cos(q2), d];
C_mat= [-b*sin(q2)*dq2, -b*sin(q2)*(dq1+dq2); b*sin(q2)*dq1,0];
G_mat= [-e*g*sin(q1)-f*g*sin(q2+q1); -f*g*sin(q2+q1)];

%Actual values
a_hat = I1 + I2 + m1*r1^2 + m1*(l1^2 + r2^2);
b_hat = m2*l1*r2;
d_hat = I2 + m2*r2^2;
M = [a_hat+2*b_hat*cos(q2), d_hat+b_hat*cos(q2); d_hat+b_hat*cos(q2), d_hat];
C = [-b_hat*sin(q2)*dq2, -b_hat*sin(q2)*(dq1+dq2); b_hat*sin(q2)*dq1,0];
G = [-m1*g*r1*sin(q1)-m2*g*(l1*sin(q1)+r2*sin(q1+q2)); -m2*g*r2*sin(q1+q2)];


%Finding the virtual control input
A=[0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B=[0,0;0,0;1,0;0,1];
eigs=[-3,-3,-4,-4];
K=place(A,B,eigs);
Kp=K(:,1:2);
Kd=K(:,3:4);
Acl=[0,0,1,0;0,0,0,1;-K];
Q=eye(4).*0.7;
P=lyap(Acl',Q);

P=zeros(4,4);

q=[q1;q2];
dq=[dq1;dq2];


qdes=[(63*t^3)/10000 - (471*t^2)/5000 + pi;
      (31*t^3)/10000 - (59*t^2)/1250 + pi/2];

qdesdot= [(189*t^2)/10000 - (471*t)/2500;
            (93*t^2)/10000 - (59*t)/625];






vdes=[(189*t)/5000 - 471/2500;
  (93*t)/5000 - 59/625];

x=[q-qdes;dq-qdesdot];

v=vdes-(K*x);
T=M_mat*v+C_mat*dq+G_mat;



T_act=T;
t1=T(1);
t2=T(2);



%Substituting the equations
%Copy pasting the found equations
dX(1)=dq1;
dX(2)=(I2*t1 - I2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) - l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(3)= dq2;
dX(4)= -(I2*t1 - I1*t2 - I2*t2 - l1^2*m2*t2 - m1*r1^2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^3*m2^2*r2*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) - g*l1^2*m2^2*r2*sin(q1 + q2) - I1*g*m2*r2*sin(q1 + q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) + l1*m2*r2*t1*cos(q2) - 2*l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + 2*dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq2^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + g*l1^2*m2^2*r2*cos(q2)*sin(q1) - g*m1*m2*r1^2*r2*sin(q1 + q2) + I1*dq1^2*l1*m2*r2*sin(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*dq1*dq2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq1^2*l1*m1*m2*r1^2*r2*sin(q2) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2) + g*l1*m1*m2*r1*r2*cos(q2)*sin(q1))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);


%Designing the regressor
ddq=inv(M)*(T-C*dq-G);
ddq1=ddq(1);
ddq2=ddq(2);
B=[0,0;0,0;1,0;0,1];
%Here we shall include the calculations for the Adaptive control law
Y = [ddq1, cos(q2)*(2*ddq1 + ddq2) - 2*sin(q2)*dq1*dq2 - sin(q2)*dq2^2,  ddq2, -sin(q1)*g, -sin(q1 + q2)*g; 0, sin(q2)*dq1^2 + cos(q2)*ddq1, ddq1 + ddq2, 0, -sin(q1+q2)*g];

alpha = [m2*l1^2 + m1*r1^2 + m2*r2^2 + I1 + I2; m2*l1*r2; m2*r2^2 + I2;m1*r1 + m2*l1;m2*r2];
Gamma=eye(5).*0.4;

phi=M_mat\Y;

dq_p=-Gamma\(phi'*B'*P*x);
% 
dX(5)=dq_p(1);
dX(6)=dq_p(2);
dX(7)=dq_p(3);
dX(8)=dq_p(4);
dX(9)=dq_p(5);


end


