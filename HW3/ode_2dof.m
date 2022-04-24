function [dX,T_act]=ode_2dof(t,X)
%Substituting the required values
m1=1;m2=1;l1=1;l2=1;I1=0.084;I2=0.084;g=9.8;t1=0;t2=0;
r1=0.45;r2=0.45;

%Creating a dX matrix
dX=zeros(4,1);
X=num2cell(X);


[q1,dq1,q2,dq2]=deal(X{:});

%Generating the feedback linearization
M=[(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(q2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(q2)*r2))/2);(I2 + (m2*(2*r2^2 + 2*l1*cos(q2)*r2))/2), (m2*r2^2 + I2) ];

C=[-(dq2*m2*(2*l1*r2*sin(q2)*(dq1 + dq2) + 2*dq1*l1*r2*sin(q2)))/2; dq1*l1*m2*r2*sin(q2)*(dq1 + dq2) - dq1*dq2*l1*m2*r2*sin(q2) ];
gq=[-g*m2*(r2*sin(q1 + q2) + l1*sin(q1)) - g*m1*r1*sin(q1); - g*m2*r2*sin(q1 + q2)];





%We shall now substitute the trajectory that was generated in part a
qdes=[(63*t^3)/10000 - (471*t^2)/5000 + pi;  (31*t^3)/10000 - (59*t^2)/1250 + pi/2];
qdesdot=[(189*t^2)/10000 - (471*t)/2500; (93*t^2)/10000 - (59*t)/625];
vdes=[(189*t)/5000 - 471/2500;
   (93*t)/5000 - 59/625];

A=[0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B=[0,0;0,0;1,0;0,1];
lambda=[-3-0.1i,-3+0.1i, -5,-5.3];
K=place(A,B,lambda);

%Calculating the Virtual control input 
Kp=K(:,1:2);
Kd=K(:,3:4);
q=[q1;q2];
dq=[dq1;dq2];
vact=(-Kp*(q-qdes)-Kd*(dq-qdesdot))+vdes;


q=[q1;q2];
dq=[dq1;dq2];


%Calculating the torque 
T=M*(vact)+C+gq;
T_act=T;
t1=T(1);

t2=T(2);



%Substituting the equations
%Copy pasting the found equations
dX(1)=dq1;
dX(2)=(I2*t1 - I2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) - l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(3)= dq2;
dX(4)= -(I2*t1 - I1*t2 - I2*t2 - l1^2*m2*t2 - m1*r1^2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^3*m2^2*r2*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) - g*l1^2*m2^2*r2*sin(q1 + q2) - I1*g*m2*r2*sin(q1 + q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) + l1*m2*r2*t1*cos(q2) - 2*l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + 2*dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq2^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + g*l1^2*m2^2*r2*cos(q2)*sin(q1) - g*m1*m2*r1^2*r2*sin(q1 + q2) + I1*dq1^2*l1*m2*r2*sin(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*dq1*dq2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq1^2*l1*m1*m2*r1^2*r2*sin(q2) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2) + g*l1*m1*m2*r1*r2*cos(q2)*sin(q1))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
 

end


