function dX=ode_2dof(t,X)
%Substituting the required values actual values.
m1=1;m2=1;l1=1;l2=1;I1=0.084;I2=0.084;g=9.8;t1=0;t2=0;
r1=0.45;r2=0.45;

%Substituting nominal values

m1_1=0.75;m2_1=0.75;I1_1=0.063;I2_1=0.063;


ro=2.5;

phi=0.0;

%Creating a dX matrix
dX=zeros(4,1);
X=num2cell(X);


[q1,dq1,q2,dq2]=deal(X{:});

%Generating the feedback linearization
a = I1_1 + I2_1 + m1_1*r1^2 + m2_1*(l1^2 + r2^2);
b = m2_1*l1*r2;
d = I2_1 + m2_1*r2^2;
Mmat= [a+2*b*cos(q2), d+b*cos(q2); d+b*cos(q2), d];
Cmat= [-b*sin(q2)*dq2, -b*sin(q2)*(dq1+dq2); b*sin(q2)*dq1,0];
Gmat= [-m1_1*g*r1*sin(q1)-m2_1*g*(l1*sin(q1)+r2*sin(q1+q2)); -m2_1*g*r2*sin(q1+q2)];
%Finding the virtual control input
A=[0,0,1,0;0,0,0,1;0,0,0,0;0,0,0,0];
B=[0,0;0,0;1,0;0,1];

%eigs=[-6,-6.4,-7,-8];
%eigs=[-3+0.03i,-4,-3-0.03i,-5];
eigs=[-3,-3,-4,-4];
%eigs=[-0.6,-0.8,-1,-1.2];
%eigs=[-2,-3,-1,-1.5];
%eigs=[-1,-1.1,-1.2,-1.3];
%eigs=[-1.8,-1.9,-1.95,-2];
K=place(A,B,eigs);
Kp=K(:,1:2);
Kd=K(:,3:4);

Acl=[0,0,1,0;0,0,0,1;-K];

Q=eye(4).*1;

P=lyap(Acl',Q);

q=[q1;q2];
dq=[dq1;dq2];


qdes=[(63*t^3)/10000 - (471*t^2)/5000 + pi;
      (31*t^3)/10000 - (59*t^2)/1250 + pi/2];

qdesdot= [(189*t^2)/10000 - (471*t)/2500;
            (93*t^2)/10000 - (59*t)/625];






vdes=[(189*t)/5000 - 471/2500;
  (93*t)/5000 - 59/625];

x=[q-qdes;dq-qdesdot]
% 
% x1=norm(x'*P*B);
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

v=vdes-(K*x)+vr';

T=Mmat*v+Cmat*dq+Gmat;



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


