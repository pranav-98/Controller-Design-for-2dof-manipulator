function dX=ode_2dof(t,X)
%Substituting the required values
m1=1;m2=1;l1=1;l2=1;I1=0.084;I2=0.084;g=9.8;t1=0;t2=0;
r1=0.45;r2=0.45;

%Creating a dX matrix
dX=zeros(4,1);
X=num2cell(X);


[q1,dq1,q2,dq2]=deal(X{:});



%Copy pasting the K obtained from the state feedback
K=[27.601403248152714,9.436701624076361,6.902035216967677,2.719017608483835;7.073885779410277,2.804942889705140,5.416085606848654,1.076042803424326];
%Implementing state feedback
T=-K*[q1;dq1;q2;dq2];

t1=T(1);

t2=T(2);



%Substituting the equations
%Copy pasting the found equations
dX(1)=dq1;
dX(2)=(I2*t1 - I2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) - l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
dX(3)= dq2;
dX(4)= -(I2*t1 - I1*t2 - I2*t2 - l1^2*m2*t2 - m1*r1^2*t2 + m2*r2^2*t1 - m2*r2^2*t2 + dq1^2*l1*m2^2*r2^3*sin(q2) + dq1^2*l1^3*m2^2*r2*sin(q2) + dq2^2*l1*m2^2*r2^3*sin(q2) - g*l1^2*m2^2*r2*sin(q1 + q2) - I1*g*m2*r2*sin(q1 + q2) + g*l1*m2^2*r2^2*sin(q1) + I2*g*l1*m2*sin(q1) + I2*g*m1*r1*sin(q1) + l1*m2*r2*t1*cos(q2) - 2*l1*m2*r2*t2*cos(q2) + 2*dq1*dq2*l1*m2^2*r2^3*sin(q2) + 2*dq1^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq2^2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) - g*l1*m2^2*r2^2*sin(q1 + q2)*cos(q2) + g*l1^2*m2^2*r2*cos(q2)*sin(q1) - g*m1*m2*r1^2*r2*sin(q1 + q2) + I1*dq1^2*l1*m2*r2*sin(q2) + I2*dq1^2*l1*m2*r2*sin(q2) + I2*dq2^2*l1*m2*r2*sin(q2) + g*m1*m2*r1*r2^2*sin(q1) + 2*dq1*dq2*l1^2*m2^2*r2^2*cos(q2)*sin(q2) + dq1^2*l1*m1*m2*r1^2*r2*sin(q2) + 2*I2*dq1*dq2*l1*m2*r2*sin(q2) + g*l1*m1*m2*r1*r2*cos(q2)*sin(q1))/(- l1^2*m2^2*r2^2*cos(q2)^2 + l1^2*m2^2*r2^2 + I2*l1^2*m2 + m1*m2*r1^2*r2^2 + I1*m2*r2^2 + I2*m1*r1^2 + I1*I2);
 

end


