clear; close; clc;
% ROS Setup
rosinit;
j1_effort = rospublisher('/rrbot/joint1_effort_controller/command');
j2_effort = rospublisher('/rrbot/joint2_effort_controller/command');
JointStates = rossubscriber('/rrbot/joint_states');
tau1 = rosmessage(j1_effort);
tau2 = rosmessage(j2_effort);
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
client = rossvcclient('/gazebo/set_model_configuration');
req = rosmessage(client);
req.ModelName = 'rrbot';
req.UrdfParamName = 'robot_description';
req.JointNames = {'joint1','joint2'};
req.JointPositions = [deg2rad(200), deg2rad(125)];
resp = call(client,req,'Timeout',3);
tic;
t = 0;

%Initializing matrices to plot
T=[];
theta1=[];
theta2=[];
Tau1=[]
Tau2=[]
d1=[]
d2=[]
qdes1=[]
qdes2=[]
qddes1=[]
qddes2=[]
alpha1=[]
alpha2=[]
alpha3=[]
alpha3=[]
alpha4=[]
alpha5=[]

%True values
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
alpha =0.75* [m2*l1^2+m1*r1^2+m2*r2^2 + I1 + I2; m2*l1*r2; m2*r2^2 + I2;m1*r1 + m2*l1;m2*r2]
prev=0
%Implementing the controller
while(t < 10)
    t = toc;
    qdes=[(63*t^3)/10000 - (471*t^2)/5000 + pi;  (31*t^3)/10000 - (59*t^2)/1250 + pi/2];
    qdesdot=[(189*t^2)/10000 - (471*t)/2500; (93*t^2)/10000 - (59*t)/625];
    vdes=[(189*t)/5000 - 471/2500; (93*t)/5000 - 59/625];
    qdes1(end+1)=qdes(1);
    qddes1(end+1)=qdesdot(1);
    qdes2(end+1)=qdes(2);
    qddes2(end+1)=qdesdot(2);
    
    jointData = receive(JointStates);
    
    q1=wrapTo2Pi(jointData.Position(1));
    dq1=jointData.Velocity(1);
    q2=wrapTo2Pi(jointData.Position(2));
    dq2=jointData.Velocity(2);

    q=[q1;q2];
    dq=[dq1;dq2];
    
    

    a = alpha(1);
    b = alpha(2);
    d = alpha(3);
    e=alpha(4);
    f=alpha(5)
    Mmat= [a+2*b*cos(q2), d+b*cos(q2); d+b*cos(q2), d];
    Cmat= [-b*sin(q2)*dq2, -b*sin(q2)*(dq1+dq2); b*sin(q2)*dq1,0];
    Gmat= [-e*g*sin(q1)-f*g*sin(q2+q1); -f*g*sin(q2+q1)];
    
    %Using the Gain matrix found in the previous parts
    K=[12.000000000000009,0,7.000000000000003,0;0,12.000000000000009,0,7.000000000000003];
    %Calculating the virtual control input
    Acl=[0,0,1,0;0,0,0,1;-K];
    B=[0,0;0,0;1,0;0,1];
    
    
    Q=eye(4).*0.7;

    P=lyap(Acl',Q);

    x=[q-qdes;dq-qdesdot];
    

    vact=(-K*x)+vdes;


    
    T1=Mmat*vact+Cmat*dq+Gmat;
    if abs(T1(1))>20
        T1(1)=0;
    end

    if abs(T1(2))>10
        T1(2)=0;
    end

    
    %Applying the torque obtained from feedback linearization
    tau1.Data = T1(1);
    tau2.Data = T1(2);
    send(j1_effort,tau1);
    send(j2_effort,tau2);


    %Calculate the actual parameters to update the adaptive law
    a_hat = I1 + I2 + m1*r1^2 + m1*(l1^2 + r2^2);
    b_hat = m2*l1*r2;
    d_hat = I2 + m2*r2^2;
    M = [a_hat+2*b_hat*cos(q2), d_hat+b_hat*cos(q2); d_hat+b_hat*cos(q2), d_hat];
    C = [-b_hat*sin(q2)*dq2, -b_hat*sin(q2)*(dq1+dq2); b_hat*sin(q2)*dq1,0];
    G = [-m1*g*r1*sin(q1)-m2*g*(l1*sin(q1)+r2*sin(q1+q2)); -m2*g*r2*sin(q1+q2)];



    ddq_a=inv(M)*(T1-C*dq-G);
    ddq1=ddq_a(1);
    ddq2=ddq_a(2);
    Y = [ddq1, cos(q2)*(2*ddq1 + ddq2) - 2*sin(q2)*dq1*dq2 - sin(q2)*dq2^2,  ddq2, -sin(q1)*g, -sin(q1 + q2)*g; 0, sin(q2)*dq1^2 + cos(q2)*ddq1, ddq1 + ddq2, 0, -sin(q1+q2)*g];

    
    Gamma=eye(5).*0.4;

    phi=Mmat\Y;

    dq_p=-Gamma\(phi'*B'*P*x);
% 
    alpha=alpha+dq_p*(t-prev);
    prev=t
    % Sampling variables to plot
    T(end+1)=t;
    theta1(end+1)=q(1);
    d1(end+1)=dq(1);
    d2(end+1)=dq(2);
    theta2(end+1)=q(2);
    Tau1(end+1)=T1(1);
    Tau2(end+1)=T1(2);
    alpha1(end+1)=alpha(1);
    alpha2(end+1)=alpha(2);
    alpha3(end+1)=alpha(3);
    alpha4(end+1)=alpha(4);
    alpha5(end+1)=alpha(5);
    
    
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);


subplot(6,2,1);

plot(T,theta1);
title('theta1 vs time');
hold on;

plot(T,qdes1)

subplot(6,2,2);
plot(T,theta2);
title('theta2 vs time');
hold on;

plot(T,qdes2)


subplot(6,2,3);

plot(T,d1);
title('dtheta1 vs time');
hold on;

plot(T,qddes1)

subplot(6,2,4);
plot(T,d2);
title('dtheta2 vs time');
hold on;

plot(T,qddes2)

subplot(6,2,5);
plot(T,Tau1);
title('T1 vs time');

subplot(6,2,6);
plot(T,Tau2);

title('T2 vs time');


subplot(6,2,7);
plot(T,alpha1);
title('alpha(1) vs time');

subplot(6,2,8);
plot(T,alpha2);
title('alpha(2) vs time');

subplot(6,2,9);
plot(T,alpha3);
title('alpha(3) vs time');

subplot(6,2,10);
plot(T,alpha4);
title('alpha(4) vs time');

subplot(6,2,11);
plot(T,alpha5);
title('alpha(5) vs time');




%Terminating connection between MATLAB and ROS
rosshutdown;
