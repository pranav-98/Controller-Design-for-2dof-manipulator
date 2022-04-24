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

%Declaring the parameters of the system
m1=1;m2=1;l1=1;l2=1;I1=0.084;I2=0.084;g=9.8;t1=0;t2=0;
r1=0.45;r2=0.45;

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

    M=[(m1*r1^2 + I1 + I2 + (m2*(2*l1^2 + 4*cos(q2)*l1*r2 + 2*r2^2))/2), (I2 + (m2*(2*r2^2 + 2*l1*cos(q2)*r2))/2);(I2 + (m2*(2*r2^2 + 2*l1*cos(q2)*r2))/2), (m2*r2^2 + I2) ];

    C=[-(dq2*m2*(2*l1*r2*sin(q2)*(dq1 + dq2) + 2*dq1*l1*r2*sin(q2)))/2; dq1*l1*m2*r2*sin(q2)*(dq1 + dq2) - dq1*dq2*l1*m2*r2*sin(q2) ];
    gq=[-g*m2*(r2*sin(q1 + q2) + l1*sin(q1)) - g*m1*r1*sin(q1); - g*m2*r2*sin(q1 + q2)];
    
    %Using the Gain matrix found in the previous parts
    Kp= [15.038099010691662,-0.462344511474929;0.570598815831362,15.859796281410224];
    Kd= [8.007585607417154,-0.087234810372718;0.113332820586433,8.292414392582849];
    
    %Calculating the virtual control input
    vact=(-Kp*(q-qdes)-Kd*(dq-qdesdot))+vdes;


    
    T1=M*vact+C+gq;
   
    
    %Applying the torque obtained from feedback linearization
    tau1.Data = T1(1);
    tau2.Data = T1(2);
    send(j1_effort,tau1);
    send(j2_effort,tau2);

    % Sampling variables to plot
    T(end+1)=t;
    theta1(end+1)=q(1);
    d1(end+1)=dq(1);
    d2(end+1)=dq(2);
    theta2(end+1)=q(2);
    Tau1(end+1)=T1(1);
    Tau2(end+1)=T1(2);
   
    
    
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);


subplot(3,2,1);

plot(T,theta1);
title('theta1 vs time');
hold on;

plot(T,qdes1)

subplot(3,2,2);
plot(T,theta2);
title('theta2 vs time');
hold on;

plot(T,qdes2)


subplot(3,2,3);

plot(T,d1);
title('dtheta1 vs time');
hold on;

plot(T,qddes1)

subplot(3,2,4);
plot(T,d2);
title('dtheta2 vs time');
hold on;

plot(T,qddes2)

subplot(3,2,5);
plot(T,Tau1);
title('T1 vs time');

subplot(3,2,6);
plot(T,Tau2);
title('T2 vs time');



%Terminating connection between MATLAB and ROS
rosshutdown;
