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
req.JointPositions = [deg2rad(30), deg2rad(45)];
resp = call(client,req,'Timeout',3);
tic;
t = 0;
T=[];
theta1=[];
theta2=[];
Tau1=[]
Tau2=[]
d1=[]
d2=[]
while(t < 10)
    t = toc;
    
    jointData = receive(JointStates);

    

   
    
    state = [wrapTo2Pi(jointData.Position(1));jointData.Velocity(1);wrapTo2Pi(jointData.Position(2));jointData.Velocity(2)];
    disp(state);
    
    
    K=[34.254141341753330,12.865616559649403,2.913360290274793,0.787799341034702;10.196405080425984,4.441605507953978,4.353998110840890,0.591411200744281]
    %K=[93.096587283626830,26.345786123342243,-18.397634684147633,-2.070276499720595;27.286683289087982,7.991047003316968,-1.079337255553084,-0.012882126340002]
    %K=[72.596410966795290,23.381184776850723,3.268435209452039,3.330374705906696;20.967135073756776,7.084077716772805,5.844453702656944,1.668862725732194]
    %K=[91.295300272185020,25.962567417421590,-22.518309232406104,-3.234225692858455;27.205628595775686,7.973154992523511,-2.162525867594412,-0.320457495304477]
    %K=[90.944755578892980,25.902484760388734,-22.692386273779256,-3.265680149395611;27.227429389190508,7.982386202336261,-2.182097525451774,-0.320115295536025]
    %264861026290,22.948015571689567,28.584027771226243,8.473049349750653;20.253375116033250,6.830724880979037,13.250310748896567,3.175166198804432];
    tau1.Data = -K(1,:)*state;
    tau2.Data = -K(2,:)*state;
    send(j1_effort,tau1);
    send(j2_effort,tau2);

    T(end+1)=t;
    theta1(end+1)=state(1);
    d1(end+1)=state(2);
    d2(end+1)=state(4);
    theta2(end+1)=state(3);
    Tau1(end+1)= -K(1,:)*state;
    Tau2(end+1)= -K(2,:)*state;
    % you can sample data here to be plotted at the end
end
tau1.Data = 0;
tau2.Data = 0;
send(j1_effort,tau1);
send(j2_effort,tau2);
subplot(3,2,1);

plot(T,theta1);
title('theta1 vs time');

subplot(3,2,2);
plot(T,theta2);
title('theta2 vs time');

subplot(3,2,3);

plot(T,Tau1);
title('torque1 vs time');

subplot(3,2,4);
plot(T,Tau2);
title('torque2 vs time');

subplot(3,2,5);

plot(T,d1);
title('dtheta1 vs time');

subplot(3,2,6);
plot(T,d2);
title('dtheta2 vs time');


rosshutdown;
