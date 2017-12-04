% https://nl.mathworks.com/help/robotics/examples/work-with-rosbag-logfiles.html
% https://nl.mathworks.com/help/robotics/ug/ros-log-files-rosbags.html
% https://nl.mathworks.com/help/matlab/ref/timeseries.synchronize.html   
% 2 timeseries synchronizen
 

%% Load an example file using the rosbag command. 

% Specify the path to the bag file either as an absolute or relative path:

% filepath = fullfile(fileparts(which('ROSWorkingWithRosbagsExample')), 'data', 'ex_multiple_topics.bag');
% bag = rosbag(filepath) ;   % Dit is een voorbeeld bag van Matlab. 

filePath = fullfile('D:\Users\Eigenaar\Desktop\TuDelft\BEP\Database\Nieuwe groep\drive-download-20171129T142713Z-001','2017-11-29-14-20-10.bag');
bag = rosbag(filePath);

%% Bag Selection

% bag.AvailableTopics    % see available topics
% bag.MessageList        % see all messages 

 
start = bag.StartTime ;   


% list of messages that were recorded within the first 10 seconds
% of the rosbag and published on the /imu/data topic
                          
%bagselect1 = select(bag, 'Time', [start start + 10], 'Topic', '/wheels');                          
bagselect2 = select(bag, 'Time', [start start + 10], 'Topic', '/imu/data') ;
%bagselect3 = select(bag, 'Time', [start start + 10], 'Topic','/cmd_vel'); 
bagselect4 = select(bag, 'Time', [start start + 10], 'Topic','/imu/data_raw');
 
% bagselect3 = select(bagselect2, 'Time', [205 206]) ; % narrow down time window even further

%% Message Selection
 
%msgs = readMessages(bagselect2);     % Read the message from bagselect2
%msgs1= readMessages(bagselect1);
%msgs2= readMessages(bagselect3);
%% Extract Message Data as Time Series

format long



A = timeseries(bagselect2, 'LinearAcceleration.X','LinearAcceleration.Y','AngularVelocity.Z') ; % in de vorm: [u v r]
%B = timeseries(bagselect1, 'Transform.Rotation.X','Transform.Rotation.Y','Transform.Rotation.Z','Transform.Rotation.W'); % in de vorm: [w1 w2 w3 w4]
%D = timeseries(bagselect3, 'Angular.Z');
E = timeseries(bagselect4, 'LinearAcceleration.X','LinearAcceleration.Y','AngularVelocity.Z') ;

A.data ; % shows the data from A (IMU)
%B.data ; % shows the data from B (Pulsentellers)
%D.data ; % shows the data from D (steering angle)
E.data;

%  [A B]=synchronize(A,B,'union')
%[A B]=synchronize(A,B,'Uniform','Interval',0.005606023869739457);  % synchronize the two timeseries
%   
% [A D]=synchronize(A,D,'union');
% [B D]=synchronize(B,D,'union');
[A E]=synchronize(A,E,'union');
%  
% C=[A.time A.data B.data];
% 
% 
% t=C(:,1);
% u_d=C(:,2);
% v_d=C(:,3);
% r_d=C(:,4);

% wf=C(:,5);
% wr=C(:,7);
% del=C(:,9);

t=A.time;
delta_t=(t(end,1)-t(1,1))./(length(A.data)-1);

A1=cumtrapz(A.data); % integrate matrix C (cumulative trapezoidal integration)
A2=A1*delta_t; % multiply by delta t``
v0=zeros(length(A.data),3); % set beginvelocities for u, v and r to 0
velocity=v0+A2;

u=velocity(:,1);       % Longitudinal velocity.  
v=velocity(:,2);       % Lateral velocity
r=velocity(:,3);   %  Yaw rate

plot(A.data(:,1))
hold on 
plot(E.data(:,1))
legend('imu','imu_raw')
axis([0 1800 0 10])

figure 
plot(u)

figure
X=gradient(A.data(:,1));
plot(X)
