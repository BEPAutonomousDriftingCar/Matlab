
%% Description
% DataAnalyse reads experimental test data from the MotionCapture system
% and the HallEffect sensors from the test rig. It uses several functions
% to trim and edit these data matrices into and evenly timescaled Matrix,
% with the complete set of motion and force values over time, in the order:
%      1  2  3  4   5    6    7   8   9   10   11 12 13  14   15   16   17   18  19  20  21
% S = [t, v, u, r, v_d, u_d, r_d, wf, wr, del, x, y, th, Fxf, Fxr, Fyf, Fyr, kf, kr, af, ar]


%% Sources

% https://nl.mathworks.com/help/robotics/examples/work-with-rosbag-logfiles.html
% https://nl.mathworks.com/help/robotics/ug/ros-log-files-rosbags.html
% https://nl.mathworks.com/help/matlab/ref/timeseries.synchronize.html   

%% Parameters
m= 3.2 ;% kg
L= 0.334 ;% m
lf=0.167 ;% m
lr=0.167; % m
I=0.048875;
%% Import Bag

% Specify the path to the bag file either as an absolute or relative path:
filePath = fullfile('NoFilter','recht2403mms.bag');
bag = rosbag(filePath);



%% Bag Selection

% bag.AvailableTopics    % see available topics
% bag.MessageList        % see all messages 

 
start = bag.StartTime ;   


% list of messages that were recorded within the first 10 seconds
% of the rosbag and published on the /imu/data topic
                          
bagselect1 = select(bag, 'Time', [start start + 10], 'Topic', '/wheels');                          
bagselect2 = select(bag, 'Time', [start start + 10], 'Topic', '/imu/data_raw') ;
bagselect3 = select(bag, 'Time', [start start + 10], 'Topic','/DonutDevice/ground_pose'); 
% bagselect4 = select(bag, 'Time', [start start + 10], 'Topic','/cmd_vel');

 

%% Message Selection
 
%msgs = readMessages(bagselect2);     % Read the message from bagselect2
%msgs1= readMessages(bagselect1);
%msgs2= readMessages(bagselect3);
%% Extract Message Data as Time Series

format long



 A = timeseries(bagselect2, 'LinearAcceleration.X','LinearAcceleration.Y','AngularVelocity.Z') ; % in de vorm: [u v r]
 B = timeseries(bagselect1, 'Transform.Rotation.X','Transform.Rotation.Y','Transform.Rotation.Z','Transform.Rotation.W'); % in de vorm: [w1 w2 w3 w4]
%D = timeseries(bagselect4, 'Angular.Z');
 E = timeseries(bagselect3, 'Pose.Position.X','Pose.Position.Y','Pose.Orientation.Z') ;

%Filters IMU Data Using a ButterworthFilter
Filter;
HoekSnelheid;

 
 [A, B]=synchronize(A,B,'union');
 [A, E] = synchronize(A,E, 'union');
 [B, E]=synchronize(B,E, 'union');
% [A B]=synchronize(A,B,'Uniform','Interval',0.005606023869739457);  % synchronize the two timeseries
% [A D]=synchronize(A,D,'union');
% [B D]=synchronize(B,D,'union');
% [D E]=synchronize(D,E,'union');


A.data ; % shows the data from A (IMU)
B.data ; % shows the data from B (Pulsentellers)
% D.data ; % shows the data from D (steering angle)
 
C=[A.time A.data B.data E.data];


t=C(:,1);   % load time data
v_d=C(:,2); % load the IMU data, longitudinal acceleration
u_d=C(:,3); % load the IMU data, lateral acceleration
%r_d=C(:,4); % load the IMU data, r_dot yaw acceleration
r_d=zeros(length(v_d),1);

 wf=C(:,5); %+C(:,6))/2; % load the Pulsenteller data, frontwheels
wr=(C(:,7)+C(:,8))/2;  % load the Pulsenteller data, rearwheels 
%del=C(:,9);  % load the Servo data, steering angle: del
del=zeros(length(v_d),1);

total_t=t(end,1)-t(1,1);
delta_t=(t(end,1)-t(1,1))./(length(A.data)-1); % time between sampling points (delta t)



%% Mocap Data

 x=E.data(:,1);                % x-coordinate MoCap     NOTE: ingekort voor dimensies !!!
 y=E.data(:,2);                % y-coordinate MoCap        NOTE: ingekort voor dimensies !!!
 th=E.data(:,3);               % theta MoCap               NOTE: ingekort voor dimensies !!!
%MOCAP DIFFERENTIATIE


u1=gradient(x)/delta_t;

%%Offset IMU
 B = 1/50*ones(50,1);
 out = filter(B,1,u1);
 Lc = find(out<-0.01);%Sample Locatie waar mocap data aangeeft te gaan stijgen
 Li = find(A.data(1:Lc,2)<0.8);
 K =Lc(1,1);
 Bias  = mean(A.data(1:Li(end,1),2));
 A.data(:,2) = A.Data(:,2)-Bias;
 A.Data(1:K(1,1),2) = 0;
 A.Data(Lc(end,1):end,2) = 0;  
 


%% Script

% INTEGRATE IMU DATA/OR DIFFERENTIATE MOCAP DATA FOR SPEED
% INTEGRATIE: Dit kan met de commando: cumtrapz(matrix). Deze waardes zijn cumulatief, dus de
% waardes binnen een kolom worden bij elkaar opgeteld. Doe deze matrix
% vervolgens keer de tijdstap (delta t). Maak nu een matrix (even lang als
% de matrix met cumulatieve waarden) met de beginsnelheid erin. Haal de
% matrix met cumulatieve waarden af van deze matrix. Dit moet de snelheden
% per tijdstip geven
%
% DIFFERENTIATE: Dit kan met de commando: gradient(matrix)/delta t of diff/delta t.
% met gradient blijven de dimensies van de matrix gelijk, dus makkelijker.
% Het probleem is dat de waarden na paar sec grote afwijkingen vertonen ==>
% Kalman filter


%IMU INTEGRATIE
A1=cumtrapz(A.data); % integrate matrix C (cumulative trapezoidal integration)
A2=A1*delta_t; % multiply by delta t``
v0=zeros(length(A.data),3); % set beginvelocities for u, v and r to 0
velocity=v0+A2;

v=velocity(:,1); % Longitudinal velocity.
% u = out(:,1);
u=velocity(:,2);       % Lateral velocity
%r=velocity(:,3);   %  Yaw rate
r=zeros(length(v_d),1);

  
 u(1:Lc(1,1),1) = 0; 
 u(Lc(end):end,1) = 0;


%% Alpha (Slip Angle)
 
alphaf = del - revtan(v+0.167.*r, u);       % slipangle frontwheel
alphar = - revtan(v-0.167*r, u);                 % slipangle rearwheel
alpha = [alphaf, alphar];



%% Kappa (Longitudinal Slip Ratio)


 kappa = zeros(length(v),2);
%  vxw = zeros(length(v),1);
 vxw = cos(del).*u + sin(del).*(v + r.*0.167);
 g=1;
 while g <= length(v)
     
     if u_d(g) == 0                                 % At constant speed use acceleration kappa
     kappaf = (wf(g,1).*0.048 - vxw(g,1))./(wf(g,1).*0.048);
     kappar = (wr(g,1).*0.048 - u(g,1))./(wr(g,1).*0.048);
     kappa(g,:) = [kappaf, kappar];
     g = g+1;
     elseif u_d(g) > 0                             %accelerating
     kappaf = (wf(g,1).*0.048 - vxw(g,1))./(wf(g,1).*0.048);
     kappar = (wr(g,1).*0.048 - u(g,1))./(wr(g,1).*0.048);
     kappa(g,:) = [kappaf, kappar];
      g= g+1;
     elseif u_d(g)<0                             % braking
%       kappaf = (vxw(g,1)-wf(g,1).*0.048)./(vxw(g,1));
%       kappar = (u(g,1)-wr(g,1)*0.048)./(u(g,1));
%       kappa(g,:) = [kappaf, kappar];
      g = g +1;
     end
 end
 
 kappaf = [kappa(:,1), -kappa(:,1)];
 kappar = [kappa(:,2), -kappa(:,2)];
 
 
 Zk = find(kappaf(:,1)<-1);% Find point where kappaf is smaller than -1 wich is impossible
 Zk2 = find(kappaf(:,1)>1);% Find point where kappaf  is bigger than 1.
 kappaf(Zk,1)  = NaN; % Deletes inrellevant noise
 kappaf(Zk2,1) = NaN; % Deletes inrellevant noise 
 
 Zk3 = find(kappar(:,1)<-1);% Find point where kappaf is smaller than -1 wich is impossible
 Zk4 = find(kappar(:,1)>1);% Find point where kappaf  is bigger than 1.
 kappaf(Zk3,1)  = NaN; % Deletes inrellevant noise
 kappaf(Zk4,1) = NaN; % Deletes inrellevant noise 
 

 zk5 = find(kappaf(:,1)<0);
 kappaf(zk5,1)= NaN;
 
 zk6 = find(kappar(:,1)<0);
 kappar(zk6,1)=NaN
 
 % KAPPA KAN OOK OP DEZE MANIER IN DE CODE
% 
% vxw= cos(del).*u + sin(del).*(v + r.*0.167); % Front wheelcenter velocity

%Kappa waardes tijdens ACCELERATION
% kappaf = (wf.*0.048 - abs(vxw))./(wf.*0.048);
% kappar = (wr.*0.048 - abs(u))./(wr.*0.048);
% kappa = [kappaf, kappar];





%% Non Linear Tire Forces
% Met behulp van de equations of motion worden de krachten op de
% banden bepaald

% 
%  Fyr=(lf*m*(v_d+u.*r)-I*r_d)./(lf+lr) ; % lateral force rear tires y-axis (rearwheel axis)
%  Fyf=m*(v_d+u.*r)-Fyr;  % lateral force front tires y-axis (not frontwheel axis)
% 
%   Fywf=  (Fyf-m*(u_d-v.*r).*sin(del)+sin(del).*(m*(u_d+v.*r).*cos(del)/(1+cos(del))))./(((sin(del).^2)/(1+cos(del)))+cos(del)); % lateral force front tire
%   Fxwf = (Fyf-Fywf.*cos(del))./(sin(del));    % longitudinal force front tire
% 
%   Fxf = Fxwf.*cos(del)-Fywf.*sin(del);   % front tire longitudinal along x-axis
%   Fxr = m*(u_d-v.*r)-Fxf ;               % rear tire longitudinal along x-axis
% 
%   Fxwr = Fxr; % longitudinal force rear tire
%   Fywr = Fyr; % Lateral force rear tire
% 
% F = [Fxwf Fywf Fxwr Fywr]; % [{long front}{lat front}{long rear}{lat rear}]
% 
% 
% %oude groep
Fyr = 0.5.*(m.*(v_d+u.*r)-(r_d.*I./0.167));    %M = I*theta_dot
Fyf = (r_d.*I./0.167) + Fyr;                        %F = m*a
Fy = [Fyf,Fyr]; 
     

Fxw = (m.*(u_d-v.*r) - tan(del).*Fy(:,1))./(cos(del) + 1 + tan(del).*sin(del)).*[1,1]; 
Fyw = [(Fy(:,1) - Fxw(:,1).*sin(del))./cos(del), Fy(:,2)];
F = [Fxw, Fyw];     % Krachten kloppen nog niet helemaal !!!

Fxf=[Fxw(:,1), -Fxw(:,1)];
Fxr=[Fxw(:,2), -Fxw(:,2)];

%F in de vorm: [Fxf Fxr Fyf Fyr]
% [Fxw(:,1) Fxw(:,2) Fyw(:,1) Fyw(:,2)]

%% Linear Tire Forces

%RECHTDOOR RIJDEN. 4x4 aangedreven en massa gelijk verdeeld. Geen laterale krachten
% Fxf = 0.5*m*u_d ;    % longitudinal force front
% Fxr = 0.5*m*u_d ;    % longitudinal force rear
% 
% % FOR STEADY STATE CORNERING (yaw acceleration is 0): slide 14 lecture 5 
%               
%  Fyr = (lf/L)*m*u.*r;      %  lateral rear, lf (COG->front tire) and L (rear tire->front tire) are lenghts
%  Fyf = (lr/L)*m*u.*r;      %  lateral front, lr (COG->rear tire) and L are lengths
% 
%  Fxw=[Fxf Fxr];
%  Fyw=[Fyf Fyr];
% Gebruik deze als we op zoek zijn naar de laterale krachten en de
% bijbehorende slip angles. Longitudinale krachten en kappa zijn tijdens
% steady state cornering niet van toepassing. 


 Fxf(Zk,1)  = NaN;
 Fxf(Zk2,1) = NaN;
 
 Fxr(Zk3,1)  = NaN;
 Fxr(Zk4,1) = NaN;
          

%% Put everything in an S matrix
% S = [t, v, u, r, v_d, u_d, r_d, wf, wr, del, x, y, th, F, kappaf, kappar, alphaf, alphar];


%% Plot tyre characteristics

% 2-D plots

figure
% subplot(2,1,1)
plot(kappaf,Fxf,'*')    % Front Tires/ kappa vs longitudinal
hold on
title('Front Tires: Kappa vs Longitudinal Force')
xlabel('Kappa');
ylabel('Longitudinal Force');
%axis([-1 1 0 inf])


% subplot(2,1,2)
% plot(alphaf,Fyf,'*')    % Front Tires/  alpha vs lateral
% title('Front Tires: Alpha vs Lateral Force')
% xlabel('Alpha')
% ylabel('Lateral Force')

 figure
% subplot(2,1,1)
plot(kappar,Fxr,'*')    % Rear Tires/  kappa vs longitudinal
title('Rear Tires: Kappa vs Longitudinal Force')
xlabel('Kappa')
ylabel('Longitudinal Force')
%axis([-1 1 0 inf])

% subplot(2,1,2)
% figure 
% 
% plot(alphar,Fyr,'*')    % Rear Tires/   alpha vs lateral
% title('Rear Tires: Alpha vs Lateral Force')
% xlabel('Alpha')
% ylabel('Lateral Force')



% 3-D plots


% figure
% plot3(kappa,alpha,Fxw)  % surface plot longitudinal forces
% title('Longitudinal')
% xlabel('Kappa')
% ylabel('Alpha')
% zlabel('Longitudinal Force')
% legend('Front Tires','Rear Tires')
% 
% figure
% plot3(kappa,alpha,Fyw)  % surface plot lateral forces
% title('Lateral')
% xlabel('Kappa')
% ylabel('Alpha')
% zlabel('Lateral Force')
% legend('Front Tires','Rear Tires')




    %% Filtering data
    % DO NOT USE WHEN MAKING TURNS LARGER THAN 90 DEGREES!!!
    %s_theta_f = bwftheta(s_theta);
    
%     %% Fixing the jump in theta
%     s_theta_f = s_theta; 
%     for i = 1:(length(s_theta_f)-1)
%         if s_theta_f(i+1)-s_theta_f(i) < -6
%             s_theta_f((i+1):length(s_theta_f)) =  s_theta_f((i+1):length(s_theta_f)) + 2*pi;
%         elseif s_theta_f(i+1)-s_theta_f(i) >6
%             s_theta_f((i+1):length(s_theta_f)) =  s_theta_f((i+1):length(s_theta_f)) + 2*pi;
%         end
%     end
        
%     %% Smooth out the raw data from Simulink
%     % 'loess' because it can follow the data more precisely than 'rloess'
%     fprintf('smoothing s_x...');
%     s_x_smooth = smooth(simulationtime,s_x,ssgx,'rloess');      
%     fprintf('done. smoothing s_y...');
%     s_y_smooth = smooth(simulationtime,s_y,ssgy,'rloess');
%     fprintf('done. smoothing theta...');
%     s_theta_smooth = smooth(simulationtime,s_theta_f,ssgt,'rloess');
%     fprintf('done\n');
  
    
   
   
%  save(sprintf('S%0.f.mat', i),'S');                                            % Save State matrix
%  CarAnimation(S);                                                              % Simulate Car motion
%  
  

    




    


   
    
