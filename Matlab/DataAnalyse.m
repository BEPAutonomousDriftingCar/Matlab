
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


%% Import all the Matrices 

addpath(genpath('D:\Users\Eigenaar\Desktop\TuDelft\BEP\Database\Oude groep\170531\170531'))   % location of testdata


% Specify the path to the bag file either as an absolute or relative path:
filePath = fullfile('D:\Users\Eigenaar\Desktop\TuDelft\BEP\Database\Nieuwe groep\drive-download-20171129T142713Z-001','2017-11-29-14-19-35.bag');
bag = rosbag(filePath);



%% Bag Selection

% bag.AvailableTopics    % see available topics
% bag.MessageList        % see all messages 

 
start = bag.StartTime ;   


% list of messages that were recorded within the first 10 seconds
% of the rosbag and published on the /imu/data topic
                          
bagselect1 = select(bag, 'Time', [start start + 10], 'Topic', '/wheels');                          
bagselect2 = select(bag, 'Time', [start start + 10], 'Topic', '/imu/data') ;
bagselect3 = select(bag, 'Time', [start start + 10], 'Topic','/cmd_vel'); 

 
% bagselect3 = select(bagselect2, 'Time', [205 206]) ; % narrow down time window even further

%% Message Selection
 
%msgs = readMessages(bagselect2);     % Read the message from bagselect2
%msgs1= readMessages(bagselect1);
%msgs2= readMessages(bagselect3);
%% Extract Message Data as Time Series

format long



A = timeseries(bagselect2, 'LinearAcceleration.X','LinearAcceleration.Y','AngularVelocity.Z') ; % in de vorm: [u v r]
B = timeseries(bagselect1, 'Transform.Rotation.X','Transform.Rotation.Y','Transform.Rotation.Z','Transform.Rotation.W'); % in de vorm: [w1 w2 w3 w4]
D = timeseries(bagselect3, 'Angular.Z');



 % [A B]=synchronize(A,B,'union')
 [A B]=synchronize(A,B,'Uniform','Interval',0.005606023869739457);  % synchronize the two timeseries
 [A D]=synchronize(A,D,'union');
 [B D]=synchronize(B,D,'union');

A.data ; % shows the data from A (IMU)
B.data ; % shows the data from B (Pulsentellers)
D.data ; % shows the data from D (steering angle)
 
C=[A.time A.data B.data D.data];


t=C(:,1);
u_d=C(:,2); % load the IMU data, longitudinal acceleration
v_d=C(:,3); % load the IMU data, lateral acceleration
r_d=C(:,4); % load the IMU data, r_dot

wf=C(:,5); % load the Pulsenteller data, frontwheels
wr=C(:,7);  % load the Pulsenteller data, rearwheels 
del=C(:,9);  % load the Servo data, steering angle: del


total_t=t(end,1)-t(1,1);
delta_t=(t(end,1)-t(1,1))./(length(A.data)-1); % time between sampling points (delta t)
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

u=velocity(:,1);       % Longitudinal velocity.  
v=velocity(:,2);       % Lateral velocity
r=velocity(:,3);   %  Yaw rate


    
% deltload = load(sprintf('delta_Steer15'));            % Load steering angle data from the file 170531 (addpath)
% deltin = deltload.delta(2,:).'; clear deltload;       
    % met deltload en deltin kunnen we direct de waardes van de steering
    % angle importeren. Deltin geeft alleen de waardes van de steering
    % angle ZONDER tijd. Deltin wordt ge-invert zodat het in de goede vorm staat
    % Delta geeft de waardes van de angle en de tijd
    
    
%del=deltin([1:4],:) ;     % steering angle data   NOTE: Vector is ingekort voor dimensies !!!!
    

%% Mocap Data
    % We need the x, y and theta from the MoCap. The MoCap data is in the
    % file: smoothpam.txt
    
Z=importdata('smoothpam.txt');     % import the Mocap Data from the file
      
% Initialize variables
% markers = 1;         Nog niet duidelijk of dit erin moet               % plot marker size


x=Z([1:4],1);                % x-coordinate MoCap        NOTE: ingekort voor dimensies !!!
y=Z([1:4],2);                % y-coordinate MoCap        NOTE: ingekort voor dimensies !!!
th=Z([1:4],3);               % theta MoCap               NOTE: ingekort voor dimensies !!!


%% Kappa (Longitudinal Slip Ratio)

 kappa = zeros(length(v),2);
 vxw = zeros(length(v),1);
 for  g = 1:length(v)                          % Dimensies kloppend maken
 vxw = cos(del).*u + sin(del).*(v + r.*0.167); % Front wheelcenter velocity


if u_d(g) == 0                                 % At constant speed use acceleration kappa
   kappaf = (wf.*0.048 - vxw)./(wf.*0.048);
   kappar = (wr.*0.048 - u)./(wr.*0.048);
   kappa = [kappaf, kappar];
elseif u_d(g) > 0 && u(g) >= 0                              %accelerating
   kappaf = (wf.*0.048 - vxw)./(wf.*0.048);
   kappar = (wr.*0.048 - u)./(wr.*0.048);
   kappa = [kappaf, kappar];
elseif u_d(g) < 0 && u(g) <= 0                              %accelerating
   kappaf = (wf.*0.048 - vxw)./(wf.*0.048);
   kappar = (wr.*0.048 - u)./(wr.*0.048);
   kappa = [kappaf, kappar];
elseif u_d(g) < 0 && u(g) > 0                               %braking
   kappaf = (wf.*0.048 - vxw)./(vxw);
   kappar = (wr.*0.048 - u)./(u); 
   kappa = [kappaf, kappar];
elseif u_d(g) > 0 && u(g) < 0                                %braking
   kappaf = (wf.*0.048 - vxw)./(vxw);
   kappar = (wr.*0.048 - u)./(u);
   kappa = [kappaf, kappar]; 
end   
  
 end
    

 % KAPPA KAN OOK OP DEZE MANIER IN DE CODE

% vxw= cos(del).*u + sin(del).*(v + r.*0.167); % Front wheelcenter velocity

% Kappa waardes tijdens ACCELERATION
% kappaf = (wf.*0.048 - abs(vxw))./(wf.*0.048);
% kappar = (wr.*0.048 - abs(u))./(wr.*0.048);
% kappa = [kappaf, kappar];



%% Alpha (Slip Angle)
 
alphaf = del - revtan(v+0.167.*r, u);       % slipangle frontwheel
alphar = - revtan(v-0.167*r, u);                 % slipangle rearwheel
alpha = [alphaf, alphar];



%% Tire Forces
% Met behulp van de equations of motion worden de krachten op de
% banden bepaald


Fyr = 0.5.*(3.2.*(v_d+u.*r)-(r_d.*0.048875./0.167));    %M = I*theta_dot
Fyf = (r_d.*0.048875./0.167) + Fyr;                        %F = m*a
Fy = [Fyf,Fyr]; 
     

Fxw = (3.2.*(u_d-v.*r) - tan(del).*Fy(:,1))./(cos(del) + 1 + tan(del).*sin(del)).*[1,1]; 
Fyw = [(Fy(:,1) - Fxw(:,1).*sin(del))./cos(del), Fy(:,2)];
F = [Fxw, Fyw];     % Krachten kloppen nog niet helemaal !!!
% F in de vorm: [Fxf Fxr Fyf Fyr]
% [Fxw(:,1) Fxw(:,2) Fyw(:,1) Fyw(:,2)]


% RECHTDOOR RIJDEN. 4x4 aangedreven en massa gelijk verdeeld. 
Fxf = 0.5*m*u     % longitudinal force front
Fxr = 0.5*m*u     % longitudinal force rear



% FOR STEADY STATE CORNERING (yaw acceleration is 0): slide 14 lecture 5 
% Fyr = (lf/L)*(m*u^2)/R            %  lateral rear, lr and L are lenghts
% Fyf = (lr/L)*(m*u^2)/R            %  lateral front, lr and L are lengths
% Gebruik deze als we op zoek zijn naar de laterale krachten en de
% bijbehorende slip angles. Longitudinale krachten en kappa zijn tijdens
% steady state cornering niet zo belangrijk; liefst gesplitst houden



          

%% Put everything in an S matrix
% S = [t, v, u, r, v_d, u_d, r_d, wf, wr, del, x, y, th, F, kappaf, kappar, alphaf, alphar];


%% Plot tyre characteristics

% 2-D plots

figure
subplot(2,1,1)
plot(kappaf,Fxw(:,1))    % Front Tires/ kappa vs longitudinal
title('Kappa vs Longitudinal Force')
xlabel('Kappa');
ylabel('Longitudinal Force');
hold on
plot(kappar,Fxw(:,2))    % Rear Tires/  kappa vs longitudinal
legend('Front Tires','Rear Tires')


% subplot(2,2,2)
% plot(kappaf,Fyw(:,1))    % Front Tires/  kappa vs lateral
% title('Kappa vs Lateral Force')
% xlabel('Kappa')
% ylabel('Lateral Force')
% hold on
% plot(kappar,Fyw(:,2))    % Rear Tires/   kappa vs lateral
% legend('Front Tires','Rear Tires')


% subplot(2,2,3)
% plot(alphaf,Fxw(:,1))    % Front Tires/ alpha vs longitudinal
% title('Alpha vs Longitudinal Force')
% xlabel('Alpha');
% ylabel('Longitudinal Force');
% hold on
% plot(alphar,Fxw(:,2))    % Rear Tires/  alpha vs longitudinal
% legend('Front Tires','Rear Tires')


subplot(2,1,2)
plot(alphaf,Fyw(:,1))    % Front Tires/  alpha vs lateral
title('Alpha vs Lateral Force')
xlabel('Alpha')
ylabel('Lateral Force')
hold on
plot(alphar,Fyw(:,2))    % Rear Tires/   alpha vs lateral
legend('Front Tires','Rear Tires')


% 3-D plots


figure
plot3(kappa,alpha,Fxw)  % surface plot longitudinal forces
title('Longitudinal')
xlabel('Kappa')
ylabel('Alpha')
zlabel('Longitudinal Force')
legend('Front Tires','Rear Tires')

figure
plot3(kappa,alpha,Fyw)  % surface plot lateral forces
title('Lateral')
xlabel('Kappa')
ylabel('Alpha')
zlabel('Lateral Force')
legend('Front Tires','Rear Tires')




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
  

    




    


   
    
