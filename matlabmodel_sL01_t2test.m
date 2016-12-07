clear, clc
% Design Project Mathmatical Model 
% Holly Wilson, Brooke Powell, Sarah Weigel 
% Eng 1101, Team 2, 28 March 2014
% Program name: Matlab trial model 

% This program is designed to find the overall speed of the powerband 
% candyvan, using multiple variables and accounting for different payload
% amounts. To reach the end goal of 20ft as quickly as possible. 

% Intializing Variables 
%--------------------------------------------------------------------------
px = -.36;          % Drive axle x-coordinate(m)
py = -.02;          % Drive axle y-coordinate (m)
Ra = .0015;         % Drive axle radius (m)
Rw = .06;           % Drive wheel radius (m)
Mv = 0.5;           % Mass of vehicle (kg)
Mp = .05;           % Mass of one payload (kg) 
Np = 0;             % Number of units in the payload
BLrelaxed =.09;     % Band length unstretched
BLstretch(1) = .20; % Band length stretched 
mus = 0.02;         % Coefficient of static friction of vehicle 
muk = 0.01;         % Coefficient of kinetic friction of vehicle
m = 0.133;          % Slope constant (N/%L)
b = 1.016;          % y-intercept when x is 0 (N)
perS = ((BLstretch (1)-BLrelaxed)/BLrelaxed)*100; % X value 
g = 9.81;           % gravitational constant
dt = .05;           % increment of time for numerical intergration of coast
                    % phase (s)

% User input payload amount 
%--------------------------------------------------------------------------
Np = input ('Enter number of candy bars to be placed on car ');

% Calculations
%--------------------------------------------------------------------------
Dop = sqrt((px)^2)+((py)^2);                               
Lstring = Dop - BLstretch(1);
Fband(1) = m*perS+b;

Fmotive(1) = Fband(1)*(Ra/Rw);
Mtot=Mv+Np*Mp;
Ffs=mus*(Mv*g);
Ffk=muk*(Mv*g);

Fnet(1) = Fmotive(1) - Ffs;
accel(1) = Fnet (1)/Mtot;
vel_car(1) = 0;
dist (1)= 0;
StrUnwind (1)= 0;
time(1)= 0;

% Setup Integration 
%--------------------------------------------------------------------------
i = 2;
dt = .05;                                      % time step (s)
time (i-1);                                    % time intergration 
delements = time (i-1)/dt-1;  
   
    % Loop through time
while dist(i-1) <= 6.1                         % in meters 
    % Calculate distance and velocity of car
        accel (i) = Fnet (i-1)/Mtot;           %acceleration equation
        vel_car (i) = vel_car(i-1)+accel(i)*dt;%velocity of car (m/s)
        dist(i) = dist(i-1)+vel_car(i)*dt;     %distance of car (m)
        time(i)=time(i-1)+dt;                  %time equation 
        StrUnwind(i) = (dist(i)*(Ra/Rw));      %amount of string unwound
        BLstretch(i)=Dop-(StrUnwind(i)+Lstring);%drive phase
    if  BLstretch(i) >=BLrelaxed;              
        perS = (BLstretch(i)-BLrelaxed)/BLrelaxed*100;
        Fband(i)= m*(perS)+b;
        Fmotive (i)= Fband(i)*(Ra/Rw);
        Fnet (i)= Fmotive(i)-Ffk;
    else                                  %amount of stretch in coast phase
        Fband(i)= 0;
        Fmotive(i) = 0; 
        Fnet(i) = Fmotive(i)-Ffk;
    end
    i=i+1;
end

%Outputs:
%--------------------------------------------------------------------------
fprintf( '\n Amount of time to reach 20 feet: %3.2f seconds \n.', time (i-1))
%Outputs graphs.
%--------------------------------------------------------------------------
subplot(2,2,1);
plot (time,dist)                              %create distance vs time plot
xlabel('time, t (s)');                        %label on x-axis
ylabel('distance, dist (m)');                 %label on y-axis
title('Figure 1.Distance of the candyvan as a function of time.');
                                              % title of plot
subplot (2,2,2);
plot (time, BLstretch)                        %create band stretch vs time plot 
xlabel('time, t (s)');                        %x-axis label 
ylabel('Rubber band stretch, BLstretch (m)'); %y-axis label
title('Figure 2.Stretch of rubber band as a function of time.');
                                              %title of plot
subplot (2,2,3)
plot (time, Fnet)                             %create net force vs. time plot
xlabel('time, t (s)');                        %x-axis label 
ylabel('Net force, Fnet (N)');                %y-axis label
title('Figure 3.Net force of the candyvan as a function of time.');

subplot(2,2,4)                                %title of plot
plot(time, vel_car)                           %create Velocity of the 
                                              %candyvan as a function of
                                              %time
xlabel('time, t (s)');                        %x-axis label
ylabel('Velocity, vel_car, (m/s)');           %y-axis label
title('Figure 4.Velocity of the candyvan as a function of time.');
                                              %title of plot
