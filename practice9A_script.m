%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%          MR3003. Autonomy of Unmanned Aerial Vehicles
%            Practice 9 - State Feedback Gain Matrix
%                 Part A. State Space Modelling
%             Dr. Carlos Sotelo - Dr. David Sotelo
%                 Carlos Hernán Auquilla Larriva
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Last update: September 30th 2024

clear;clc;close all;

%% Step 1: Load Qube Servo 3 Parameters
% Motor
% Resistance
Rm = 7.5;
% Current-torque (N-m/A)
kt = 0.0422;
% Back-emf constant (V-s/rad)
km = 0.0422;
%
% Rotary Arm
% Mass (kg)
mr = 0.095;
% Total length (m)
r = 0.085;
% Moment of inertia about pivot (kg-m^2)
Jr = 2.2879e-4;
% Equivalent Viscous Damping Coefficient (N-m-s/rad)
br = 1e-3; % damping tuned heuristically to match QUBE-Sero 2 response
%
% Pendulum Link
% Mass (kg)
mp = 0.024;
% Total length (m)
Lp = 0.129;
% Pendulum center of mass (m)
l = 0.0645;
% Moment of inertia about pivot (kg-m^2)
Jp = 1.3313e-4;
% Equivalent Viscous Damping Coefficient (N-m-s/rad)
bp = 5e-5; % damping tuned heuristically to match QUBE-Sero 2 response
% Gravity Constant
g = 9.81;

% Find Total Inertia
Jt = Jp*Jr-mp^2*l^2*r^2;
 
%% Step 2: Program and compute the State Space Representation matrixes
A = (1/Jt)*[0 0 Jt 0;
     0 0 0 Jt;
     0 mp^2*l^2*r*g -Jp*(br+km^2/Rm) mp*l*r*bp;
     0 -mp*g*l*Jr mp*l*r*(br+km^2/Rm) -Jp*bp];

B = [0
    0;
    km*Jp/(Rm*Jt);
    -mp*r*l*km/(Rm*Jt)];

C = [1 0 0 0;
     0 1 0 0];

D = [0;0];

sys = ss(A,B,C,D)


%% Step 3: Plot the response
load("a00834778_P9A.mat");
t = data(1,:);

voltage = data(2,:);

pos_arm = data(3,:);
por_arm_model = data(4,:);
pos_pend = data(5,:);
pos_pend_model = data(6,:);

figure(1)
plot(t,voltage,"r");
legend("plant","model")
legend("Manipulation")
ylabel("Motor Voltage [V]");

figure(2)
plot(t,pos_arm,"g");
hold on
plot(t,pos_pend_model,"--r");
legend("plant","model")
ylabel("Rotary Arm Angle [rad]");

figure(3)
plot(t,pos_pend,"g");
hold on
plot(t,pos_pend_model,"--r");
legend("plant","model")
ylabel("Inverted Pendulum Angle [rad]");
