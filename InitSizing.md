# prelimcode
preliminary sizing code

clear all
close all
clc;

%atmospheric conditions
altitude = 48; %kft
T0 = 389.970; %R
P0 = 266.654; %lb/ft^2
rho0 = 0.000398389; %slugs/ft^3
a0 = 968.076; %ft/s
mu0 = 2.99135e-7; 
theta0 = 0.7519;
sigma0 =   0.1685;

Mcruise = 1.8;
Vcruise = a0*Mcruise;

%% Takeoff Weight

%flight knowns

M0 = 1.8; %1.6-1.8
Range = 4000*1852;% m- 4000 nautical miles
RangeE = 4000*6080;
FuelEfficiency = 1; %passenger mile per lb fuel
V0 = M0*a0; %ft/s

%Wto = Woe + Wf + Wp;
%Woe = We + Wc + Wtfo;
%Wf/Wp = (1+We/Wp)(1.022*exp(R/X-1);

%Assumptions
WavgPerson = 180; %lbf
WavgLuggage = 50; %lbf
numPass = 10; %lbf
numCrew = 2; % 2 pilots and 1 flight attendant
Wp = numPass*(WavgPerson+WavgLuggage);
Wc = numCrew*WavgPerson;
%TSFC = (0.9+0.3*M0)*sqrt(theta0); %correlation from LB mixed flow turbofan



%Known Equations
%Wto = Woe + Wf + Wp;
%Woe = We + Wc + Wtfo;
%Wf/Wp = (1+We/Wp)(1.022*exp(R/X-1);


H = 44.65*10^6; %Jm/kg from google jet fuel
nProp = 0.84; %from typical prop efficiency
g = 9.80665; %m/s^2
v_TSFC = nProp*H/g; 


TSFC = (0.9+0.3*M0)*sqrt(theta0);
L_D = 4*(M0+3)/M0; %empirical model
RF = L_D*V0/(TSFC)*3600; %ft




Wto1 = 105000*2.20462; %lbf

%Fuel Fractions
%Mission: Takeoff, Climb and Accelerate, Cruise, Descent, Land

W10= 0.97;
W21 = 0.991-0.007*M0-0.01*M0^2;
%W32 = exp(-Range/(v_TSFC*L_D*g)); %based off of correlation in class
W32E = exp(-RangeE/RF);
W43 = 0.99;
W54 = 0.992;
W5_1 = W10*W21*W32E*W43*W54;

Wf_0 = 1.07*(1-W5_1);

%Empty Weight Fraction
%Simple Method- Jet Fighter
A = 2.34;
C = -0.13;
Ks = 1;


%A = 1.09;
%C = -0.05;

z = 0;
residual = inf;



while z<1000 && residual > 0.01

We_0 = A*Wto1^(C)*Ks;
We = Wto1*We_0;

Wf = Wto1*Wf_0;

Wto = We+Wf+Wp+Wc;

residual = abs(Wto1-Wto);
Wto1 = Wto;
z = z+1;
end 


Wto %lbf
We = Wto*We_0;
Wf = Wto*Wf_0;


%% Stall and Landing Requirements

Vstall = 219.415; %from 130 knots  Table 4.11
CLmaxTO = 2; %Table 3.1
WS_stall = 0.5*rho0*Vstall^2*CLmaxTO;
WS_landing = 0.5*rho0*(1.3*Vstall)^2*CLmaxTO;



%% Vmax Requirements

e = 0.9; %Pg 122
AR = 4; %Table 5.8
K = 1/(pi*e*AR);
CD0 = 0.025; %pg 127 Table 4.12

Vmax = Vcruise; %pg 126
WS = linspace(10,50);
WT_Vmax = ((rho0*Vmax^2*CD0*1./(2*WS)+ 2*K/(rho0*sigma0*Vmax^2)*WS)).^-1;

%% Takeoff Sizing Requirements
Xto = 7000; %ft maximum takeoff distance

TOP = 190; % BF for 3 engines
sigmaSL = 1;

WT_TO = (sigmaSL*TOP*CLmaxTO)./WS;
%TW(i,:,k) = TWrange;
%WS(i,:,k) = TOP(k)*sigma(k)*CLmaxto(i)*TW(i,:,k);



%% Sizing Diagram

ystart = 0;
yend  = WT_TO(1);

figure()
%plot(WS,WT_Vmax);
%hold on
plot(WS,WT_TO);
hold on
plot([WS_stall WS_stall], [ystart yend])
hold on
plot([WS_landing WS_landing], [ystart yend])
hold on
plot(18,13,'*')
legend('Takeoff', 'Stall','Landing')

%% Drag Polar

WSloc = 18;
S = Wto/WSloc;

CL = linspace(0,2);
CD = CD0 + K*CL.^2;

figure()
plot(CL,CD)
xlabel('Coefficient of Lift')
ylabel('Coefficient of Drag')
title('Drag Polar')

WSloc = 18 
WTloc = 13
T = Wto/WTloc;
LD = 12;

CLmaxTO = Wto/(0.5*rho0*Vstall^2*S)
CLmaxL= (Wto - Wf)/(0.5*rho0*Vstall^2*S)
