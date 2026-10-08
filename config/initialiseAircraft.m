% Aircraft aerodynamic, mass, inertia, propulsion and trim parameters.
% Derived from the project SIXDOFparams.m; called by prepareFlightSimulation.

g = 9.81;
density = 1.225;


% Mass of the aircraft

aircraft_mass = 145;
payloadvolume = 47;
emptymass = aircraft_mass - payloadvolume;

%Cruise

CruiseSpeed = 25.8;



%Aerodynamic dimensions

l = 0.84 ; % chord length
lt = 3; % distance between CoG and the aerodynamic centre of the tail unit

S = 3.35; % Wing planform area
St = 0.977 ; % Tail unit planform area

b = 5.46; % wingspan



alpha0 = -4*(pi/180);

stallAoA = 14.5*(pi/180);

dwGrad = 0.36;



alpha_bp = -45:45;

CL_table = [0 0 0 0 0 0 0 0 0 0 0 -0.009 -0.034 -0.074 -0.127 -0.190 ...
            -0.262 -0.341 -0.425 -0.512 -0.599 -0.686 -0.769 -0.848 ...
            -0.920 -0.984 -1.036 -1.076 -1.102 -1.111 -1.070 -0.971 ...
            -0.849 -0.739 -0.646 -0.554 -0.461 -0.368 -0.275 -0.182 ...
            -0.089 0.004 0.096 0.189 0.282 0.375 0.468 0.561 0.654 ...
            0.746 0.839 0.932 1.025 1.118 1.211 1.304 1.414 1.536 ...
            1.634 1.675 1.664 1.632 1.582 1.516 1.435 1.343 1.241 ...
            1.131 1.016 0.897 0.778 0.659 0.544 0.434 0.332 0.240 ...
            0.159 0.093 0.043 0.011 0 0 0 0 0 0 0 0 0 0 0];

Cldeltae = 0.359;


%Stab Con

hPayload = 0.0855;
hNoPayload = 0.227;

CGPayload = hPayload * l;
CGNoPayload = hNoPayload *l;

%h0 = 0.21;
h0 = 0.25;
COP = h0 * l;

%FullInertiaMatrix = [150 0 0 ;
%                     0 200 0 ;
%                     0 0 180];

%EmptyInertiaMatrix = [150 0 0 ;
%                     0 180 0 ;
%                     0 0 180];

FullInertiaMatrix = [155 0 0 ;
                     0 185 0 ;
                     0 0 315];

EmptyInertiaMatrix = [150 0 0 ;
                      0 170 0 ;
                      0 0 300];


%Propellor

maxPropRad = 423.38;
rho = 1.225;
CT =  0.0136;
CQ =  0.01540; 

PropDiameter = 0.889;
PropRadius = PropDiameter/ 2;


Cldampingrates = [-11 0 5]; 
Cmdampingrates = [0 -4.03*(St/S)*(lt^2/l^2) 0];
Cndampingrates = [1.7 0 -11.5];

ClCsurfaces = [-0.6 0 0.22];
CmCsurfaces = [0 -3.1*(St/S)*(lt/l) 0];
CnCsurfaces = [0 0 -0.63];

InitUb = CruiseSpeed;
%Initwb = 1.5747;
Initwb = 3.087;
InitPitch = 0.11910; 


%ElTrim = 0.1098111092818;
%ElTrim = -0.123;
%ElTrim = -0.016;
%ElTrim = -0.015671;
ElTrim = -0.01568216;
%ElTrim = -0.01568;


AilTrim = 0;

%PitchTrim = 0.060509;
PitchTrim = 0.11910;

ThrottleTrim = 0.7606;

