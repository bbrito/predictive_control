%% Clean up
clear; clc; close all; clearvars;
rng('shuffle');

%% Delete previous Solver
% Forces does not always code changes and might reuse the previous solution
try
FORCEScleanup('FORCESNLPsolver','all');
catch
end

try
    rmdir('@FORCESproWS','s')
catch
end
try
    rmdir('FORCESNLPsolver','s')
catch
end
% 
%% Some utility functions
deg2rad = @(deg) deg/180*pi; % convert degrees into radians
rad2deg = @(rad) rad/pi*180; % convert radians into degrees

%% Problem dimensions
model.N = 12;            % horizon length
model.nvar = 7;          % number of variables
model.neq= 4;            % number of equality constraints
model.nh = 0;            % number of inequality constraint functions
n_other_param = 40;

model.npar =  n_other_param;          % number of parameters

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
%            inputs               |               states
%                v      w     sv     x      y       theta      dummy
%               
% model.lb = [ -2.0,  -1.0,   0, -200,   -200,    -1.5*pi,        0  ];
% model.ub = [ +2.0,  +1.0,   800, +200,   +200,    +1.5*pi,    inf];

% Lower limits for robot
lb_R = [ -2.0,  -1.0, 0, -500,   -500,    -pi,   0];
model.lb = lb_R;

% Upper limits for robot
ub_R = [ +2.0,  +1.0, +inf, +500,   500,    +pi,   +inf];

model.ub =ub_R;
%%
for i=1:model.N
    %% Objective function
    model.objective{i} = @(z, p) objective_scenario(z, p); 

    %model.ineq{i} = @(z,p) inequality_constr_scenario(z, p, i);

    %% Upper/lower bounds For road boundaries
    %model.hu{i} = [6, 6];   
    %model.hl{i} = [-inf, -inf];
end
%% Dynamics, i.e. equality constraints 
%model.objective = @(z, p) objective_scenario_try(z, p);
model.eq = @(z, p) dynamic_scenario(z, p, 0);

model.E = [zeros(4,3), eye(4)];

%% Initial and final conditions
% Initial condition on vehicle states

model.xinitidx = 4:7; % use this to specify on which variables initial conditions are imposed
%model.xfinal = 0; % v final=0 (standstill), heading angle final=0?
%model.xfinalidx = 6; % use this to specify on which variables final conditions are imposed

%% Define solver options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.maxit = 250;   % Maximum number of iterations
codeoptions.printlevel = 1 ; % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 2;   % 0: no optimization, 1: optimize for size, 2: optimize for speed, 3: optimize for size & speed
codeoptions.timing = 1;
codeoptions.overwrite = 1;
codeoptions.mu0 = 20;
codeoptions.cleanup = 0;
codeoptions.BuildSimulinkBlock = 0;
%codeoptions.nlp.lightCasadi = 1;

FORCES_NLP(model, codeoptions);
