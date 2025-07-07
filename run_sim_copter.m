%% Prepare the environment

clear all
close all

addpath('simulator');

%% Define the simulation

% Define the hardware architecture
r = robots.quadrotor();
%r = robots.octorotor_assymmetric();
%r = robots.floating_hex();
%r = robots.tilted_hex(true);
%r = robots.odar();

% Define the world
average_wind = [];
w = worlds.empty_world(average_wind, false);
%w = worlds.straight_wall(average_wind, false);
%w = worlds.sloped_wall_20_deg(average_wind, false);

% Define the controller
c = controllers.fully_actuated(r, attitude_strategies.Full);

% Define the simulation object
sim = simulation(r, c, w);

%% Initial multirotor state

pos = [0; 0; -4];
vel = [0; 0; 0];
rpy = [0; 0; 0];
omega = [0; 0; 0];
lastThrust = [0; 0; 0];
sim.Multirotor.SetInitialState(pos, vel, rpy, omega, lastThrust);

%% Get the controller response(s)

% Simulate position response
sim.SetTotalTime(10);
figure;
sim.SimulatePositionResponse([17; 8; -2], -45, true);

%% Draw Additional plots

graphics.PlotSignalsByName(3, {'pos', 'vel', 'accel', 'rpy', 'euler deriv', 'ang accel'}, false, true);

%% Animate the result

fpv_cam = camera;
fpv_cam.Offset = [0; 0; -0.35];
graphics.AnimateLoggedTrajectory(sim.Multirotor, sim.Environment, 0, 1, true, true, []);
%graphics.RecordLoggedTrajectoryAnimation('myvideo', 30, sim.Multirotor, sim.Environment, 0, 1, true, true, fpv_cam);
