close all; clear all; clc;
%% simulation parameters
without_meas_uncertainty  = true;
without_actuator_dynamics = true;

%% assumed manipulator truth parameters
% these are the values used to compute the dynamics of the manipulator,
% which is used for feedback linearization

% link lengths (m)
l = [0.25 0.25];

% link widths (m)
w = [0.04 0.04];

% link heights (m)
h = [0.04 0.04];

% link masses (kg)
m = [1 1];

% link moments of inertia (kgm^2) where links are represented as solid
% cuboids of dimension lxwxh and the matrix is represented in the body
% frame
I = 1/12 * m .* [w.^2+h.^2; l.^2+h.^2; l.^2+w.^2; zeros(3,2)];

% gravity vector (m/s^2)
g = [0 -9.81 0];

% denavit-hartenberg parameters (row i corresponds to link i)
dh = [...
    l(1) 0 0 0; ...
    l(2) 0 0 0];

%% manipulator truth setup
% this is where we should apply model uncertainties

% add joints
joint1 = robotics.Joint('joint1', 'revolute');
joint2 = robotics.Joint('joint2', 'revolute');

joint1.setFixedTransform(dh(1,:), 'dh');
joint2.setFixedTransform(dh(2,:), 'dh');

% add links
link1 = robotics.RigidBody('link1');
link2 = robotics.RigidBody('link2');

link1.Joint = joint1;
link2.Joint = joint2;

link1.Mass = m(1);
link2.Mass = m(2);

link1.CenterOfMass = [l(1) / 2 0 0];
link2.CenterOfMass = [l(2) / 2 0 0];

link1.Inertia = I(:,1)';
link2.Inertia = I(:,2)';

% create robot
robot = robotics.RigidBodyTree();
robot.addBody(link1, 'base');
robot.addBody(link2, 'link1');
robot.DataFormat = 'row';
robot.Gravity    = g;

%% controller parameters
% controller state will be [q1 q2 q1_dot q2_dot]
A = [zeros(2) eye(2); zeros(2) zeros(2)];
B = [zeros(2); eye(2)];

Q = diag([1 2 0.1 0.1]);
R = diag([0.005 0.005]);

K = lqr(A,B,Q,R);

%% Encoder settings
resolution = 2^12;
%resolution = 2^14;
%resolution = 2^16;

offset_error = 1/180*pi; %degree to rad

%measurement delay
delay_length = 1;

%signal noise
mean = 0;
variance = 0.001;
seed = 1;