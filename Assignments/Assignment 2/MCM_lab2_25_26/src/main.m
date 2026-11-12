%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
geometricModel = geometricModel(iTj_0,jointType);

%% Q1.3
bTe = geometricModel.getTransformWrtBase(7);
disp('first bTe')
disp(bTe)
T62 = geometricModel.getTransformWrtRandomFrame(6,2);
disp('T62')
disp(T62)

%% Q1.4 Simulation
% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
qi = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(qi)
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

% Show simulation ?
show_simulation = true;

% Set initial and final joint positions
qf = [5*pi/12, -pi/4, 0, -pi/4, 0, 0.18, pi/5];

%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples
    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
        pm.plotIter(bTi)
    end

end

pm.plotFinalConfig(bTi)

%% Q1.5

km = kinematicModel(geometricModel); %Per abbreviare
J_6 = km.getJacobianOfLinkWrtBase(6); %Siamo già nella qf di prima
disp('J_6')
disp(J_6);

%% Q1.6
km.updateJacobian(); %Siamo già nella qf di prima
disp('updatedJacobian')
disp(km.J);
%% Q1.7
qf = [0.7; -0.1; 1; -1; 0; 0.03; 1.3];
q_dot = [0.9; 0.1; -0.2; 0.3; -0.8; 0.5; 0];

%Update geometry
geometricModel.updateDirectGeometry(qf)

km.updateJacobian();
J_ee = km.J;
Twist = J_ee * q_dot;


bTe = geometricModel.getTransformWrtBase(geometricModel.jointNumber);
rot = bTe(1:3, 1:3);
t = bTe(1:3,4);%Translation
Tcross = [0 -t(3) t(2); t(3) 0 -t(1); -t(2) t(1) 0]; %skew-symmetric of the translation vector t. 
% Ad = inv([rot, zeros(3); Tcross * rot, rot]); %Adjoint operator to
% transform twists, Jacobians and wrenches from a reference frame to
% another.
% eTwist = Ad * Twist;
omega_ee = rot' * Twist(1:3); %I express the vector on the ee frame (instead on the base frame)
Vl = Twist(4:6);
Vl_ee = rot' * (Vl - Tcross * Twist(1:3)); %I express the vector on the ee frame (instead on the base frame)
disp('Omega in EE frame');
disp(omega_ee);
disp('Velocity in EE frame');
disp(Vl_ee);


%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

pm = plotManipulators(show_simulation);
pm.initMotionPlot(t);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples
    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
        pm.plotIter(bTi)
    end

end

pm.plotFinalConfig(bTi)
