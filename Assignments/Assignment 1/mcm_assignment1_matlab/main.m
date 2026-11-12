addpath('include');

% TO DO: Test assignment 1 MCM 2024-2025
%% 1.1 Angle-axis to rot
%Q1.2
h1_2 = [1; 0; 0];
theta1_2 = 90;

rad_theta1_2 = deg2rad(theta1_2); %conversion in radiadians

R1_2 = AngleAxisToRot(h1_2, rad_theta1_2)

%Q1.3
h1_3 = [0; 0; 1];
theta1_3 = pi/3;

R1_3 = AngleAxisToRot(h1_3, theta1_3)

%Q1.4

rho = [-pi/3; -pi/6; pi/3];

theta1_4 = norm(rho);
h1_4 = rho / theta1_4;
R1_4 = AngleAxisToRot(h1_4, theta1_4)

%% 1.2 Rot to angle-axis
%Q2.2

R2_2 = [1, 0, 0; 0, 0, -1; 0, 1, 0];
[h2_2, theta2_2] = RotToAngleAxis(R2_2)

R2_3 = [0.5, -sqrt(3)/2, 0; sqrt(3)/2, 0.5, 0; 0, 0, 1];
[h2_3, theta2_3] = RotToAngleAxis(R2_3)

R2_4 = [1, 0, 0; 0, 1, 0; 0, 0, 1];
[h2_4, theta2_4] = RotToAngleAxis(R2_4)

R2_5 = [-1, 0, 0; 0, -1, 0; 0, 0, 1];
[h2_5, theta2_5] = RotToAngleAxis(R2_5)

R2_6 = [-1, 0, 0; 0, 1, 0; 0, 0, 1];
[h2_6, theta2_6] = RotToAngleAxis(R2_6);


%% 1.3 Euler to rot

%Q3.2

R3_2 = YPRToRot(0, 0, pi/2)

%Q3.3
psi3_2 = deg2rad(60);
R3_3 = YPRToRot(psi3_2, 0, 0)

%Q3.4

R3_4 = YPRToRot(pi/3, pi/2, pi/4)

%Q3.5

R3_5 = YPRToRot(0, pi/2, -pi/12)

%% 1.4 Rot to Euler (Rot to YPR)
%Q4.2
R4_2 = [1, 0, 0; 0, 0, -1; 0, 1, 0];
[psi4_2, theta4_2, phi4_2] = RotToYPR(R4_2)

%Q4.3
R4_3 = [1/2, -sqrt(3)/2, 0; sqrt(3)/2, 1/2, 0; 0, 0, 1];
[psi4_3, theta4_3, phi4_3] = RotToYPR(R4_3)

%Q4.4
R4_4 = [0, -sqrt(2)/2, sqrt(2)/2; 0.5, (sqrt(2)*sqrt(3))/4, (sqrt(2)*sqrt(3))/4; -sqrt(3)/2, sqrt(2)/4, sqrt(2)/4];
[psi4_4, theta4_4, phi4_4] = RotToYPR(R4_4)


%% 1.5 Rot to angle-axis with eigenvectors
%Q5.1
R5_1 = [1, 0, 0; 0, 0, -1; 0, 1, 0];
[h5_1, theta5_1] = RotToAngleAxis(R5_1)

[V_1,D_1] = eig(R5_1); %calcolo una matrice di autovalori di R (che è D_1) e poi un matrice le cui colonne sono gli autovettori di R (V_1)
idx = find(abs(diag(D_1) - 1) < 1e-3); %trovo l'autovalore che sia uguale o molto vicino a 1 vedendolo sulla base della differenza con 1.
                                       %idx individua la colonna contenente l'autovettore associato all'autovalore 1;
h5_1e = V_1(:,idx); %l'autovettore è definito sulla base dei valori sulla colonna idx 
h5_1e = h5_1e / norm(h5_1e)

%Q5.2
R5_2 = (1/9)*[4, -4, -7; 8, 1, 4; -1, -8, 4];
[h5_2, theta5_2] = RotToAngleAxis(R5_2)

[V_2,D_2] = eig(R5_2);
idx_2 = find(abs(diag(D_2) - 1) < 1e-3);
h5_2e = V_2(:,idx_2);
h5_2e = h5_2e / norm(h5_2e)
%Sono opposti ma è giusto perchè è un autovettore
