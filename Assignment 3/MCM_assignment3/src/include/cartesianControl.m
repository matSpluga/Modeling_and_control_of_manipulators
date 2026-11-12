%% Kinematic Model Class - GRAAL Lab
classdef cartesianControl < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        k_a
        k_l
    end

    methods
        % Constructor to initialize the geomModel property
        function self = cartesianControl(gm,angular_gain,linear_gain)
            if nargin > 2
                self.gm = gm;
                self.k_a = angular_gain;
                self.k_l = linear_gain;
            else
                error('Not enough input arguments (cartesianControl)')
            end
        end
            %% getCartesianReference function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % x_dot : cartesian reference for inverse kinematic control

            function [x_dot]=getCartesianReference(self, bTg)
            b_e = self.cartesianError(bTg);

            LAMBDA = [self.k_a * eye(3), zeros(3,3); zeros(3,3), self.k_l * eye(3)];
            x_dot = LAMBDA * b_e;
            end
        
            %% cartesianError function
            % Inputs :
            % bTg : goal frame
            % Outputs :
            % b_e : cartesian error

            function [b_e] = cartesianError(self, bTg)
            bTt = self.gm.getToolTransformWrtBase();
            tTg = inv(bTt)*bTg; %bTg=bTt*tTg
            bRt = bTt(1:3,1:3); %The rotation matrix itself is not sufficient because it doesn't give us enough information, we need to transform it into its angle-axis parameters.
            [h, theta] = RotToAngleAxis(tTg(1:3,1:3));

            % position error expressed in base frame
            r = bTg(1:3, 4) - bTt(1:3, 4); 

            % angular error expressed in base frame
            t_rho = h*theta; % Angle-axis vector
            b_rho = bRt*t_rho; % Vector oriented wrt the base frame

            b_e = [b_rho; r];   % cartesian error ([angolar part; linear part])
        end
    end
end

