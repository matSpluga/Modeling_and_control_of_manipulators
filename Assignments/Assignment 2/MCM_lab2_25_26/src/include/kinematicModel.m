%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end

        function bJi = getJacobianOfLinkWrtBase(self, j)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint indnex ;

            % The function returns:
            % bJi
            Tn = self.gm.getTransformWrtBase(j);
            
            for i = 1:j

                Ti = self.gm.getTransformWrtBase(i);% I compute this because, fot the definition of the Jacobian, I need the values of the z-axis (for the angular parts) (ki) 
                bJi(:, i) = zeros(6, 1); % Initialize the Jacobian for i-joint (i-esima colonna)

                if(self.gm.jointType(i) == 0)   %Rotational joint
    
                    bJi(1:3, i) = Ti(1:3, 3); %z-axis of the i joint %angular part
                    bJi(4:6, i) = cross(Ti(1:3, 3), Tn(1:3, 4) - Ti(1:3, 4)); %linear part (ki x rj/i)
                end
    
                if(self.gm.jointType(i) == 1)   %Prismatic joint 
                    bJi(4:6, i) = Ti(1:3, 3); %linear part (angular part = 0)
                end
            end %These jacobians, multiplied by vector q, will give me the angular velocities and the linear ones of a certain joint with respect to the base.

            self.J = bJi;
        end

        %% Update Jacobian function
        function updateJacobian(self)
            % The function update:
            % - J: end-effector jacobian matrix
            r = [0, 0, 0];

            self.getJacobianOfLinkWrtBase(self.gm.jointNumber);
            
            J_A = self.J(1:3, :); %After calling the previous function to compute the jacobian of the ee wrt the base frame, the new jacobian is stored in self.J. Then I store those values in J_A and J_L.
            J_L = self.J(4:6, :);
            
            Rcross = [0 -r(3) r(2); r(3) 0 -r(1); -r(2) r(1) 0]; %cross-product operator
            J_L = J_L - Rcross * J_A; % Update the Jacobian for the end-effector
            self.J(4:6, :) = J_L; % Store the updated Jacobian back into the property
        end
    end
end

