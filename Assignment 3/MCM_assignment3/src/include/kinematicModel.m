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

        function [bJi] = getJacobianOfLinkWrtBase(self, i)
            %%% getJacobianOfJointWrtBase
            % This method computes the Jacobian matrix bJi of joint i wrt base.
            % Inputs:
            % i : joint index ;

            % The function returns:
            % bJi
            bJi = zeros(6, i);
            bTi = self.gm.getTransformWrtBase(i);
            for j = 1:i
                bTj = self.gm.getTransformWrtBase(j);
                if self.gm.jointType(j) == 0
                    bJi(1:3, j) = bTj(1:3, 3);
                    bJi(4:6, j) = cross(bTj(1:3, 3), (bTi(1:3, 4) - bTj(1:3, 4)));
                end
                if self.gm.jointType(j) == 1
                    bJi(1:3, j) = [0; 0; 0];
                    bJi(4:6, j) = bTj(1:3, 3);
                end
            end
        end

        
        %% Update Jacobian function
        % The function update:
        % - J: Jacobian for the tool
        
        function updateJacobian(self)
            self.J = getJacobianOfToolWrtBase(self);
        end
        % Cambiabile: posso usare getJacobianOfLinkWrtBase(jointNumber) e
        % pre moltiplicare per la rigid body jacobian
        function [J_t] = getJacobianOfToolWrtBase(self)
            J_t = getJacobianOfLinkWrtBase(self, self.gm.jointNumber);
            r_t_e = self.gm.eTt(1:3, 4);
            r_t_e_skew = [0, -r_t_e(3), r_t_e(2);
                          r_t_e(3), 0, -r_t_e(1);
                          -r_t_e(2), r_t_e(1), 0];
            rigid_body_J = [eye(3), zeros(3,3)
                            -r_t_e_skew, eye(3)];
            J_t = rigid_body_J * J_t;
        end
    end
end

