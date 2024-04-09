classdef ductedfan < handle
    
    properties
        param = struct('m', 4.25,...
                       'd', 0.1,...
                       'g', 9.8,...
                       'r', 0.26,...
                       'J', 0.0475,...
                       'gamma_x', 1,...
                       'gamma_r', 1,...
                       'gamma_a', 1)
        
        tSim
        xSim            % states are x, y, theta, dx, dy, dtheta
        xdotSim
        uSim            % controls are tau and psi
        
        % Most likely useful for other implementations, possibly as
        % subclss. Think through how coding design permits the different
        % versions to easily run.
        A
        B
        Alin
        Blin
        Alin_modified
        Blin_modified
        P
        Klqr
    end
    
    methods
        %========================== ductedfan ==========================
        %
        % Constructor for the inverted pendulum on a cart instance.
        %
        function obj = ductedfan(param)
             if exist('param','var')
                  obj.param = param;
             end
        end
        
        %============================ runSim ===========================
        %
        % Abstracted interface to run the simulation of the ducted
        % fan. Uses a format similar to odeXX functions for specifying the
        % initial value problem (IVP) with the addition of a control
        % function.
        %
        function [tSim, xSim, xdotSim, uSim] = runSim(self, tspan, x0, u)
            Q=diag([1,1,1,1,1,1]);
            [self.Alin,self.Blin]=self.linearize(x0,u(0,x0));
            [~,~,self.Klqr]=care(self.Alin,self.Blin,Q);
            % To put more weight on x
            Q=diag([30,1,1,30,1,1]);
            [self.P,~,~]=care(self.Alin,self.Blin,Q);

            sys = @(t,x) self.dynamics_nonlinear(t, x, u);

            % Simulate the system.
            [tSim, xSim] = ode45(sys, tspan, x0);

            % Use simulation data to recover controls and diff eq.
            % Data is row-wise.
            num     = length(tSim);
            uSim    = zeros(num,3);
            xdotSim = zeros(size(xSim));
            %r=@(t) [3;4];
            r=@(t) [5*cos(t/10);5*sin(t/10)];
            phi=[0;0;1];
            
            for i = 1:num
                x = xSim(i,:)';
                t = tSim(i);
                kx=reshape(xSim(i,13:30),6,3);
                kr=reshape(xSim(i,31:36),2,3);
                alpha_hat=reshape(xSim(i,37:45),3,3);
                uSim(i,:) = (kx'*xSim(i,1:6)'+kr'*r(t)-alpha_hat'*phi)';
                xdotSim(i,:) = self.dynamics_nonlinear(t, x, u)';
            end
            
            % Store variables in case needed for later.
            self.tSim = tSim;
            self.xSim = xSim;
            self.xdotSim = xdotSim;
            self.uSim = uSim;
        end
        
        %=========================== dynamics ==========================
        %
        % Compute the dynamics of the system under the applied control.
        %
        function xdot = dynamics(self, t, xvec, uvec)
%             kx=reshape(xvec(13:24),6,2);
%             kr=reshape(xvec(25:28),2,2);

            X=xvec(1:6);
            Xm=xvec(7:12);

            
%             gamma_x= self.param.gamma_x;
%             gamma_r= self.param.gamma_r;

            Am=self.Alin-self.Blin*self.Klqr;
            Bm=self.Blin;

            r=@(t) [3;4];
            %r = @(t) [5*cos(t/10);5*sin(t/10)];

%             xdot(13:24)=-gamma_x*X*e'*self.P*self.Blin; %kx'
%             xdot(25:28)=-gamma_r*r(t)*e'*self.P*self.Blin; %kr'

            % Static Linear Feedback part
            kx=(pinv(self.Blin)*(Am-self.Alin))';
            kr=(pinv(self.Blin)*Bm)';

            u=kx'*X+kr'*r(t);
            
%             xdot(1:6)=self.Alin*X+self.Blin*u; %X
%             xdot(7:12)=Am*Xm+Bm*r(t); %Xm
            
            %For parameter mismatched
            xdot(1:6)=self.Alin_modified*X+self.Blin_modified*u; %X
            xdot(7:12)=Am*Xm+Bm*r(t); %Xm

            xdot=xdot';
            
            
        end

        function xdot = dynamics_nonlinear(self, t, xvec, uvec)
            th  = xvec(3);
 
            kx = reshape(xvec(13:30),6,3);
            kr = reshape(xvec(31:36),2,3);
            alpha_hat = reshape(xvec(37:45),3,3);

            X=xvec(1:6);
            Xm=xvec(7:12);

            e=X-Xm;
            
            m = self.param.m;
            d = self.param.d;
            g = self.param.g;
            r = self.param.r;
            J = self.param.J;
     
   
            gamma_x= self.param.gamma_x;
            gamma_r= self.param.gamma_r;
            gamma_a= self.param.gamma_a;

            self.A = [0 0 0 1 0 0;
                      0 0 0 0 1 0;
                      0 0 0 0 0 1;
                      0 0 0 -d/m 0 0;
                      0 0 0 0 -d/m 0;
                      0 0 0 0 0 0];
            self.B = [0 0 0;
                      0 0 0;
                      0 0 0;
                      cos(th)*(1/m) -sin(th)/m 0;
                      sin(th)*(1/m) cos(th)/m -1;
                      -r/J 0 0];
                  
            

          
        %Parameter mismatched
          m = 4.4;
          r = 0.3;
          J = 0.04;
                  
            A_mismatched = [0 0 0 1 0 0;
                      0 0 0 0 1 0;
                      0 0 0 0 0 1;
                      0 0 0 -d/m 0 0;
                      0 0 0 0 -d/m 0;
                      0 0 0 0 0 0];
            B_mismatched = [0 0 0;
                      0 0 0;
                      0 0 0;
                      cos(th)*(1/m) -sin(th)/m 0;
                      sin(th)*(1/m) cos(th)/m -1;
                      -r/J 0 0];


            alpha = [0 0 0;
                     0 0 0;
                     0 0 g];
            phi=[0;0;1];

            Am=self.Alin-self.Blin*self.Klqr;
            Bm=self.Blin;

            %r=@(t) [3;4];
            r=@(t) [5*cos(t/10);5*sin(t/10)];

            xdot(13:30)=-gamma_x*X*e'*self.P*self.B; %kx'
            xdot(31:36)=-gamma_r*r(t)*e'*self.P*self.B; %kr'
            xdot(37:45)=gamma_a*phi*e'*self.P*self.B; %alpha_hat'
            
%             kx=(pinv(self.B)*(Am-self.A))';
%             kr=(pinv(self.B)*Bm)';
           

            %u=kx'*X+kr'*r(t)-alpha'*phi; %alpha_hat=alpha for static controller
            u=kx'*X+kr'*r(t)-alpha_hat'*phi;  % Adaptive controller
            
%             xdot(1:6)=self.A*X+self.B*(u+alpha'*phi);%X
%             xdot(7:12)=Am*Xm+Bm*r(t);
            
            %%For parameter mismatched
            xdot(1:6)=A_mismatched*X+B_mismatched*(u+alpha'*phi);%X
            xdot(7:12)=Am*Xm+Bm*r(t);

            xdot=xdot';
            
        end

        %=========================== rotation ==========================
        %
        % Helper function. Not best place to put it, but preserves
        % self-contained nature of code.
        %
        function R = rotationR(~,th)
            R = [ cos(th), -sin(th) ;
                  sin(th),  cos(th) ];
        end
        
        %========================== linearize ==========================
        %
        % Returns the linearized system equations for use by control
        % synthesis or model reference schemes.
        %
        function [A,B] = linearize(self, x0, u0)
          % TO BE FILLED OUT IF WORTH DOING.
          m = self.param.m;
          d = self.param.d;
          g = self.param.g;
          r = self.param.r;
          J = self.param.J;


          
          tau0=u0(1);
          psi0=u0(2);
          th0=x0(3);

          A=[zeros(2) [0;0] eye(2) [0;0];
              [0 0] 0 [0 0] 1;
              zeros(2) [-cos(psi0)*(tau0/m);sin(psi0)*(tau0/m)] -d/m*eye(2) [0;0];
              [0 0] 0 [0 0] 0];
           B=[[0;0] [0;0];
              0 0;
              1/m -th0/m;
              th0/m 1/m;
              -r/J 0];
          
          %%%For parameter mismatched
          m = 3.6;
          r = 0.2;
          J = 0.03;

         
          self.Alin_modified=[zeros(2) [0;0] eye(2) [0;0];
              [0 0] 0 [0 0] 1;
              zeros(2) [-cos(psi0)*(tau0/m);sin(psi0)*(tau0/m)] -d/m*eye(2) [0;0];
              [0 0] 0 [0 0] 0];
          self.Blin_modified=[[0;0] [0;0];
              0 0;
              1/m -th0/m;
              th0/m 1/m;
              -r/J 0];
        end
        
        
    end
end

