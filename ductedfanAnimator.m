classdef ductedfanAnimator < handle
    
    properties
        X
        U
        lastPlot
        vecMovie
        param
        fig = NaN;
    end
    
    methods
        function obj = ductedfanAnimator(fig)
             if exist('fig','var')
                  obj.fig = fig;
             end
        end
                
        function animate(self, t, X, factor)
            if ~exist('factor','var')
                factor = 1;
            end
            if(factor<1)
                factor = 1;
                warning('factor has been clamped to 1. factor must be (factor>=1)!')
            end
            if ~isgraphics(self.fig)
                self.fig = figure();
            end
          
            % get lims
            x = X(:,1);
            y = X(:,2);
            th = X(:,3);
            
            inc = 0.1;
            indx = [min(x)-inc, max(x)+inc];
            indy = [min(y)-inc, max(y)+inc];
                        
            N = size(X,1);
            
            P = [];
            hold on
            tmp = 1:factor:N;
            if tmp(end) ~= N
                tmp = [tmp, N];
            end
            k = 1;
            
            for i = tmp
                delete(P);
                P = self.snapshot(X(i,:)');
                title("Time = " + sprintf('%.2f',t(i)) + " sec")
                axis equal
%                 xlim(indx)
                ylim(indy)
                drawnow
                vec(i) = getframe(self.fig);
                k = i;
            end
            self.vecMovie = vec;
        end
        
        function animateInput(self, t, X, U, factor)
            if ~exist('factor','var')
                factor = 1;
            end
            if(factor<1)
                factor = 1;
                warning('factor has been clamped to 1. factor must be (factor>=1)!')
            end
            if ~isgraphics(self.fig)
                self.fig = figure();
            end
          
            % get lims
            x = X(:,1);
            y = X(:,2);
            th = X(:,3);
            
            inc = 0.1;
            indx = [min(x)-inc, max(x)+inc];
            indy = [min(y)-inc, max(y)+inc];
                        
            N = size(X,1);
            
            P = [];
            hold on
            tmp = 1:factor:N;
            if tmp(end) ~= N
                tmp = [tmp, N];
            end
            k = 1;
            
            for i = tmp
                delete(P);
                P = self.snapshot(X(i,:)', U(i,:)');
                title("Time = " + sprintf('%.2f',t(i)) + " sec")
                axis equal
%                 xlim(indx)
                ylim(indy)
                drawnow
                vec(i) = getframe(self.fig);
                k = i;

            end
            self.vecMovie = vec;
        end

        function P = snapshot(self, x_, u_, ax)
            if ~exist('ax', 'var')
                ax = gca;
            end
            if ~exist('u_', 'var')
                u_ = [];
            end
                        
            x = x_(1);
            y = x_(2);
            th = x_(3);

                       
            w = 0.05;
            h = w*2;
            mg = 9.8*6/h;
            fan_poly = polyshape([0,w,w,0],[0,0,h,h]);
            fan_poly = rotate(fan_poly, th*180/pi, [w/2,h/2]);
            fan_poly = translate(fan_poly, [x-w/2,y-h/2]);
            
            % r and g axis
            r1 = [x;y];
            r2 = r1 + self.rotationR(th)*[w*1.2;0];
            
            g1 = [x;y];
            g2 = g1 + self.rotationR(th)*[0;w*1.2];
                        
            P(1) = plot(fan_poly, 'facecolor', 'k');
            P(2) = plot([r1(1), r2(1)],[r1(2), r2(2)], 'color', 'red');
            P(3) = plot([g1(1), g2(1)],[g1(2), g2(2)], 'color', 'green');
            
            if ~isempty(u_)
                % plot control
                tau = u_(1);
                psi = u_(2);
                ut = [x;y] + self.rotationR(th)*[0;-w/2];
                ur = ut - self.rotationR(th)*self.rotationR(-psi)*[0;tau/mg];
                P(5) = quiver( ur(1),ur(2),ut(1)-ur(1),ut(2)-ur(2),0 ,'color', 'yellow', 'linewidth', 2, 'AutoScaleFactor', 1.2);
            end
            
        end
        
        function R = rotationR(~,th)
            R = [cos(th), -sin(th);
                 sin(th), cos(th)];
        end
        
        
        function exportMovie(self, txt, fr)
            num = length(self.vecMovie);
            
            for i = 1:(num+10)
                if i <= 10
                    vec(i) = self.vecMovie(1);
                else
                    vec(i) = self.vecMovie(i-10);
                end
            end
%             myWriter = VideoWriter(txt, 'MPEG-4');
            myWriter = VideoWriter(txt);
            myWriter.FrameRate = fr;
            
            open(myWriter);
            writeVideo(myWriter, vec);
            close(myWriter)
        end
    end
end

