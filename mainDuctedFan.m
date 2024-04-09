%============================= mainDuctedFan =============================
%
% Code stub for the ducted fan project. Demonstrates basic functionality of
% the associated classes.
%
% The controller is set to be an anonymous function for provision of
% executable code.  It should be remapped to a better function that
% implements some form of feedback or error feedback control.  In some
% cases, an anonymous function will suffice.  For implementation of
% adaptive control laws, this function may have to parse the state x and
% unpack it to synthesize the control law.  Such synthesis might also
% include the adaptive laws.
%
% It should even be possible to define the control law as a member function
% of the ductedfan class.  Consider how to best augment this implementation 
% with an adaptive controller.  It might be by creating a sub-class with
% this expanded functionality and larger state size, or it might be by
% adding in new member functions.  Up to you. Just be clean about it.
%

if (~exist('prettyup'))
  prettyup = true;                  % Should environment be cleared out?
end                                 % Affects repeat runs. Set as you please.

if (prettyup)                       % If prettyup should be done, do so.
  clearEnv;
  prettyup = true;                  
end                                 % otherwise, environment stays as is.


robo = ductedfan();

kx0=zeros(6,3);
kr0=zeros(2,3);
a0=zeros(3,3);

u = @(t,x) [1.01*robo.param.m*robo.param.g;0];
%adaptive linear controller initial condition
x0 = [0;0;0;0;0;0 ;0;0;0;0;0;0 ;reshape(kx0,numel(kx0),1);reshape(kr0,numel(kr0),1);reshape(a0,numel(a0),1)];           % Lame initial condition.
%x0 = [-0.5;0.2;0;0;0;0 ;-0.5;0.2;0;0;0;0;reshape(kx0,numel(kx0),1);reshape(kr0,numel(kr0),1)]; %initial condition for non
% adaptive and non linear
%x0 = [0;0;0;0;0;0 ;0;0;0;0;0;0];

tspan = [0,100];
[tSim, xSim, dxSim, uSim] = robo.runSim(tspan, x0, u);

fh = figure(1); clf;
ani = ductedfanAnimator(fh);
ani.animateInput(tSim, xSim, uSim, 50);

kx0=xSim(end,13:30);
kr0=xSim(end,31:36);
a0=xSim(end,37:45);

x0 = [0;0;0;0;0;0 ;0;0;0;0;0;0 ;reshape(kx0,numel(kx0),1);reshape(kr0,numel(kr0),1);reshape(a0,numel(a0),1)];           % Lame initial condition.

[tSim, xSim, dxSim, uSim] = robo.runSim(tspan, x0, u);

fh = figure(2); clf;
ani = ductedfanAnimator(fh);
ani.animateInput(tSim, xSim, uSim, 50);

optionTitle={'FontWeight', 'bold', 'FontSize', 20, 'FontName', 'Times New Roman', 'Interpreter', 'tex'};

figure(3);
subplot(3,1,1);
plot(tSim, xSim(:,1),'b',tSim,xSim(:,7), 'r');
title("x,y and theta", optionTitle{:})
xlabel('t');
ylabel('x(t)');
legend('x','xm');

subplot(3,1,2);
plot(tSim, xSim(:,2),'b', tSim,xSim(:,8), 'r');
xlabel('t');
ylabel('y(t)');
legend('y','ym');

subplot(3,1,3);
plot(tSim,xSim(:,3) , 'b',tSim,xSim(:,9), 'r');
xlabel('t');
ylabel('theta(t)');
legend('th','thm');

figure(4);
A = uSim(:,1);
B = uSim(:,2);
tau = sqrt(A.^2+B.^2);
psi = atan(A./B);
subplot(2,1,1);
plot(tSim, tau,'b');
title("Control", optionTitle{:})
xlabel('t');
ylabel('Tau');

subplot(2,1,2);
plot(tSim, psi,'b');
xlabel('t');
ylabel('Psi');

figure(5)
plot(xSim(:,1), xSim(:,2), 'b',xSim(:,7), xSim(:,8),'r');
title("Trajectory and Orientation", optionTitle{:})
hold on;
lim=[-6 6];
diff=lim(2)-lim(1);
for temp = 1:floor(length(tSim)/50)
    quiver(xSim(50*temp,1),xSim(50*temp,2),diff/10*cos(xSim(50*temp,3)),diff/10*sin(xSim(50*temp,3)),'b', 'MaxHeadSize',2);
    quiver(xSim(50*temp,7),xSim(50*temp,8),diff/10*cos(xSim(50*temp,9)),diff/10*sin(xSim(50*temp,9)),'r','MaxHeadSize',2);
end
xlabel('x');
ylabel('y');
legend('System','Model');
xlim(lim);
ylim(lim);
hold off;

figure(6);
plot(tSim, xSim(:,1)-xSim(:,7),tSim,xSim(:,2)-xSim(:,8),tSim, xSim(:,3)-xSim(:,9));
hold on;
plot(tSim, xSim(:,4)-xSim(:,10),tSim,xSim(:,5)-xSim(:,11),tSim, xSim(:,6)-xSim(:,12));
legend('x-xm', 'y-ym', 'th-thm', "x'-xm'","y'-ym'","th'-thm'");
title("Error", optionTitle{:})
xlabel('t');
ylabel('Error');
hold off;

figure(7)
subplot(3,1,1);
title("Gain", optionTitle{:})
hold on;
for temp = 1:18
    plot(tSim,xSim(:,12+temp));
end
xlabel('t');
ylabel('kx');
hold off;

subplot(3,1,2);
hold on;
for temp = 1:6
    plot(tSim,xSim(:,30+temp));
end
xlabel('t');
ylabel('kr');
hold off;

subplot(3,1,3);
hold on;
for temp = 1:9
    plot(tSim,xSim(:,36+temp));
end
xlabel('t');
ylabel('alpha hat');
hold off;
