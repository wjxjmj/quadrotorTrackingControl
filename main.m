% This source code is written to implement flight simulations for one quadrotor
% Author: wjxjmj
% Email: wjxjmj@126.com
% Open Source License: GPL

% shall we go!
clear all
clc
% simulation paraments set up
dt=0.01;
stime=50;
loop=stime/dt;

% flocking paraments set up
d=3;
n=1;

% init state
s=zeros(12,n);
s(1:3,:)=unifrnd(-0,0,[3,n]);
s(4:6,:)=unifrnd(-0,0,[3,n]);
s(7,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
s(8,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
s(9,:)=unifrnd(-0.0*pi,0.0*pi,[1,n]);
x=s(1);y=s(2);z=s(3);
vx=s(4);vy=s(5);vz=s(6);
phi=s(7);theta=s(8);psi=s(9);
vphi=s(10);vtheta=s(11);vpsi=s(12);

% public virtual leadr init
xl=[0;0;0];
vl=[0;0;0];

%parameters for quadrotor
para.g=9.8;
para.m=1.2;
para.Iy=0.05;
para.Ix=0.05;
para.Iz=0.1;
para.b=10^-4;
para.l=0.5;
para.d=10^-6;
para.Jr=0.01;
para.k1=0.02;
para.k2=0.02;
para.k3=0.02;
para.k4=0.1;
para.k5=0.1;
para.k6=0.1;
para.omegaMax=330;

% history capture
xyHis=zeros(d,n+1,loop+1);
xyHis(:,:,1)=[xl s(1:3)];

%simulation start
hwait=waitbar(0,'½øÐÐÖÐ>>>>>>>>>>');

sp=1;
omegaHis=zeros(4,loop);
for t=1:loop
    
    %leader information generator
    if t/loop<0.1
        al=([0;0;sp]-vl);
    elseif t/loop<0.2
        al=([sp;0;0]-vl);
    elseif t/loop<0.4
        al=([0;sp;0]-vl);
    elseif t/loop<0.6
        al=([-sp;0;0]-vl);
    elseif t/loop<0.8
        al=([0;-sp;0]-vl);
    elseif t/loop<0.9
        al=([sp;0;sp]-vl);
    else
        al=([sp;0;0]-vl);
    end

    vl=vl+dt*al;
    xl=xl+dt*vl;

    % get motor speeds form the controller
    omega=quadrotor_controller(s,xl,vl,0,para,1,10);
    
    %record the speeds
    omegaHis(:,t)=omega;
    
    %send speeds of four motors to quadrotor and get its state
    s=quadrotor_kinematics(s,omega,para,dt);
    
    %recodrd the position of quadrotor at time t/loop*stime
    xyHis(:,:,t+1)=[xl s(1:3)];
    
    waitbar(t/loop,hwait,'simulating...');
end

close(hwait);
%show the animation of the flight process
figure(1)
plotHis3(xyHis,dt,-1,200)
axis equal
grid on

%show changes in motor speeds during the flight
figure(2)
plot(omegaHis')
grid on
