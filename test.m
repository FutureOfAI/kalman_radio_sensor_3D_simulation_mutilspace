close all;
clear all;
clc;
% dbax = (bx-bx_h)/g;
% Ebax = mean(dbax);
% dbay = (by-by_h)/g;
% Ebay = mean(dbay);
% dbaz = (bz-bz_h)/g;
% Ebaz = mean(dbaz);
% dbgx = (bgx0-bgx_h)*r2d;
% Ebgx = mean(dbgx);
% dbgy = (bgy0-bgy_h)*r2d;
% Ebgy = mean(dbgy);
% dbgz = (bgz0-bgz_h)*r2d;
% Ebgz = mean(dbgz);
% dphi = dq11*r2d;
% Ephi = mean(dphi);
% dtheta = dq21*r2d;
% Etheta = mean(dtheta);
% dpsi = dq31*r2d;
% Epsi = mean(dpsi);

% x = rand(9,2); 
% n = 20; 
% ndx = randperm(numel(x), n); 
% [row,col] = ind2sub(size(x), ndx); 

% x = linspace(-2,2,30);
% y = linspace(-2,2,30);
% [xx,yy] = meshgrid(x,y);
% zz = xx.*exp(-xx.^2-yy.^2);
% mesh(xx,yy,zz);

% x = 1:10;
% y = 11:20;
% z = x'*y;
% c = randi(10,10);
% surf(x,y,z,c)

d2r = (pi/180);
r2d = (180/pi);

xr1 = 0;% in meter
yr1 = 0;% in meter
zr1 = 2.52;% in meter

xr2 = 5.79;
yr2 = 0;
zr2 = 2.52;

% c1
xr3 = 0;% in meter
yr3 = 5.79*cos(0*d2r);% in meter
zr3 = 2.52;
% c2
% xr3 = 4.44;% in meter
% yr3 = 5.79*cos(50*d2r);% in meter
% zr3 = 2.52;
% c3
% xr3 = 5.7;% in meter
% yr3 = 5.79*cos(80*d2r);% in meter
% zr3 = 2.52;

xr4 = 0.41;
yr4 = 0.64;
% c2
zr4 = 0.464;% in meter
% c3
% zr4 = 1.464;% in meter  

x = [1.6 2.46 3.32 4.19];
y = [-2.8 -0.93 0.94 2.81 4.69 6.56];
[xx,yy] = meshgrid(x,y);
% c1
c = [1.1404 0.8491 0.4953 0.4671 0.5166 0.5457 
    0.9727 0.7176 0.2988 0.2221 0.1786 0.4315 
    0.9517 0.6842 0.3508 0.2512 0.5815 0.4992 
    0.973 0.779 0.3384 0.3316 0.412 0.4817]';
% c2
% c = [0.8992 0.4743 0.393 0.3308 0.9293 1.8334 
%     0.63 0.2662 0.1981 0.258 0.884 1.3874 
%     0.3792 0.367 0.4667 0.3435 0.5449 1.1082 
%     0.2221 0.5466 0.5931 0.5154 0.2053 1.0284]';
% c3
% c = [1.606 0.9152 0.4232 0.6616 1.0443 1.4574 
%     1.4305 0.7205 0.6633 0.4561 0.7867 1.1999 
%     0.9721 0.7681 0.6676 0.4761 0.5683 1.1923 
%     0.4648 0.3641 0.5285 0.7395 0.7101 0.9833]';
surf(xx,yy,c);
% view(80,30)
hold on
plot3(xr1,yr1,zr1,'r*',xr2,yr2,zr2,'r*',xr3,yr3,zr3,'r*',xr4,yr4,zr4,'r*','linewidth',2)
xlabel('x-axis(m)','FontSize',14)
ylabel('y-axis(m)','FontSize',14)
zlabel('z-axis(m)','FontSize',14)
text(xr1,yr1,zr1,' (0,0,2.52)')
text(xr2,yr2,zr2,' (5.79,0,2.52)')
text(xr3,yr3,zr3,' (0,5.44,2.52)')
% text(xr3,yr3,zr3,' (4.44,3.72,2.52)')
% text(xr3,yr3,zr3+0.15,' (5.7,1,2.52)')
text(xr4,yr4,zr4,' (0,0,0.46)')
% text(xr4,yr4,zr4,' (0.41,0.64,0.46)')
% text(xr4-1.5,yr4,zr4,' (0.41,0.64,1.46)')
% title(['Anchors distribution with psi=40' char(176) ' phi=20' char(176) ''],'FontSize',18);
cb = colorbar;

c1 = [1.1404 0.8491 0.4953 0.4671 0.5166 0.5457 0.9727 0.7176 0.2988 0.2221 0.1786 0.4315 0.9517 0.6842 0.3508 0.2512 0.5815 0.4992 0.973 0.779 0.3384 0.3316 0.412 0.4817];
c2 = [0.8992 0.4743 0.393 0.3308 0.9293 1.8334 0.63 0.2662 0.1981 0.258 0.884 1.3874 0.3792 0.367 0.4667 0.3435 0.5449 1.1082 0.2221 0.5466 0.5931 0.5154 0.2053 1.0284];
c3 = [1.3698 0.7132 0.1669 0.9016 1.8024 1.236 0.9454 0.6875 0.1925 0.8135 1.4813 1.8194 0.8783 0.7563 0.2325 0.5556 1.3113 2.0363 0.9056 0.6532 0.2665 0.5131 1.0739 1.7982];
c4 = [1.606 0.9152 0.4232 0.6616 1.0443 1.4574 1.4305 0.7205 0.6633 0.4561 0.7867 1.1999 0.9721 0.7681 0.6676 0.4761 0.5683 1.1923 0.4648 0.3641 0.5285 0.7395 0.7101 0.9833];
B = [sort(abs(c1')) sort(abs(c2')) sort(abs(c3')) sort(abs(c4'))];
N = length(B);
number = fix(0.6827*N);
B1 = [B(number,1) B(number,2) B(number,3) B(number,4)]


an1 = [0,0,2.52];
an2 = [5.79,0,2.52];
an3 = [5.79*sin(20*d2r),5.79*cos(20*d2r),2.52];
an4 = [0.41,0.64,0.464];
k = an2-an1;
l = an3-an1;
m = an4-an1;
psi=atan2(norm(cross(k,l)), dot(k,l))*r2d
alpha=atan2(norm(cross(k,m)), dot(k,m))*r2d
beta=atan2(norm(cross(m,l)), dot(m,l))*r2d

%// normal vector to plane ABC 
N = cross(k, l); 
%// angle between plane and line, equals pi/2 - angle between D-E and N 
phi = 90-abs(pi/2 - acos(dot(m, N)/norm(N)/norm(m)))*r2d


% [X,Y,Z] = sphere(19);
% r = 25;
% X2 = X * r;
% Y2 = Y * r;
% Z2 = Z * r;
% surf(X2,Y2,Z2)
% axis equal