function dTh = ErrProp3D(X,dX,Th)
% Calculates Error Propogation for the
% 3D Registration Problem
%
% sum||xhati-[R(xi)+t]||^2
%
% Inputs:
% X  = [x1;x2;...;xn;xhat1;xhat2;...;xhatn] 
% dx = uncertainty of X
% Th = [theta;phi;rho;x;y;z]
%
% Outputs:
% dTh = uncertainty of Th
%
% Mili Shah
[m,n] = size(X);
if m<n, X = X';  end
[m,n] = size(dX);
if m<n, dX = dX'; end
n=length(X);

u  = [cos(Th(1))*cos(Th(2));cos(Th(1))*sin(Th(2));sin(Th(1))];
uT = [-sin(Th(1))*cos(Th(2));-sin(Th(1))*sin(Th(2));cos(Th(1))];
uP = [-cos(Th(1))*sin(Th(2));cos(Th(1))*cos(Th(2));0];
uTT= [-cos(Th(1))*cos(Th(2));-cos(Th(1))*sin(Th(2));-sin(Th(1))];
uTP= [sin(Th(1))*sin(Th(2));-sin(Th(1))*cos(Th(2));0];
uPP= [-cos(Th(1))*cos(Th(2));-cos(Th(1))*sin(Th(2));0];
uS = [0 -u(3) u(2);u(3) 0 -u(1);-u(2) u(1) 0];
uST= [0 -uT(3) uT(2);uT(3) 0 -uT(1);-uT(2) uT(1) 0];
uSP= [0 -uP(3) uP(2);uP(3) 0 -uP(1);-uP(2) uP(1) 0];
uSTT= [0 -uTT(3) uTT(2);uTT(3) 0 -uTT(1);-uTT(2) uTT(1) 0];
uSTP= [0 -uTP(3) uTP(2);uTP(3) 0 -uTP(1);-uTP(2) uTP(1) 0];
uSPP= [0 -uPP(3) uPP(2);uPP(3) 0 -uPP(1);-uPP(2) uPP(1) 0];

R  = cos(Th(3))*eye(3) + sin(Th(3))*uS + (1-cos(Th(3)))*u*u';
RT = sin(Th(3))*uST + (1-cos(Th(3)))*(uT*u'+u*uT');
RP = sin(Th(3))*uSP + (1-cos(Th(3)))*(uP*u'+u*uP');
RR = -sin(Th(3))*eye(3) + cos(Th(3))*uS + sin(Th(3))*u*u';
RTT= sin(Th(3))*uSTT + (1-cos(Th(3)))*(uTT*u'+2*uT*uT'+u*uTT');
RTP= sin(Th(3))*uSTP + (1-cos(Th(3)))*(uTP*u'+uT*uP'+uP*uT'+u*uTP');
RTR= cos(Th(3))*uST + (sin(Th(3)))*(uT*u'+u*uT');
RPP= sin(Th(3))*uSPP + (1-cos(Th(3)))*(uPP*u'+2*uP*uP'+u*uPP');
RRP= cos(Th(3))*uSP + (sin(Th(3)))*(uP*u'+u*uP');
RRR= -cos(Th(3))*eye(3) -sin(Th(3))*uS + cos(Th(3))*u*u';

dgdX = zeros(6,n);
dgdTh= zeros(6,6);

for i = 1:n/6
    dgdX(:,3*i-2:3*i) = [...
        2*(Th(4:6)-X(n/2+3*i-2:n/2+3*i))'*RT;...
        2*(Th(4:6)-X(n/2+3*i-2:n/2+3*i))'*RP;...
        2*(Th(4:6)-X(n/2+3*i-2:n/2+3*i))'*RR;...
        2*R];
    dgdX(:,n/2+3*i-2:n/2+3*i) = [...
        -2*(RT*X(3*i-2:3*i))';...
        -2*(RP*X(3*i-2:3*i))';...
        -2*(RR*X(3*i-2:3*i))';...
        -2*eye(3)];
    dgdTh = dgdTh + [...
        2*(RTT*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RTP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RTR*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RT*X(3*i-2:3*i))';...
        2*(RTP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RPP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RRP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RP*X(3*i-2:3*i))';...
        2*(RTR*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RRP*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RRR*X(3*i-2:3*i))'*(Th(4:6)-X(n/2+3*i-2:n/2+3*i)),...
        2*(RR*X(3*i-2:3*i))';...
        2*(RT*X(3*i-2:3*i)) 2*(RT*X(3*i-2:3*i)) 2*(RT*X(3*i-2:3*i)) 2*eye(3)];
end
dTh = -inv(dgdTh)*dgdX*dX;