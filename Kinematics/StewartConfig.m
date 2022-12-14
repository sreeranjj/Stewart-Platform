%This program calls the function servoangles by giving the orientation 
%Give angles in degrees and translation in meters
x=0.0;
y=0.0;
vert=0.0;
z=0.293+vert; %29.3cm is the distance between two platforms
phi=0;
theta=0;
psi=0;

%length is a vector containing magnitude of the lengths of each leg
%bA contains the components of the 6 leg vectors
%servoang is a vector which contains all the six servo angles
[length,bA,servoang]=servoangles(x,y,z,psi,theta,phi);

%(alpa*180/pi)
lo=[0.2062,0.2062,0.2062,0.2062,0.2062,0.2062]';   %Length of the legs initially before motion starts
%lo=[0.2544,0.2544,0.2544,0.2544,0.2544,0.2544]';
%lo=[0.3793,0.3793,0.3793,0.3793,0.3793,0.3793]';
%deltal=length-lo;
%deltal;
%a=0.05;
%alpha=zeros(6,1);
%for i=1:6
% if(deltal(i,1)<0)
%     deltal(i,1)=-1*deltal(i,1);
% end
% 
% alpha(i,1)=asin(deltal(i,1)/a);
%end
%alpha;
%deltal;
%***********************************************************************

