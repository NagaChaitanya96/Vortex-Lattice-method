%% INFLUENCE COEFFICIENT MATRIX

clc
clear
format SHORTG

AR=60;                                                                      % Aspect Ratio = 60
c=1.0;                                                                      % Chord = 1 m
Vinf=50;                                                                    % Freestream velocity = 50 m/s
b=AR*c;
S=b*c;
alpha= linspace(0,5,21);                                                    %ANGLE OF ATTACK (0 to 5 degrees)
n=100;                                                                      %number of Horseshoe vortices= number of collocation points
N=n+1;                                                                      % No of trailing lines of points in which span is divided into
dy=b/N;
CL = zeros(21,1);
Cd_i = zeros(21,1);
DragInduced_InfCoeff = zeros(n);
Influence_coefficientMatrix = zeros(n);
vel_matrix = zeros(n,3);
Vel_induced = zeros(n,3);

%%%
for k =1:21
    V_inf_x=Vinf;
    

    zA=-(0.25*c*sind(alpha(k)))'*ones(N,1);
    zB=zA;
    zcp=-(0.25*c*sind(alpha(k)))'*ones(n,1);
    
    [GT,xcp] = Geometric_Locations(alpha(k),b,N,c,zA,zB);                   % CALLING TABLE FUNCTION OF GEOMETRIC LOCATIONS
    A=GT.A;                                                                 % OBTAINING (X1,Y1,Z1,X2,Y2,Z2) LOCATION FROM GT.
    B=GT.B;
    y=GT.y;
    
   [CP]=C_points(n,y,xcp,zcp);                                                 % Collocation points are obtained for the given input

    % NOW WE NEED VELOCITY FOR ALL HORSESHOE ELEMENTS 
    for j=1:n                                                               %(J represents collocation points variation)                                  
        V=zeros(n,3);                                                       % velocity of all vortices (U,V,W)
         cp=CP(j,:);                                                             
% EACH TIME THE ITERATION RUNS THE VELOCITY FOR N NUMBER 
% OF HORSESHOE VORTICES WILL BE FOUND FROM THE CODE AT
% 1 COLLOCATION POINT. AND FOR J tends from 1 to N.
% THE CODE IS REPEATED
        for i=1:n                                                           %(I-NO OF HORSESHOE ELEMENTS)                               
            G=1;                                                            % Velocity effect of n HRSE on 1 CP.  % GAMMA
            [Vel,Vel_D]=velocity(A,B,i,cp);                                 % Velocity effect of n HRSE on 1 CP
            vel_matrix(i,:)=Vel;                                            % COMBINING ALL THE VELOCITY VALUES DUE ON N COLLOCATION
                                                                            % POINTS IN A MATRIX 
            Vel_induced(i,:)=Vel_D;
        end
% Since n_cap is parallel to the origin the values it will have(0,0,1)        
        n_cp=[zeros(n,1) zeros(n,1) ones(n,1)];  
        % V.n_cap=A MATRIX
        Influence_coefficientMatrix(:,j)=dot(vel_matrix,n_cp,2);
        DragInduced_InfCoeff(:,j)=dot(Vel_induced,n_cp,2);

    end
    Free_stream=-Vinf.*sind(alpha(k))*ones(n,1);
    % Gamma_values=zeros(n,1);
    Gamma_Values=(Influence_coefficientMatrix\Free_stream);
    %Finding Induced Drag
    Winduced=DragInduced_InfCoeff*Gamma_Values;
    Aircraft_config=table(b,c,AR,alpha(k),n,N);
    
%     Values=table(CL,Cd_i,Gamma_Values);
    
    
    % LIFT
    rho=1.225;
    Qs=0.5*rho*Vinf^2*S;                                                    %product of dynamic pressure and wing planform area
    delta_L=rho*Gamma_Values*V_inf_x*dy;                                    %Lift contribution of each elemental area
    L=sum(delta_L);                                                         % Total Lift
    CL(k,1)=L/Qs;                                                           % Lift Coefficient

    % Drag
    deltaD=-rho*dy*Gamma_Values.*Winduced;                                  %Drag contribution of each elemental area
    D=sum(deltaD);                                                          %Total Drag
    Cd_i(k,1)=D/Qs;                                                         %Induced Drag 
end

%%  PLOTS
xflr1 = table2array(readtable('AR60.csv'));
CL1 = xflr1(1:21,3);
CD1 = xflr1(1:21,4);
AOA = xflr1(1:21,1);
CLCD = CL1./CD1;

%% 
figure
plot(Cd_i,CL,'k');                                                          % Cl vs Cd graph
xlabel('Cd');
ylabel('Cl');
title('Drag Polar Variation');
hold on;
plot(CD1,CL1,'r');
legend('Calculated values','XFLR values','location',"best");
hold off

figure
plot(alpha,CL,'k');
xlabel('Angle of Attack');
ylabel('Cl');
title('Cl vs AOA');
hold on;
plot(AOA,CL1,'r');
legend('Calculated values','XFLR values','location',"best");
hold off

figure
plot(alpha,CL./Cd_i,'k');
xlabel('Angle of Attack (degrees)');
ylabel('Cl/Cd');
title('Cl/Cd vs AOA');
hold on;
plot(AOA,CLCD,'r');
legend('Calculated values','XFLR values','location',"best");
hold off

figure
plot(alpha, Cd_i,'k');
xlabel('Angle of Attack');
ylabel('Cd');
title('Cd vs AOA');
hold on;
plot(AOA,CD1,'r');
legend('Calculated values','XFLR values','location',"best");
hold off

%% ERROR PLOTS
figure
CL_ERROR = (abs(CL1 - CL)./CL1)*100;
plot(AOA,CL_ERROR);
xlabel('Angle of Attack');
ylabel('CL error %');
title('Cl error % vs AOA');

figure
CD_ERROR = (abs(CD1 - Cd_i)./CD1)*100;
plot(AOA,CD_ERROR);
xlabel('Angle of Attack');
ylabel('Cd error %');
title('Cd error % vs AOA');


%% Function definition
function [GT,xcp] = Geometric_Locations(alpha,b,N,c,zA,zB)          %% defining inputs will lead to necessary input for every value

L=21*b ;                                                %LENGTH OF WAKE 
% x1=zeros(N,1);
% since wing is at alpha angle of attack, the projection of x1,x2,xcp will
% be on other plane by a distance zcp from the reference line (0,0,0)

x1=ones(N,1)*c/4.*(cosd(alpha));                           %locations at lifting line
x2= x1 + (L + (3*c/4));                              %location at wake 
n = N-1;
xcp =ones(n,1)*((c/4.*cosd(alpha)) + (c/2));            % taking wing reference as (0,0) xcp , zcp will vary as the AOA varies
y=linspace(-b/2,b/2,N)' ;                               % y location for equal parts                                           % assume z on wing itself
A=[x1 y zA];                                            %location of point (x1,y1,z1) on at c/4
B=[x2 y zB];                                            %location of point (x2,y2,z2) in wake
GT=table(x1,x2,y,A,B);
end

%% defining collocation points depending on span

function [CP]= C_points(n,y,xcp,zcp)    % wing from -b to b
ycp=zeros(n,1);                     % defining the size of matrix
for i=1:n
    ycp(i)=((y(i)+y(i+1))/2);       %storing iteration values   
end
   CP=[xcp,ycp,zcp];  
end

%% Velocity of N horseshoe vortices on each collocation point 

function  [Vel,Vel_D]=velocity(A,B,i,cp)
HORSESHOE_LOCATIONS=[B(i,:);A(i,:);A(i+1,:);B(i+1,:)];      % I defined as B as point 1 and A as point 2
R=HORSESHOE_LOCATIONS-cp;               %  r1,r2,r3,r4
r1=R(1,:);m1=norm(r1);
r2=R(2,:);m2=norm(r2);
r3=R(3,:);m3=norm(r3);
r4=R(4,:);m4=norm(r4);


r12=r1-r2; 
r23=r2-r3;
r34=r3-r4;
r41=r4-r1;

cr12=cross(r1,r2);
m12=norm(cr12);
L12=(dot(r12,r1)/m1-dot(r12,r2)/m2);
UVW12=(1/(4*pi))*L12*(cr12/(m12)^2);

cr23=cross(r2,r3);
m23=norm(cr23);
L23=(dot(r23,r2)/m2-dot(r23,r3)/m3);
UVW23=(1/(4*pi))*L23*(cr23/(m23)^2);

cr34=cross(r3,r4);
m34=norm(cr34);
L34=(dot(r34,r3)/m3-dot(r34,r4)/m4);
UVW34=(1/(4*pi))*L34*(cr34/(m34)^2);


cr41=cross(r4,r1);
m41=norm(cr41);
L41=(dot(r41,r4)/m4-dot(r41,r1)/m1);
UVW41=(1/(4*pi))*L41*(cr41/(m41)^2);


U_induced=UVW12+UVW34;
b1(i,:)=U_induced;
UVW=UVW12+UVW23+UVW34+UVW41;            %(U,V,W) OVERALL
a1(i,:)=UVW; %
Vel=a1(i,:);
Vel_D=b1(i,:);
end


