%Parametere
windSpeed = 1;
windAngle = 0;
currentSpeed = 1;
currentAngle = 90;
% Matrise med veipunkt 
GPS = [0 0; 0 10; 5 10; 5 0; 10 0; 10 10; 15 10; 15 0; 20 0; 20 10; 25 10; 25 0; 30 0; 30 10; 35 10; 
    35 0; 40 0; 40 10; 45 10; 45 0; 50 0; 50 10; 55 10]*5;
i = 1;
Xc = GPS(1,1);
Yc = GPS(1,2);
X_k_1 = GPS(i,1);
Y_k_1 = GPS(i,2);
SatelittMSG = 0;
Rv = 6;
Rk = 5;
vesselSpeed = 3;

w=0;
t = 0;
YAWangle = 7/4*pi;
%% 
while(t < 600 && w<1)
    
if SatelittMSG == 1
    i=25;
end

    
    r = Rv;
    t = t + 1;
    
    X_k = GPS(i+1,1);
    Y_k = GPS(i+1,2);

%% Navigering og plotting
LOSpointAngle = atan((Y_k - Yc)/(X_k-Xc));
if LOSpointAngle < 0
    LOSpointAngle = LOSpointAngle + 2*pi;
end

if (Y_k - Yc) > 0 && pi < LOSpointAngle && LOSpointAngle < 2*pi
    LOSpointAngle = LOSpointAngle - pi;
    
elseif (Y_k - Yc) < 0 && 0 < LOSpointAngle && LOSpointAngle < pi
        LOSpointAngle = LOSpointAngle + pi;
end

corrAngle = LOSpointAngle - YAWangle;
if corrAngle > pi
    corrAngle = corrAngle - 2*pi;
elseif corrAngle < -pi
    corrAngle = corrAngle + 2*pi;
end


%Plotter punkter og linjer mellom veipunkt
plot(X_k_1,Y_k_1,'+',X_k,Y_k,'o',Xc,Yc,'*',X_k,Y_k,'x',[X_k_1 X_k], [Y_k_1 Y_k])
hold on

%Plotter radius til sirkel
th = 0:pi/50:2*pi;
xunit = 0 * cos(th) + Xc;
yunit = 0 * sin(th) + Yc;
plot(xunit, yunit)

%Plotter vektor til vessel
starts = [Xc,Yc];
[rx, ry] = pol2cart(YAWangle, vesselSpeed);
ends = starts + [rx, ry];
xlist = [starts(:,1), ends(:,1), nan(size(starts,1),1)] .';
ylist = [starts(:,2), ends(:,2), nan(size(starts,1),1)] .';
plot( xlist(:), ylist(:) );

%Posisjon beskrevet med hastighet og ytre paavirkning
Xc = Xc + vesselSpeed*cos(YAWangle) + windSpeed*cos(windAngle) + currentSpeed*cos(currentAngle);
Yc = Yc + vesselSpeed*sin(YAWangle) + windSpeed*sin(windAngle) + currentSpeed*sin(currentAngle);
YAWangle = YAWangle + corrAngle*0.3;

if YAWangle < 0
    YAWangle = YAWangle + 2*pi;
elseif YAWangle > 2*pi
    YAWangle = YAWangle - 2*pi;
end
%Velger nytt veipunkt 
if (sqrt((X_k-Xc)^2+(Y_k-Yc)^2) < Rk)
    i = i + 1;
    X_k_1 = X_k;
    Y_k_1 = Y_k;
if (i==22)
    i = 1;
elseif (i==23)
    w = 1;
end
end
end
