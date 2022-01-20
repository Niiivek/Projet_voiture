
close all;

duree_simulation=260; %duree totale de simulation * secondes
frequence_des_scannes=0.4; %lidar fait un scan tous les * secondes
pas_de_tracage_figure=15000; %augmenter pour tracer moins d'instants
angle_de_braquage=1.5; %+/-(droite et gauche) angle de braquage en degre 

%circuit
    %int
    xt = [100 40 38 25 25 50 100 120 135 128 113 50 38 50 125 155 200 213 100]/200*400; %mapshow relis les points deux à deux, il faut terminer par le premier pour bien fermer le circuit
    yt = [28 28 38 63 190 185 195 188 196 180 188 175 125 80 90 125 180 30 28]/196*400;

    %ext
    xt1 =[100 13 25 13 13 50 100 125 160 143 113 63 50 75 113 138 213 238 100]/200*400;
    yt1 =[13 13 38 63 213 200 213 200 213 168 175 163 123 100 110 125 213 20 13]/196*400;

    mapshow(xt,yt,'Marker','+')
    mapshow(xt1,yt1,'Marker','*')

    pas_teta=pi/10;

    setenv("delta_f","0");%angle de braquage initiale

% conditions initiales du véhicule en positions et vitesses
xinit=40; % en m
yinit=150; % en m
psiinit=pi/2; % en rad

xdotinit=0; % en m/s
ydotinit=7; % en m/s
psidotinit=0; % en rad/s

XINIT=[xinit;yinit;psiinit;xdotinit;ydotinit;psidotinit];

liste_delta_f=[0];
liste_moy_dist_int=[0];
liste_moy_dist_ext=[0];

T0=0; % temps initial
figure(1)
while T0<duree_simulation %simulation pour <-- secondes
    %clf() %POINT DE VUE DE LA VOITURE

%Appel du solver de l'intégration de l'équation différentielle ordinaire
%EDO (ODE en anglais)


    TFINAL=T0+frequence_des_scannes; % pas temporel des scan
    [TOUT,XOUT]=ode45('model_car',[T0 TFINAL],XINIT);

    liste_delta_f=[liste_delta_f ,str2double(getenv("delta_f"))];
    

%rayons
    k=0;
    r=35;
    xr=XINIT(1);
    yr=XINIT(2);
    x_finint=[]; y_finint=[];x_finext=[]; y_finext=[];
    while(k<10)
        xr=(XINIT(1)+cos(k*pas_teta+XINIT(3)-psiinit)*r);
        yr=(XINIT(2)+sin(k*pas_teta+XINIT(3)-psiinit)*r);
        
        liste_x=[XINIT(1) xr];
        liste_y=[XINIT(2) yr];
        line(liste_x,liste_y);

        [x_result1,y_result1]=polyxpoly(liste_x,liste_y,xt,yt);
        [x_result2,y_result2]=polyxpoly(liste_x,liste_y,xt1,yt1);
        
        if size(x_result1,1) ~= 0
        x_finint=[x_finint , x_result1(1)];
        y_finint=[y_finint , y_result1(1)];
        end
        if size(x_result2,1) ~= 0
        x_finext=[x_finext , x_result2(1)];
        y_finext=[y_finext , y_result2(1)];
        end

        k=k+1;

    end
    
    %tracage des point detecté et calculs des moyens de distances
    %interieurs et exterieurs
    if size(x_finint,1) ~= 0
        mapshow(x_finint,y_finint,'DisplayType','point','Marker','o');
        moyint=0;
    for i=1:size(x_finint,2)
        moyint=moyint+calcul_distance([x_finint(i),XINIT(1)],[y_finint(i),XINIT(2)]);
        moyint=moyint/size(x_finint,2);
    end
    else
        moyint=r;
    end
    if size(x_finext,1) ~= 0
        mapshow(x_finext,y_finext,'DisplayType','point','Marker','o');
        moyext=0;
    for i=1:size(x_finext,2)
        moyext=moyext+calcul_distance([x_finext(i),XINIT(1)],[y_finext(i),XINIT(2)]);
        moyext=moyext/size(x_finext,2);
    end
    else
        moyext=r;
    end
    
    %stockage des moyennes int et int
    liste_moy_dist_int=[liste_moy_dist_int,moyint];
    liste_moy_dist_ext=[liste_moy_dist_ext,moyext];

    
    %changement d'angle de braquage
    if (moyext/moyint>0.4 && moyext/moyint<2.2)
       setenv("delta_f",string(0));
    else 
        if moyext>moyint
            setenv("delta_f",string(angle_de_braquage));
        else
            setenv("delta_f",string(-angle_de_braquage));
      
        end
    end

% AFFICHAGE et ANIMATION

taille = 2; 
for i=1:pas_de_tracage_figure:size(TOUT) 
        
        %clf(); % reinitialiser le dessin.
        hold on;  
        axis([0 500 0 500]);
        axis square;
        
        % Tracer la position du robot
        pose=[XOUT(i,1);XOUT(i,2);XOUT(i,3)];
        trace_car(pose,'blue',taille); 
        %plot(x(:),y(:),'r');  
        
        drawnow();
end
long=size(TOUT);
XINIT=[XOUT(long(1),1);XOUT(long(1),2);XOUT(long(1),3);XOUT(long(1),4);XOUT(long(1),5);XOUT(long(1),6)];
T0=TFINAL; % temps initial reset
end
figure(2)

plot([0:frequence_des_scannes:TFINAL],liste_delta_f);
title("angle de braquage en fonction du temp")
xlabel("temp en seconde")
ylabel("angle de braquage en degré (delta_f)")


figure(3)
hold on
plot([0:frequence_des_scannes:TFINAL],liste_moy_dist_int);
plot([0:frequence_des_scannes:TFINAL],liste_moy_dist_ext);
legend("moyenne des distances avec le circuit interieur","moyenne des distances avec le circuit exterieur")
hold off
title ("moyennes")
