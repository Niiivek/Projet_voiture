clear all;
close all;

duree_simulation=85; %duree totale de simulation * secondes
frequence_des_scannes=0.5; %lidar fait un scan tous les * secondes
pas_de_tracage_figure=15000; %augmenter pour tracer moins d'instants
angle_de_braquage=0.9; %+/-(droite et gauche) angle de braquage en degre 
intersect_ext=[];
intersect_int=[];
    %circuit 1
    %x_circuit_ext = [-130 -110 40 100 100 -30 -130]; %mapshow relis les points deux à deux, il faut terminer par le premier pour bien fermer le circuit
    %y_circuit_ext = [-50 10 90 0 -110 -130 -50];

    %x_circuit_int =[-180 -160 40 150 150 -80 -180];
    %y_circuit_int = [-50 10 140 0 -160 -180 -50];

    %circuit 2
    x_circuit_int=[50 50 150 450 650 650 450 650 650 200 50]/4;
    y_circuit_int=[100 650 850 850 750 600 400 300 50 50 100]/4;

    x_circuit_ext=[200 200 200 500 500 300 300 500 200]/4;
    y_circuit_ext=[200 350 650 700 600 500 300 200 200]/4;
    
   
    mapshow(x_circuit_ext,y_circuit_ext,'Marker','*')
    mapshow(x_circuit_int,y_circuit_int,'Marker','+')

    pas_teta=pi/10;
    setenv("delta_f","0");%angle de braquage initiale

% conditions initiales du véhicule en positions et vitesses
%circuit 1
%xinit=-100; % en m
%yinit=-130; % en m

%circuit 2
xinit=25; % en m
yinit=32.5; % en m

psiinit=pi/2; % en rad

xdotinit=0; % en m/s
ydotinit=10; % en m/s
psidotinit=0; % en rad/s

X=[xinit;yinit;psiinit;xdotinit;ydotinit;psidotinit];

T0=0; % temps initial
liste_delta_f=[0];
liste_moy_dist_int=[0];
liste_moy_dist_ext=[0];
while T0<duree_simulation %simulation pour <-- secondes
    %clf() %POINT DE VUE DE LA VOITURE

%Appel du solver de l'intégration de l'équation différentielle ordinaire
%EDO (ODE en anglais)


    TFINAL=T0+frequence_des_scannes; % pas temporel des scan
    [TOUT,XOUT]=ode45('model_car',[T0 TFINAL],X);

    liste_delta_f=[liste_delta_f ,str2double(getenv("delta_f"))];


%fonction lidar 

pas_teta=pi/10;
portee_lidar=50;

[x_finint,y_finint,x_finext,y_finext]=lidar(pas_teta,portee_lidar,X,x_circuit_ext,y_circuit_ext,x_circuit_int,y_circuit_int)
    
    
    %tracage des point detecté et calculs des moyens de distances
    %interieurs et exterieurs
    if size(x_finint,1) ~= 0
        mapshow(x_finint,y_finint,'DisplayType','point','Marker','o');
        moyint=0;
    for i=1:size(x_finint,2)
        moyint=moyint+calcul_distance([x_finint(i),X(1)],[y_finint(i),X(2)]);
        moyint=moyint/size(x_finint,2);
    end
    else
        moyint=portee_lidar;
    end
    
    if size(x_finext,1) ~= 0
        mapshow(x_finext,y_finext,'DisplayType','point','Marker','o');
        moyext=0;
    for i=1:size(x_finext,2)
        moyext=moyext+calcul_distance([x_finext(i),X(1)],[y_finext(i),X(2)]);
        moyext=moyext/size(x_finext,2);
    end
    else
        moyext=portee_lidar;
    end
    
    %stockage des moyennes int et int
    liste_moy_dist_int=[liste_moy_dist_int,moyint];
    liste_moy_dist_ext=[liste_moy_dist_ext,moyext];
    
    %changement d'angle de braquage
    if (moyext/moyint>0.5 && moyext/moyint<2)
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
        %circuit 1
        %axis([-200 200 -200 200]);
 
        %circuit 2
        axis([0 250 0 250]);
        axis square;

        % Tracer la position du robot
        pose=[XOUT(i,1);XOUT(i,2);XOUT(i,3)];
        trace_car(pose,'blue',taille); 
        %plot(x(:),y(:),'r');  
        
        drawnow();
end
long=size(TOUT);
X=[XOUT(long(1),1);XOUT(long(1),2);XOUT(long(1),3);XOUT(long(1),4);XOUT(long(1),5);XOUT(long(1),6)];
T0=TFINAL; % temps initial reset
end

figure(2)

plot([0:frequence_des_scannes:TFINAL],liste_delta_f);
title("angle de braquage en fonction du temps")
xlabel("temps en seconde")
ylabel("angle de braquage en degré (delta_f)")

figure(3)
hold on
plot([0:frequence_des_scannes:TFINAL],liste_moy_dist_int);
plot([0:frequence_des_scannes:TFINAL],liste_moy_dist_ext);
xlabel("temps en seconde")
ylabel("distance par rapport à la voiture en dm")
legend("moyenne des distances avec le circuit interieur","moyenne des distances avec le circuit exterieur")
hold off
title ("moyennes")

