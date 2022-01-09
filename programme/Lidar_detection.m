
    
    clear all;
    close all;
   
    axis([-200 200 -200 200])
    x_fin=[];
    y_fin=[];


    x=[-100];
    y=[-130];
    teta=pi/10; %20 rayons

    xt = [-130 -110 40 100 100 -30 -130]; %avec un circuit points un peu chelou mais ça passe. mapshow relis les points deux à deux, il faut terminer par le premier pour bien fermer le circuit
    yt = [-50 10 90 0 -110 -130 -50];

    xt1 =[-180 -160 40 150 150 -80 -180];
    yt1 = [-50 10 140 0 -160 -180 -50];

    xt=[xt1 xt];
    yt=[yt1 yt];

    mapshow(xt,yt,'Marker','+')
    mapshow(xt1,yt1,'Marker','*')
    %xtti=[x]; %on initialise deux matrices pour stocker les abscisses et ordonnées de nos points du capteur
    %ytti=[y];

    k=0;
    r=50;
    xr=x;
    yr=y;
    while(k<10)
        xr=x+cos(k*teta)*r*20;
        yr=y+sin(k*teta)*r*20;
        
        liste_x=[x xr];
        liste_y=[y yr];
        line(liste_x,liste_y);

        [x_fin,y_fin]=polyxpoly(liste_x,liste_y,xt,yt);
        
        if size(x_fin,1) ~= 0
        mapshow(x_fin(size(x_fin,1)),y_fin(size(y_fin,1)),'DisplayType','point','Marker','o');
        end

        k=k+1;

    end