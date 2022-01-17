function [x_fin,y_fin] = lidar2 (pas_teta, r, XINIT,xt,yt,xt1,yt1)
    k=0;
    xr=XINIT(1);
    yr=XINIT(2);
    x_fin=[]; y_fin=[];
    while(k<pi/pas_teta)
        xr=(XINIT(1)+cos(k*pas_teta+XINIT(3)-(pi/2))*r);
        yr=(XINIT(2)+sin(k*pas_teta+XINIT(3)-(pi/2))*r);
        
        liste_x=[XINIT(1) xr];
        liste_y=[XINIT(2) yr];
        line(liste_x,liste_y);

        [x_result1,y_result1]=polyxpoly(liste_x,liste_y,xt,yt);
        [x_result2,y_result2]=polyxpoly(liste_x,liste_y,xt1,yt1);
        x_result=[x_result1,x_result2];
        y_result=[y_result1,y_result2];
        if size(x_result,1) ~= 0
            outlidar=point_plus_proche(XINIT,x_result,y_result);
            x_fin=[x_fin,outlidar(1)];
            y_fin=[y_fin, outlidar(2)];
        end
        k=k+1;

    end
end
