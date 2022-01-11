function [x_finint,y_finint,x_finext,y_finext] = lidar (pas_teta, r, XINIT,xt,yt,xt1,yt1)
    k=0;
    xr=XINIT(1);
    yr=XINIT(2);
    x_finint=[]; y_finint=[];x_finext=[]; y_finext=[];
    while(k<pi/pas_teta)
        xr=(XINIT(1)+cos(k*pas_teta+XINIT(3)-(pi/2))*r);
        yr=(XINIT(2)+sin(k*pas_teta+XINIT(3)-(pi/2))*r);
        
        liste_x=[XINIT(1) xr];
        liste_y=[XINIT(2) yr];
        line(liste_x,liste_y);

        [x_result1,y_result1]=polyxpoly(liste_x,liste_y,xt,yt);
        [x_result2,y_result2]=polyxpoly(liste_x,liste_y,xt1,yt1);
        
        if size(x_result1,1) ~= 0
        x_finint=[x_finint , x_result1(end)];
        y_finint=[y_finint , y_result1(end)];
        end
        if size(x_result2,1) ~= 0
        x_finext=[x_finext , x_result2(end)];
        y_finext=[y_finext , y_result2(end)];
        end

        k=k+1;

    end
end
