figure

axis([-200 200 -200 200]);
set (gcf, 'WindowButtonMotionFcn', @mouseMove);
while true
    [~,~,button]=ginput(1);
    a=mouseMove();
    if button==29
        X(3)=0;
    else if button==28
        X(3)=pi;
    else
        X(3)=pi/2;
    end
    end

x_circuit_ext = [-130 -110 40 100 100 -30 -130]; %mapshow relis les points deux à deux, il faut terminer par le premier pour bien fermer le circuit
y_circuit_ext = [-50 10 90 0 -110 -130 -50];

x_circuit_int =[-180 -160 40 150 150 -80 -180];
y_circuit_int = [-50 10 140 0 -160 -180 -50];


mapshow(x_circuit_ext,y_circuit_ext,'Marker','*')
mapshow(x_circuit_int,y_circuit_int,'Marker','+')

pas_teta=pi/10;
portee_lidar=70;
X(1)=a(1);
X(2)=a(2);
%X(3)=pi/2;
[x_fin,y_fin]=lidar2(pas_teta,portee_lidar,X,x_circuit_ext,y_circuit_ext,x_circuit_int,y_circuit_int);
if size(x_fin,1) ~= 0
        mapshow(x_fin,y_fin,'DisplayType','point','Marker','o');
end
end