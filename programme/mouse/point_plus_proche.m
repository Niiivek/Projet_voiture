function out=point_plus_proche(XINIT,X,Y)
taille_tab=size(X);
if taille_tab==1
    out=[X(1),Y(1)];
    
else
    distance=[];
    for i=[1:taille_tab(2)]
       distance=[distance ,calcul_distance([XINIT(1),X(i)],[XINIT(2),Y(i)])] ;
    end
    index_min=1;
    
    for i=[2:taille_tab(2)]
        if distance(i)<distance(index_min)
            index_min=i;
        end
    end
    out=[X(index_min),Y(index_min)];
end