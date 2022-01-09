function trace_car(x,couleur,taille)
   if (exist('taille')==0), taille=1; end;
   
   % série de points décrivant la géométrie du robot exprimés dans le
   % repère du robot
    
   M = taille*...
      [  1 -1  0  0 -1 -1 0 0 -1 1 0 0 3    3    0; 
	    -2 -2 -2 -1 -1  1 1 2  2 2 2 1 0.5 -0.5 -1]; 
    
   % en coordonnées homogènes
    M=[M;ones(1,length(M))]; 
      
   % Transformation homogène
    TH=[cos(x(3)),-sin(x(3)),x(1);...
        sin(x(3)),cos(x(3)),x(2);...
        0 0 1];
    
   % coordonnées des points dans le repère sol
   M =TH*M;
   
   %tracé robot
   plot(M(1,:),M(2,:),couleur,'LineWidth',1);       
end

