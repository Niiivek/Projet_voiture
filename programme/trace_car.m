function trace_car(x,couleur,taille)
   if (exist('taille')==0), taille=1; end;
   
   % s�rie de points d�crivant la g�om�trie du robot exprim�s dans le
   % rep�re du robot
    
   M = taille*...
      [  1 -1  0  0 -1 -1 0 0 -1 1 0 0 3    3    0; 
	    -2 -2 -2 -1 -1  1 1 2  2 2 2 1 0.5 -0.5 -1]; 
    
   % en coordonn�es homog�nes
    M=[M;ones(1,length(M))]; 
      
   % Transformation homog�ne
    TH=[cos(x(3)),-sin(x(3)),x(1);...
        sin(x(3)),cos(x(3)),x(2);...
        0 0 1];
    
   % coordonn�es des points dans le rep�re sol
   M =TH*M;
   
   %trac� robot
   plot(M(1,:),M(2,:),couleur,'LineWidth',1);       
end

