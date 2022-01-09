function XDOT=model_car(t,X)

%t : temps
%X : ETAT

x=X(1); % abscisse du cdg
y=X(2); % ordonnée du cdg
psi=X(3); % angle d'orientation
xdot=X(4); % dx/dt
ydot=X(5); % dy/dt
psidot=X(6); % dpsi/dt

Vpsi=psidot;

%définir les unités ????

m=0.340; % masse du véhicule en Kg
Cf=1000; % rigidité de dérive latérale avant en N/rad
Cr=1000; % rigidité de dérive latérale arrière en N/rad
a=0.20; % position du centre de gravité par rapport au roues avant en m
b=0.20; % position du centre de gravité par rapport au roues arrière en m
Iz=0.01; % moment d'inertie à l'axe verticale en Kg.m^2

%La commande de l'angle de braquage 
deltaf=str2double(getenv("delta_f"))*pi/180; % angle de braquage avant en rad 

%%% Modèle dynamique : entrées deltaf et état X, sorties XDOT (dX/dt) 

%Matrice rotation

R=[cos(psi) -sin(psi); sin(psi) cos(psi)];
%Calcul de la vitesse dans la base du véhicule

V=R'*[xdot;ydot];
Vx=V(1);
Vy=V(2);

%la dynamique
Vxdot=0; % accéleration longitidinal nulle, vitesse longitudinal constante pour le moment

Vydot=-Vx*Vpsi-2*Vy*(Cf+Cr)/(m*Vx) -2*Vpsi*(a*Cf-b*Cr)/(m*Vx)+2*Cf*deltaf/m; % accélération latérale

Vpsidot=-2*Vy*(a*Cf-b*Cr)/(Iz*Vx)-2*Vpsi*(a^2*Cf+b^2*Cr)/(Iz*Vx)+2*a*Cf*deltaf/Iz; % accélération angulaire de lacet

%calcul de l'accélartion du centre de gravité dans le repère global

V0=R*[Vxdot;Vydot];
xddot=V0(1);
yddot=V0(2);

%Sorties de la fonction

XDOT=[xdot;ydot;psidot;xddot;yddot;Vpsidot];

end
        
