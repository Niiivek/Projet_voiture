# Projet_voiture


L’objectif de ce projet est l’obtention d’un modèle de véhicule autonome de robot mobile rapide au vue de participer à la compétition de course de mini- voiture autonome organisé par l’Université Paris-Saclay en avril 2022.
En effet, le but est de construire une voiture permettant de finir un circuit prédéfini le plus rapidement possible. Le modèle et les dimensions de voiture étant imposées par les organisateurs, il s’agit d’une voiture électrique à 4 roues motrices (Tamiya XV-01 Lancia Delta). Ces voitures très légères et rapides né- cessitent le développement d’un modèle dynamique qui prend en compte l’inertie de la voiture, les forces au sol et les glissements entre les roues et le sol.
Nous supposons dans cette étude que la vitesse du robot est constante et que nous aurons seulement à gérer l’angle de braquage des roues avant afin de main- tenir le robot sur le circuit. Nous supposons également qu’il n’existe pas d’obs- tacle (pas de concurrent) sur le circuit.
Le modèle de simulation nécessite également la construction d’un modèle de l’en- vironnement qui est constitué d’un circuit fermée avec des bordures qui servent à la sécurité et au détecteur LiDAR embarqué sur le véhicule. Ce capteur fournit un nuage de points 2D dans un plan sur 360° avec une fréquence 10 Hz.
Un modèle de simulation du capteur a été développé pour rendre compte de l’environnement local du robot.
Pour finir, nous avons proposé un stratégie de commande qui permet d’ajuster l’angle de braquage en fonction de l’environnement local et la configuration du robot sur le circuit.
Ces différents modules ont été développés sous Matlab. Les résultats sont illus- trés à travers des animations graphiques et des courbes indispensables pour l’analyse et l’optimisation des différents paramètres de la course.
