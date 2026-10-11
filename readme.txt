===============================================================================
  SIMULATION MATLAB — DRONE AERO-SOUS-MARIN
  Projet etudiant : modelisation et simulation d'un drone capable d'evoluer
  sous l'eau, dans l'air, et de passer de l'un a l'autre (transition).
===============================================================================

-------------------------------------------------------------------------------
 1. PREREQUIS
-------------------------------------------------------------------------------
  - MATLAB (version recente conseillee).
  - AUCUNE toolbox payante, AUCUN Simulink : tout fonctionne avec MATLAB de base.
  - Garder TOUS les fichiers .m dans le MEME dossier.

  Pour demarrer : ouvrir MATLAB, se placer dans ce dossier (commande "cd" ou
  l'explorateur de fichiers MATLAB), puis taper le nom du main_complet_3D


-------------------------------------------------------------------------------
 2. FICHIERS DE LA SIMULATION
-------------------------------------------------------------------------------
  POINTS D'ENTREE (a lancer) :
    main_complet_3D.m   Meme physique que main_complet, mais affichage 3D
                        (modele 3D du drone + camera qui le suit).

  MOTEUR PHYSIQUE (appeles automatiquement, ne pas lancer seuls) :
    paramsDrone.m       Tous les parametres du drone (masse, geometrie,
                        surfaces, coefficients...). Unites SI.
    dynamicsWater.m     Equations de la dynamique sous-marine.
    dynamicsAir.m       Equations de la dynamique aerienne.
    waypointController.m  Regulateur automatique de suivi de waypoints
                          (cap + profondeur/altitude + vitesse, en cascade)

-------------------------------------------------------------------------------
 3. COMMENT LANCER (selon ce que vous voulez voir)
-------------------------------------------------------------------------------
  > main_complet_3D   Voir un cycle complet avec transitions et waypoints en vue 3D animee.

  IMPORTANT : cliquer d'abord SUR LA FENETRE de simulation avant d'utiliser
  le clavier, sinon les touches ne sont pas captees.


-------------------------------------------------------------------------------
 4. COMMANDES CLAVIER
-------------------------------------------------------------------------------
  Fleche HAUT / BAS    Canards -> nez monte / nez descend (tangage).
  Fleche GAUCHE/DROITE Gouverne -> vire a gauche / a droite (cap).
  W / S                Augmenter / reduire la poussee de l'helice.
  F / V                Remplir / Vider le ballast (mode sous-marin seulement).
  ESPACE               Pause / Reprise.
  ECHAP                Arreter la simulation.

  Specifique a main_complet.m et main_complet_3D.m :
  T                    Declencher la transition EAU -> AIR
                       (conditions : tangage > 30 degres, profondeur < 5 m).
  A                    Basculer pilotage MANUEL <-> AUTO (suivi de waypoints).
  J / L                Tourner la camera autour du drone (azimut).
  I / K                Monter / descendre le point de vue (elevation).
  + / -                Zoom avant / arriere.

  Couleurs de trajectoire : Cyan = sous-marin, Jaune = aerien, Vert = transition.


-------------------------------------------------------------------------------
 5. VOIR ET MODIFIER LA SIMULATION
------------------------------------------------------------------------------

  Changer le comportement du drone :
    - paramsDrone.m   : modifier masse, surfaces des ailes, coefficients, etc.
    - main_complet.m  : modifier la liste "waypoints" (section 2b) pour definir
                        votre propre mission. Format [x, y, z], avec z < 0 sous
                        l'eau et z > 0 dans l'air.
    - etat initial    : variable "state" en debut de chaque main
                        [x, y, z, vx, vy, vz, pitch, yaw, q, r, ballast].


-------------------------------------------------------------------------------
 6. REPERES UTILES
-------------------------------------------------------------------------------
  Vecteur d'etat (11 composantes) :
    x, y, z        positions [m]   (z < 0 = sous l'eau, z = 0 = surface,
                                    z > 0 = en l'air)
    vx, vy, vz     vitesses [m/s]
    pitch          tangage [rad]   (+ = nez vers le haut)
    yaw            cap [rad]
    q, r           vitesses angulaires de tangage / lacet [rad/s]
    ballast        remplissage du ballast (0 = vide, 1 = plein)

  Commandes (vecteur ctrl) :
    T              poussee de l'helice [N]
    delta_c        angle des canards [rad]
    delta_r        angle de la gouverne [rad]

  Quelques ordres de grandeur (mode sous-marin) :
    Ballast a flottabilite neutre ~ 84 %  (remplir au-dela -> plonge,
    vider en dessous -> remonte).
  Quelques ordres de grandeur (mode aerien) :
    Decrochage ~ 22 m/s, croisiere ~ 24 m/s a 3 degres de tangage.


-------------------------------------------------------------------------------
 7. HYPOTHESES ET LIMITES DU MODELE
-------------------------------------------------------------------------------
  Ce projet est UNIQUEMENT une simulation numerique, volontairement simplifiee
  (pas de CFD, pas de modele industriel). Principales hypotheses :
    - Masse ponctuelle, pas de roulis (roll) modelise.
    - Portance prise dans le plan vertical (effets lateraux negliges).
    - Trainee proportionnelle a v^2.
    - Volume de coque constant (poussee d'Archimede independante du ballast).
    - Houle, courant et vent ignores.
    - Integration numerique simple (Euler).
    - Transition eau-air / air-eau modelisee de facon ABSTRAITE : une impulsion
      verticale + un vidage rapide du ballast. Aucun systeme reel (bonbonne,
      pression, etc.) n'est modelise.

  Le but est un modele physique simple, stable et comprehensible.

===============================================================================