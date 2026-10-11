function p = paramsDrone()
% paramsDrone.m  —  Paramètres du drone aéro-sous-marin
%
% UTILISATION : p = paramsDrone();
%
% Toutes les valeurs sont des ordres de grandeur pour simulation étudiante.
% Elles sont modifiables librement pour explorer le comportement du drone.
% UNITÉS : SI (mètres, kilogrammes, secondes, newtons, radians).

%% ================================================================
%% GÉOMÉTRIE
%% ================================================================
p.L         = 1.50;              % [m]   longueur du fuselage
p.D         = 0.15;              % [m]   diamètre du fuselage
p.A_front   = pi * (p.D/2)^2;   % [m²]  section frontale (≈ 0.0177 m²)
%  → utilisée pour calculer la traînée (force ∝ A_front)

%% ================================================================
%% MASSES ET VOLUMES
%% ================================================================
p.m_dry     = 15.0;              % [kg]  masse sèche (sans eau dans le ballast)
p.V_hull    = 0.018;             % [m³]  volume externe total du fuselage
%  → donne la poussée d'Archimède : F_buo = rho_eau * g * V_hull ≈ 181 N
%  → à comparer au poids à sec : W_dry = m_dry * g ≈ 147 N  →  le drone flotte sans ballast

p.V_ballast = 0.004;             % [m³]  volume maximal du réservoir de ballast
%  → quand plein d'eau : masse_ballast_plein = rho_eau * V_ballast ≈ 4.1 kg
%  → flottabilité neutre à ballast ≈ 84 %  (calculé dans la section commentaires ci-dessous)

% --- Calcul de la flottabilité neutre (pour info) ---
%   F_buo = rho_eau * g * V_hull
%   F_poids = (m_dry + ballast * rho_eau * V_ballast) * g
%   Équilibre : ballast_neutre = (rho_eau*V_hull - m_dry) / (rho_eau*V_ballast)
%             = (1025*0.018 - 15) / (1025*0.004) = 3.45 / 4.1 ≈ 0.84
%   → Pour plonger : remplir au-delà de 84 %
%   → Pour remonter : vider en dessous de 84 %

%% ================================================================
%% SURFACES AÉRODYNAMIQUES / HYDRODYNAMIQUES
%% ================================================================
p.S_wings   = 0.80;              % [m²]  surface des grandes ailes déployées
%  Doublée pour réduire v_stall (v_stall ∝ 1/√S) → v_stall ≈ 19 m/s
p.S_canards = 0.012;             % [m²]  surface des canards (avant, orientables)
p.S_rudder  = 0.006;             % [m²]  surface de la gouverne de queue

%% ================================================================
%% COEFFICIENTS DE PORTANCE ET DE TRAÎNÉE
%% ================================================================
%  Hydrodynamique (mode sous-marin)
p.CD_water      = 0.35;          % [-]     traînée du fuselage dans l'eau (section frontale)
p.CL_canards    = 1.5;           % [-/rad] portance des canards (linéaire en angle δ)
%  → Force canard = 0.5 * rho_eau * v² * CL_canards * S_canards * delta_c

%  Aérodynamique (mode aérien, pour l'étape 2)
p.CD_air        = 0.04;          % [-]     traînée du fuselage dans l'air
p.CL_wings      = 1.2;           % [-]     portance des ailes (pour l'étape 2)
p.CD_wings_ind  = 0.08;          % [-]     traînée induite des ailes (pour l'étape 2)

%% ================================================================
%% FLUIDES
%% ================================================================
p.rho_air   = 1.225;             % [kg/m³] densité de l'air (au niveau de la mer, 15°C)
p.rho_water = 1025;              % [kg/m³] densité de l'eau de mer

%% ================================================================
%% CONSTANTES PHYSIQUES
%% ================================================================
p.g         = 9.81;              % [m/s²]  accélération gravitationnelle

%% ================================================================
%% PROPULSION (hélice sur le nez)
%% ================================================================
p.T_max     = 30.0;              % [N]  poussée maximale de l'hélice
p.T_min     =  0.0;              % [N]  poussée minimale (pas de freinage actif)
%  Ordres de grandeur : à v=2 m/s, traînée ≈ 12 N → T=12 N pour maintenir le cap

%% ================================================================
%% INERTIE (ordres de grandeur pour un objet torpille)
%% ================================================================
p.I_pitch   = 2.83;              % [kg·m²] inertie de tangage  (cylindre : m*(3r²+L²)/12 ≈ 2.83)
p.I_yaw     = 2.83;              % [kg·m²] inertie de lacet   (même valeur par symétrie)

%% ================================================================
%% BRAS DE LEVIER (distances canards/gouverne → centre de masse)
%% ================================================================
p.L_canard  = 0.55;              % [m]  bras de levier des canards
p.L_rudder  = 0.55;              % [m]  bras de levier de la gouverne

%% ================================================================
%% AMORTISSEMENT ANGULAIRE
%  (représente la résistance du fluide à la rotation)
%% ================================================================
p.damp_pitch = 4.0;              % [N·m·s/rad]  amortissement de tangage
p.damp_yaw   = 4.0;              % [N·m·s/rad]  amortissement de lacet

%% ================================================================
%% LIMITES DES COMMANDES
%% ================================================================
p.canard_max   = deg2rad(20);    % [rad]  ± angle max des canards (= ± 20°)
p.rudder_max   = deg2rad(25);    % [rad]  ± angle max de la gouverne (= ± 25°)
p.pitch_max    = deg2rad(180);   % [rad]  pas de limite (looping possible)

%% ================================================================
%% VITESSES DE VARIATION DES COMMANDES (pilotage au clavier)
%% ================================================================
p.canard_rate  = deg2rad(15);    % [rad/s]  vitesse de déflexion des canards
p.rudder_rate  = deg2rad(20);    % [rad/s]  vitesse de déflexion de la gouverne
p.thrust_rate  =  4.0;           % [N/s]    variation de poussée par seconde
p.ballast_rate =  0.15;          % [frac/s] vitesse remplissage/vidage du ballast
%  → ballast de 0 à 1 en ~7 secondes

%% ================================================================
%% TRANSITION EAU → AIR  (impulsion CO2 — modèle abstrait)
%% ================================================================
p.F_co2_max       = 600;         % [N]   force max de l'impulsion (paramètre libre)
p.t_co2           = 1.5;         % [s]   durée de l'impulsion
p.pitch_trans_min = deg2rad(30); % [rad] tangage mini pour autoriser la transition (≈30°)
%  30° au lieu de 50° → le drone sort avec une composante horizontale bien plus grande
p.v_min_air       = 16.0;        % [m/s] vitesse horizontale minimale imposée à la transition
%  Simplification : représente la propulsion pendant la sortie de l'eau.
%  → à régler selon les paramètres du drone (doit être > v_stall à alpha max)
%  → Le pilote doit orienter le nez vers la surface avant d'appuyer sur T.
%  → La force est appliquée le long de l'axe du nez (absorption du choc modélisée de façon abstraite).

%% ================================================================
%% TRANSITION AIR → EAU  (entrée dans l'eau)
%% ================================================================
p.t_entry         = 0.40;        % [s]   durée du choc d'entrée
p.CD_entry_factor = 8.0;         % [-]   multiplicateur de traînée hydrodynamique au choc
%  → La traînée est multipliée par CD_entry_factor * (1 - t/t_entry) lors de l'entrée.
%  → Modélise le choc et le freinage brutal sans calcul de pression réel.

%% ================================================================
%% PARAMÈTRES DE SIMULATION
%% ================================================================
p.dt    = 0.02;                  % [s]  pas de temps d'intégration (50 Hz)
p.t_max = 600;                   % [s]  durée maximale de la simulation

end
