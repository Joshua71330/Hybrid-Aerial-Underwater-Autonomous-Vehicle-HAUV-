function [ctrl, wp_reached] = waypointController(state, wp_prev, wp_curr, mode, p)
% waypointController.m  —  Régulateur en cascade pour le suivi de waypoints
%
% ── VALIDATION DU WAYPOINT : méthode du produit scalaire ─────────────────
%
%   On ne valide PAS un waypoint b par la distance (évite de tourner en rond).
%   On le valide quand le drone a dépassé le plan perpendiculaire au segment
%   (a → b) passant par b, c'est-à-dire quand :
%
%       <b − a,  m − b>  >  0
%
%   où a = wp_prev, b = wp_curr, m = position du drone.
%   Géométriquement : le drone est dans le demi-espace "après b".
%
% ── ARCHITECTURE EN CASCADE ───────────────────────────────────────────────
%
%   CAP      (yaw) :
%     Niveau 1 : yaw_error [rad]   →  r_désiré [rad/s]      (gain Kp_yaw)
%     Niveau 2 : r_error   [rad/s] →  δ_r [rad]             (gain Kr)
%
%   PROFONDEUR (z) :
%     Niveau 1 : z_error   [m]     →  pitch_désiré [rad]    (gain Kp_z)
%     Niveau 2 : pitch_err [rad]   →  δ_c [rad]             (gain Kp_pitch)
%
%   VITESSE :
%     v_error  [m/s]  →  T [N]                              (gain Kp_v)
%
% ── ENTRÉES ──────────────────────────────────────────────────────────────
%   state    [1×11]  vecteur d'état du drone
%   wp_prev  [1×3]   waypoint précédent (ou position au lancement de l'auto)
%   wp_curr  [1×3]   waypoint cible [x, y, z]
%   mode     string  'water' ou 'air'
%   p        struct  paramètres du drone (paramsDrone.m)
%
% ── SORTIES ──────────────────────────────────────────────────────────────
%   ctrl      [1×3]  [T (N),  δ_c (rad),  δ_r (rad)]
%   wp_reached bool  true quand le waypoint est validé

% ── Extraction de l'état ─────────────────────────────────────────────────
m_pos = [state(1), state(2), state(3)];
pitch = state(7);
yaw   = state(8);
r     = state(10);                              % vitesse angulaire de lacet [rad/s]
v_mag = sqrt(state(4)^2 + state(5)^2 + state(6)^2);

% ── Erreurs géométriques vers la cible ───────────────────────────────────
dx = wp_curr(1) - m_pos(1);
dy = wp_curr(2) - m_pos(2);
dz = wp_curr(3) - m_pos(3);

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║            VALIDATION PAR PRODUIT SCALAIRE                             ║
% ╚══════════════════════════════════════════════════════════════════════════╝
%   seg    = direction du segment de mission  (a → b)
%   depass = vecteur du waypoint vers le drone  (b → m)
%   Si leur produit scalaire est positif : le drone a dépassé b  ✓

seg      = wp_curr - wp_prev;
depass   = m_pos   - wp_curr;
wp_reached = (dot(seg, depass) > 0);

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║       GAINS DU RÉGULATEUR — à ajuster si le comportement est trop     ║
% ║       lent (gains trop faibles) ou oscillant (gains trop élevés)      ║
% ╚══════════════════════════════════════════════════════════════════════════╝

if strcmp(mode, 'water')
    % ── Mode sous-marin (v ≈ 2.5 m/s) ────────────────────────────────────
    v_target    =  2.5;            % [m/s]   vitesse de croisière cible
    T_trim      = 13.0;            % [N]     poussée d'équilibre à v_target
    Kp_v        =  3.0;            % [N/(m/s)]

    % Cap — calcul des gains :
    %   F_rudder = ½ρv²·CL·S·δ_r = 0.5·1025·6.25·1.5·0.006·δ_r = 28.8·δ_r
    %   M_yaw    = L_rudder·F = 0.55·28.8·δ_r = 15.8·δ_r
    %   r_ss     = M/damp = 15.8/4.0·δ_r = 3.96·δ_r  →  δ_r = r/3.96
    %   → Kr = 0.4 donne δ_r ≈ r_error/2.5 (sans saturer pour r<0.4 rad/s)
    Kp_yaw      =  0.6;            % [rad/s / rad]   yaw_error → r_désiré
    r_max       =  0.5;            % [rad/s]          cap sur la vitesse de rotation
    Kr          =  0.5;            % [rad / (rad/s)]  r_error → δ_r

    % Profondeur — calcul des gains :
    %   On veut 5 m d'erreur → 15° de tangage : Kp_z = 15°/5m ≈ 0.052 rad/m
    %   On veut 10° d'erreur de tangage → 10° canard : Kp_pitch = 1.0 rad/rad
    Kp_z        =  0.055;          % [rad/m]   z_error → pitch_désiré
    pitch_max_auto = deg2rad(20);  % [rad]     tangage max autorisé en auto
    Kp_pitch    =  2.0;            % [rad/rad] pitch_error → δ_c

else
    % ── Mode aérien (v ≈ 20 m/s) ──────────────────────────────────────────
    v_target    = 20.0;
    T_trim      = 16.0;
    Kp_v        =  1.5;

    % Cap — en air, gouverne plus faible (densité 840× plus faible que eau)
    %   r_max physique à δ_r_max = 25° : r_ss = 0.30·0.436 = 0.13 rad/s → virages larges
    Kp_yaw      =  0.5;
    r_max       =  0.12;           % [rad/s]   limité par la physique
    Kr          =  4.0;

    Kp_z        =  0.04;
    pitch_max_auto = deg2rad(15);
    Kp_pitch    =  1.5;
end

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║  1. CONTRÔLE DU CAP  (cascade 2 niveaux)                              ║
% ╚══════════════════════════════════════════════════════════════════════════╝

% Cap désiré vers la cible
yaw_desired = atan2(dy, dx);

% Erreur de cap, normalisée dans [-π, π]
yaw_error = yaw_desired - yaw;
yaw_error = mod(yaw_error + pi, 2*pi) - pi;

% Niveau 1 : cap → vitesse angulaire désirée
r_desired = Kp_yaw * yaw_error;
r_desired = max(min(r_desired, r_max), -r_max);   % saturation physique

% Niveau 2 : erreur de vitesse angulaire → gouverne
% ATTENTION au signe : delta_r > 0 → vire à DROITE (yaw diminue)
% Donc pour un r_desired positif (virage gauche), on a besoin de delta_r NÉGATIF.
r_error  = r_desired - r;
delta_r  = -Kr * r_error;   % signe négatif : corrige l'inversion physique
delta_r  = max(min(delta_r, p.rudder_max), -p.rudder_max);

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║  2. CONTRÔLE DE PROFONDEUR  (cascade 2 niveaux)                       ║
% ╚══════════════════════════════════════════════════════════════════════════╝

% Niveau 1 : erreur z → tangage désiré
pitch_desired = Kp_z * dz;
pitch_desired = max(min(pitch_desired, pitch_max_auto), -pitch_max_auto);

% Niveau 2 : erreur de tangage → canards
pitch_error = pitch_desired - pitch;
delta_c     = Kp_pitch * pitch_error;
delta_c     = max(min(delta_c, p.canard_max), -p.canard_max);

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║  3. CONTRÔLE DE VITESSE                                                ║
% ╚══════════════════════════════════════════════════════════════════════════╝

v_error = v_target - v_mag;
thrust  = T_trim + Kp_v * v_error;
thrust  = max(min(thrust, p.T_max), p.T_min);

% ── Commandes ────────────────────────────────────────────────────────────
ctrl = [thrust, delta_c, delta_r];

end
