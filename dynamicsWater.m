function dstate = dynamicsWater(state, ctrl, p)
% dynamicsWater.m  —  Dynamique du drone en mode sous-marin
%
% Calcule les dérivées du vecteur d'état à un instant donné (intégration Euler).
%
% ── VECTEUR D'ÉTAT (11 composantes) ───────────────────────────────────────
%   state(1)  = x        [m]      position horizontale (axe avant)
%   state(2)  = y        [m]      position latérale
%   state(3)  = z        [m]      altitude  (z < 0 = sous l'eau, z = 0 = surface)
%   state(4)  = vx       [m/s]    vitesse selon x
%   state(5)  = vy       [m/s]    vitesse selon y
%   state(6)  = vz       [m/s]    vitesse selon z
%   state(7)  = pitch    [rad]    tangage  (+ = nez vers le haut)
%   state(8)  = yaw      [rad]    cap      (+ = rotation à gauche vue de dessus)
%   state(9)  = q        [rad/s]  vitesse angulaire de tangage
%   state(10) = r        [rad/s]  vitesse angulaire de lacet
%   state(11) = ballast  [-]      fraction du ballast rempli (0 = vide, 1 = plein)
%
% ── VECTEUR DE COMMANDES (3 composantes) ──────────────────────────────────
%   ctrl(1)  = T        [N]      poussée de l'hélice
%   ctrl(2)  = delta_c  [rad]    angle des canards  (+ = force vers le haut → monte)
%   ctrl(3)  = delta_r  [rad]    angle de la gouverne (+ = vire à droite)
%
% ── HYPOTHÈSES SIMPLIFICATRICES ───────────────────────────────────────────
%   1. Masse ponctuelle + orientation simplifiée (pas de couplage roll)
%   2. Portance des canards supposée purement verticale (plan x-z)
%   3. Traînée proportionnelle à v² (régime turbulent)
%   4. Volume de la coque constant (Archimède ne dépend pas du ballast)
%   5. Les effets de houle et de courant sont ignorés

% ── Extraction des variables ──────────────────────────────────────────────
vx      = state(4);
vy      = state(5);
vz      = state(6);
pitch   = state(7);
yaw     = state(8);
q       = state(9);   % vitesse angulaire de tangage [rad/s]
r       = state(10);  % vitesse angulaire de lacet   [rad/s]
ballast = state(11);

T       = ctrl(1);    % poussée [N]
delta_c = ctrl(2);    % angle canards [rad]
delta_r = ctrl(3);    % angle gouverne [rad]

% ── Masse totale avec ballast ─────────────────────────────────────────────
%   Le ballast ajoute de l'eau dans le réservoir interne.
%   Le volume externe (Archimède) reste constant.
m = p.m_dry + ballast * p.rho_water * p.V_ballast;
%   ballast=0 → m = 15 kg (flotte)
%   ballast=0.84 → m ≈ 18.4 kg (neutre)
%   ballast=1.0 → m ≈ 19.1 kg (coule lentement)

% ── Vecteur unitaire de l'axe du nez ─────────────────────────────────────
%   Le drone propulse dans la direction (pitch, yaw).
%   Repère : x = avant, y = gauche, z = haut
nx = cos(pitch) * cos(yaw);
ny = cos(pitch) * sin(yaw);
nz = sin(pitch);

% ── Module de la vitesse ──────────────────────────────────────────────────
v_mag = sqrt(vx^2 + vy^2 + vz^2);

% Vecteur unitaire de vitesse (évite la division par zéro)
if v_mag > 0.001
    v_hat_x = vx / v_mag;
    v_hat_y = vy / v_mag;
    v_hat_z = vz / v_mag;
else
    % Si quasi-immobile, la "direction" est celle du nez
    v_hat_x = nx;
    v_hat_y = ny;
    v_hat_z = nz;
end

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║                        CALCUL DES FORCES                               ║
% ╚══════════════════════════════════════════════════════════════════════════╝

% ── 1. Poussée de l'hélice (dans la direction du nez) ────────────────────
Ft_x = T * nx;
Ft_y = T * ny;
Ft_z = T * nz;

% ── 2. Poids (vers le bas) et Archimède (vers le haut) ───────────────────
%   F_buo  = rho_eau * g * V_hull  [N, vers le haut]
%   F_poids = m * g                [N, vers le bas]
F_buo   =  p.rho_water * p.g * p.V_hull;   % ≈ 181 N (constant)
F_poids = -m * p.g;                          % négatif car vers le bas
Fg_z    = F_buo + F_poids;  % résultante verticale statique
%   Fg_z > 0 → drone monte (sous-gonflé)
%   Fg_z < 0 → drone coule (sur-gonflé)
%   Fg_z = 0 → flottabilité neutre

% ── 3. Traînée hydrodynamique (s'oppose à la vitesse) ────────────────────
%   F_drag = ½ · ρ_eau · Cd · A · v² · (-v_hat)
F_drag_coeff = 0.5 * p.rho_water * p.CD_water * p.A_front * v_mag^2;
Fd_x = -F_drag_coeff * v_hat_x;
Fd_y = -F_drag_coeff * v_hat_y;
Fd_z = -F_drag_coeff * v_hat_z;

% ── 4. Portance hydrodynamique des canards ────────────────────────────────
%   Les canards créent une force PERPENDICULAIRE À L'AXE DU DRONE (dans le
%   plan de tangage), pas nécessairement verticale dans le monde.
%
%   BUG CORRIGÉ : l'ancienne version forçait la force toujours en z_monde.
%   Conséquence : à pitch=-90°, la force était nulle → plus de moment →
%   le drone plongeait en ligne droite au lieu de boucler.
%
%   CORRECTION : la force est exprimée dans la direction "haut du corps"
%   (perpendiculaire au nez, dans le plan de tangage) puis projetée en
%   coordonnées monde.
%
%   Convention :
%     delta_c > 0 → force dans le sens "haut du corps" (nez monte)
%     delta_c < 0 → force dans le sens "bas du corps"  (nez descend)

% Magnitude scalaire (signée) de la force des canards
F_canard_mag = 0.5 * p.rho_water * v_mag^2 * p.CL_canards * p.S_canards * delta_c;

% Direction "haut du corps" dans le repère monde
%   = vecteur perpendiculaire au nez dans le plan de tangage
%   Formule : rotation de +90° autour de l'axe y_monde par rapport au nez
%   Vérification :
%     pitch=0   → (0, 0, 1)   = vertical ↑           ✓
%     pitch=-90° → (1, 0, 0)  = horizontal vers avant ✓ (continue le looping)
%     pitch=+90° → (-1, 0, 0) = horizontal vers arrière ✓
body_up_x = -sin(pitch) * cos(yaw);
body_up_y = -sin(pitch) * sin(yaw);
body_up_z =  cos(pitch);

Fc_x = F_canard_mag * body_up_x;
Fc_y = F_canard_mag * body_up_y;
Fc_z = F_canard_mag * body_up_z;

% ── 5. Force latérale de la gouverne (pour le lacet) ─────────────────────
%   La gouverne arrière crée une force latérale.
%   Convention :
%     delta_r > 0 → vire à DROITE (yaw diminue)
%     delta_r < 0 → vire à GAUCHE (yaw augmente)
F_rudder_y = 0.5 * p.rho_water * v_mag^2 * p.CL_canards * p.S_rudder * delta_r;

% ── Somme des forces linéaires (Newton : F = m·a) ─────────────────────────
Fx_total = Ft_x + Fd_x + Fc_x;
Fy_total = Ft_y + Fd_y + Fc_y;
Fz_total = Ft_z + Fd_z + Fg_z + Fc_z;

% ── Accélérations linéaires ───────────────────────────────────────────────
ax = Fx_total / m;
ay = Fy_total / m;
az = Fz_total / m;

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║                    DYNAMIQUE ANGULAIRE SIMPLIFIÉE                      ║
% ╚══════════════════════════════════════════════════════════════════════════╝
%
%   On modélise les moments créés par les surfaces de contrôle.
%   L'équation est : I · dω/dt = M_commande - k_damp · ω
%   (analogue à un ressort-amortisseur angulaire)

% ── Tangage (pitch) ───────────────────────────────────────────────────────
%   Les canards à l'avant créent un moment de tangage.
%   Le moment utilise la MAGNITUDE de la force (pas sa composante z_monde).
%   Ainsi le moment est constant tout au long d'un looping, peu importe l'orientation.
M_pitch = p.L_canard * F_canard_mag;
dq = (M_pitch - p.damp_pitch * q) / p.I_pitch;

% ── Lacet (yaw) ───────────────────────────────────────────────────────────
%   La gouverne à l'arrière crée un moment de lacet.
%   Gouverne à l'arrière → force latérale sur la queue → nez tourne dans le sens OPPOSÉ
%   Convention : delta_r > 0 → queue va à gauche → nez tourne à droite → yaw diminue → M_yaw < 0
M_yaw = -p.L_rudder * F_rudder_y;
dr = (M_yaw - p.damp_yaw * r) / p.I_yaw;

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║                    ASSEMBLAGE DU VECTEUR DÉRIVÉE                       ║
% ╚══════════════════════════════════════════════════════════════════════════╝
dstate = zeros(11, 1);

dstate(1)  = vx;      % dx/dt  = vitesse selon x
dstate(2)  = vy;      % dy/dt  = vitesse selon y
dstate(3)  = vz;      % dz/dt  = vitesse selon z
dstate(4)  = ax;      % dvx/dt = accélération selon x
dstate(5)  = ay;      % dvy/dt = accélération selon y
dstate(6)  = az;      % dvz/dt = accélération selon z
dstate(7)  = q;       % dpitch/dt = vitesse angulaire de tangage
dstate(8)  = r;       % dyaw/dt   = vitesse angulaire de lacet
dstate(9)  = dq;      % dq/dt     = accélération angulaire de tangage
dstate(10) = dr;      % dr/dt     = accélération angulaire de lacet
dstate(11) = 0;       % Le ballast est mis à jour directement par le clavier (main.m)

end
