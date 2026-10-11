function dstate = dynamicsAir(state, ctrl, p)
% dynamicsAir.m  —  Dynamique du drone en mode aérien
%
% Même vecteur d'état que dynamicsWater.m — la structure du code reste identique.
% Seules changent : les forces (air vs eau) et le modèle de portance.
%
% ── VECTEUR D'ÉTAT (11 composantes) ───────────────────────────────────────
%   state(1)  = x        [m]      position horizontale
%   state(2)  = y        [m]      position latérale
%   state(3)  = z        [m]      altitude (z > 0 = en l'air)
%   state(4)  = vx       [m/s]    vitesse selon x
%   state(5)  = vy       [m/s]    vitesse selon y
%   state(6)  = vz       [m/s]    vitesse selon z
%   state(7)  = pitch    [rad]    tangage  (+ = nez vers le haut)
%   state(8)  = yaw      [rad]    cap
%   state(9)  = q        [rad/s]  vitesse angulaire de tangage
%   state(10) = r        [rad/s]  vitesse angulaire de lacet
%   state(11) = ballast  [-]      ignoré en mode aérien (= 0)
%
% ── VECTEUR DE COMMANDES ──────────────────────────────────────────────────
%   ctrl(1)  = T        [N]      poussée de l'hélice
%   ctrl(2)  = delta_c  [rad]    angle des canards (+ = nez monte)
%   ctrl(3)  = delta_r  [rad]    angle de la gouverne (+ = vire à droite)
%
% ── MODÈLE DE PORTANCE RETENU ─────────────────────────────────────────────
%   CL = CL_cambre + CL_alpha * alpha
%
%   où alpha = pitch - gamma  (angle d'attaque = tangage - angle de trajectoire)
%
%   CL_cambre = 0.83   : portance résiduelle du profil cambré (α=0)
%   CL_alpha  = 4.0/rad : pente de la courbe de portance (théorie ≈ 2π ≈ 6.28,
%                          réduit à 4.0 pour tenir compte des effets 3D)
%
%   → Vitesse d'équilibre (F_lift=F_poids) à α=3° : v ≈ 24 m/s
%   → En dessous de 22 m/s (α~0) : le drone commence à perdre de l'altitude
%
% ── HYPOTHÈSES SIMPLIFICATRICES ───────────────────────────────────────────
%   1. Pas de roulis (roll)
%   2. Portance dans le plan vertical (x-z) — effet y négligé
%   3. Vent et turbulences ignorés
%   4. Ballast vide (masse = m_dry)
%   5. Même inertie (I_pitch, I_yaw) qu'en mode sous-marin

% ── Extraction des variables ──────────────────────────────────────────────
vx    = state(4);
vy    = state(5);
vz    = state(6);
pitch = state(7);
yaw   = state(8);
q     = state(9);
r     = state(10);

T       = ctrl(1);
delta_c = ctrl(2);
delta_r = ctrl(3);

% ── Masse (ballast vide en mode aérien) ───────────────────────────────────
m = p.m_dry;

% ── Vecteur unitaire du nez ───────────────────────────────────────────────
nx = cos(pitch) * cos(yaw);
ny = cos(pitch) * sin(yaw);
nz = sin(pitch);

% ── Vitesse scalaire et vecteur unitaire de vitesse ───────────────────────
v_mag = sqrt(vx^2 + vy^2 + vz^2);
if v_mag > 0.5
    v_hat_x = vx / v_mag;
    v_hat_y = vy / v_mag;
    v_hat_z = vz / v_mag;
else
    v_hat_x = nx;
    v_hat_y = ny;
    v_hat_z = nz;
end

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║               MODÈLE AÉRODYNAMIQUE SIMPLIFIÉ                           ║
% ╚══════════════════════════════════════════════════════════════════════════╝

% ── Angle de trajectoire (gamma) ──────────────────────────────────────────
%   gamma = angle entre la vitesse et le plan horizontal
%   γ > 0 : le drone monte  |  γ < 0 : le drone descend
if v_mag > 0.5
    v_horiz = sqrt(vx^2 + vy^2);
    gamma = atan2(vz, max(v_horiz, 0.1));
else
    gamma = 0;
end

% ── Angle d'attaque effectif ──────────────────────────────────────────────
%   α = pitch - gamma
%   α > 0 : le nez pointe AU-DESSUS de la trajectoire → portance augmente
%   α = 0 : le nez est aligné avec la trajectoire → portance résiduelle (cambre)
%   α < 0 : le nez pointe EN-DESSOUS de la trajectoire → portance diminue
alpha = pitch - gamma;

% Limitation à un angle de décrochage réaliste (pas de modèle de spin)
alpha = max(min(alpha, deg2rad(18)), deg2rad(-8));

% ── Coefficient de portance effectif ──────────────────────────────────────
CL_cambre = 0.83;   % [-]    portance résiduelle (profil cambré, α=0)
CL_alpha  = 4.0;    % [/rad] pente de la courbe CL(α)

CL_eff = CL_cambre + CL_alpha * alpha;
CL_eff = max(min(CL_eff, p.CL_wings * 1.2), -0.2);  % plafond physique

% ── Portance (perpendiculaire à la vitesse, dans le plan vertical) ─────────
%   Direction de la portance = perpendiculaire à v, dans le sens "vers le haut"
%   Si γ est l'angle de trajectoire : direction portance = [-sin(γ), 0, cos(γ)]
F_lift_mag = 0.5 * p.rho_air * v_mag^2 * CL_eff * p.S_wings;
F_lift_mag = max(F_lift_mag, 0);  % pas de portance négative dans ce modèle simplifié

% Composantes de la portance (plan x-z)
F_lift_x = -F_lift_mag * sin(gamma);
F_lift_z =  F_lift_mag * cos(gamma);

% ── Traînée aérodynamique totale ──────────────────────────────────────────
%   Traînée = traînée de forme (fuselage) + traînée induite (ailes)
F_drag_coeff = 0.5 * p.rho_air * v_mag^2 * ...
    (p.CD_air * p.A_front + p.CD_wings_ind * p.S_wings);
Fd_x = -F_drag_coeff * v_hat_x;
Fd_y = -F_drag_coeff * v_hat_y;
Fd_z = -F_drag_coeff * v_hat_z;

% ── Poids ─────────────────────────────────────────────────────────────────
F_grav_z = -m * p.g;

% ── Poussée ───────────────────────────────────────────────────────────────
Ft_x = T * nx;
Ft_y = T * ny;
Ft_z = T * nz;

% ── Canards (correction de tangage — effet plus faible dans l'air) ────────
% Même correction que dynamicsWater : force dans le repère corps, pas vertical monde.
F_canard_mag = 0.5 * p.rho_air * v_mag^2 * p.CL_canards * p.S_canards * delta_c;

body_up_x = -sin(pitch) * cos(yaw);
body_up_y = -sin(pitch) * sin(yaw);
body_up_z =  cos(pitch);

Fc_x = F_canard_mag * body_up_x;
Fc_y = F_canard_mag * body_up_y;
Fc_z = F_canard_mag * body_up_z;

% ── Gouverne (lacet — même principe qu'en mode sous-marin) ───────────────
F_rudder_y = 0.5 * p.rho_air * v_mag^2 * p.CL_canards * p.S_rudder * delta_r;

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║                    SOMME DES FORCES ET ACCÉLÉRATIONS                   ║
% ╚══════════════════════════════════════════════════════════════════════════╝
Fx_total = Ft_x + Fd_x + F_lift_x + Fc_x;
Fy_total = Ft_y + Fd_y + Fc_y;
Fz_total = Ft_z + Fd_z + F_grav_z + F_lift_z + Fc_z;

ax = Fx_total / m;
ay = Fy_total / m;
az = Fz_total / m;

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║                      DYNAMIQUE ANGULAIRE                               ║
% ╚══════════════════════════════════════════════════════════════════════════╝

% Tangage : moment des canards + stabilité statique + amortissement
% Utilise la magnitude (pas la composante z_monde) → moment constant en looping

% ── Stabilité statique en tangage ────────────────────────────────────────
%   Sur un avion réel, le centre de pression se déplace quand α varie :
%   si α trop grand → moment nez-bas automatique (drone stable).
%   On modélise ce comportement par un rappel proportionnel à (α - α_trim).
%
%   α_trim = 3° : angle d'attaque d'équilibre à v=24 m/s
%   K_pitch_stab [N·m/rad] : "rigidité" angulaire (plus grand = plus stable,
%                             mais aussi plus raide à la commande)
%
%   Avec K=3 et damp=4 : ratio d'amortissement ζ ≈ 0.69 (légèrement sous-amorti,
%   le drone oscille un peu mais converge vers α_trim en quelques secondes).

alpha_trim   = deg2rad(3.0);   % [rad] angle d'attaque d'équilibre
K_pitch_stab = 3.0;            % [N·m/rad] rigidité de rappel en tangage
M_stab = -K_pitch_stab * (alpha - alpha_trim);

M_pitch = p.L_canard * F_canard_mag + M_stab;
dq = (M_pitch - p.damp_pitch * q) / p.I_pitch;

% Lacet : moment de la gouverne + amortissement
M_yaw = -p.L_rudder * F_rudder_y;
dr = (M_yaw - p.damp_yaw * r) / p.I_yaw;

% ╔══════════════════════════════════════════════════════════════════════════╗
% ║                   ASSEMBLAGE DU VECTEUR DÉRIVÉE                        ║
% ╚══════════════════════════════════════════════════════════════════════════╝
dstate = zeros(11, 1);

dstate(1)  = vx;
dstate(2)  = vy;
dstate(3)  = vz;
dstate(4)  = ax;
dstate(5)  = ay;
dstate(6)  = az;
dstate(7)  = q;
dstate(8)  = r;
dstate(9)  = dq;
dstate(10) = dr;
dstate(11) = 0;   % ballast inchangé (= 0 en mode aérien)

end
