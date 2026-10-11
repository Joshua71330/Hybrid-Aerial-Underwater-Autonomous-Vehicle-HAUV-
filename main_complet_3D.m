% main_complet_3D.m  —  Simulation unifiée du drone aéro-sous-marin (vue 3D)
% Modes : SOUS-MARIN / TRANSITION Eau→Air / AÉRIEN / TRANSITION Air→Eau
%
% Version 3D de main_complet.m : la physique est IDENTIQUE (l'état contenait
% déjà x, y, z, pitch, yaw — donc déjà du 3D). Seul l'AFFICHAGE change :
% une seule vue 3D avec une "caméra" qui suit le drone, et un VRAI MODÈLE 3D
% du drone : fuselage torpille, ailes déployables (sorties en mode aérien,
% repliées sous l'eau), canards mobiles, gouverne mobile, hélice qui tourne.
%
% PRINCIPE DU MODÈLE 3D (hgtransform) :
%   - Chaque pièce est construite UNE SEULE FOIS dans le "repère corps"
%     (x = vers l'avant, y = vers la gauche, z = vers le haut).
%   - Toutes les pièces sont attachées à un objet hgtransform.
%   - À chaque image, on ne change QUE la matrice de transformation
%     (position + cap + tangage) : c'est simple et très rapide.
%
% LANCEMENT : taper  main_complet_3D  dans la fenêtre MATLAB
%
% COMMANDES CLAVIER (identiques à main_complet.m) :
%   ↑ ↓        Canards (tangage : monter / plonger)
%   ← →        Gouverne (cap : gauche / droite)
%   W / S      Poussée  (plus / moins)
%   F / V      Ballast  (remplir / vider)  — mode sous-marin uniquement
%   T          Déclencher la TRANSITION Eau→Air
%              (conditions : pitch > 30°, profondeur < 5 m)
%   A          Basculer entre mode MANUEL et mode AUTO (suivi de waypoints)
%   J / L      Caméra : tourner autour du drone (azimut)
%   I / K      Caméra : monter / descendre le point de vue (élévation)
%   + / -      Caméra : zoom avant / arrière (pavé numérique accepté)
%   ESPACE     Pause / Reprise
%   ECHAP      Arrêter
%
% COULEURS DE TRAJECTOIRE :
%   Cyan  → sous-marin   |   Jaune → aérien   |   Vert → transition
%
% NOTE SUR LA VUE 3D :
%   - L'axe Z = altitude (z<0 sous l'eau, z>0 dans l'air, z=0 = surface).
%   - Un plan bleu transparent représente la surface de l'eau.
%   - La "caméra" (xlim/ylim/zlim) recentre une boîte de 50x50x50 m autour
%     du drone à chaque image, avec un angle de vue fixe (vue de 3/4 arrière).
%   - daspect([1 1 1]) garde les proportions réelles (pas de déformation).

clear; close all; clc;
global gKeys gPause;

%% ================================================================
%% 1. PARAMÈTRES
%% ================================================================
p = paramsDrone();

CL_cambre    = 0.83;
alpha_trim   = deg2rad(3.0);
K_pitch_stab = 3.0;
v_stall      = sqrt(2*p.m_dry*p.g / (p.rho_air * CL_cambre * p.S_wings));

%% ================================================================
%% 2. ÉTAT INITIAL
%% ================================================================
% [x, y, z, vx, vy, vz, pitch, yaw, q, r, ballast]
state = [0, 0, -10.0,  1.0, 0, 0,  0, 0, 0, 0,  0.84];

%% ================================================================
%% 2b. WAYPOINTS
%% ================================================================
% Modifiez cette liste pour définir votre mission.
% Format : [x, y, z]    z < 0 = sous l'eau
waypoints = [
      0,    0,  -10;   % WP1
     60,    0,  -10;   % WP2 - ligne droite
     80,   30,   -5;   % WP3 - virer gauche, remonter
    140,   30,  -10;   % WP4 - continuer
    140,    0,  -15;   % WP5 - virer droite, plonger
    200,    0,  -10;   % WP6 - terminus
];
n_wp      = size(waypoints, 1);
wp_idx    = 1;
wp_prev_pos = state(1:3);   % position "avant le premier WP" = position initiale
auto_mode   = false;

%% ================================================================
%% 3. COMMANDES ET MODE
%% ================================================================
ctrl  = [8.0, 0.0, 0.0];
mode  = 'water';
trans_timer = 0;
wings_out   = false;

%% ================================================================
%% 4. CLAVIER
%% ================================================================
gKeys.up     = false;  gKeys.down  = false;
gKeys.left   = false;  gKeys.right = false;
gKeys.w      = false;  gKeys.s     = false;
gKeys.f      = false;  gKeys.v     = false;
gKeys.t      = false;
gKeys.a      = false;   % toggle auto/manuel
gKeys.j      = false;  gKeys.l     = false;   % caméra : azimut
gKeys.i      = false;  gKeys.k     = false;   % caméra : élévation
gKeys.plus   = false;  gKeys.minus = false;   % caméra : zoom
gKeys.escape = false;
gPause       = false;

%% ================================================================
%% 5. FIGURE — vue 3D + tableau de bord
%% ================================================================
fig = figure('Name',          'Drone Aéro-Sous-Marin — Simulation 3D', ...
             'NumberTitle',   'off', ...
             'Position',      [60, 60, 1280, 840], ...
             'Color',         [0.08 0.10 0.14], ...
             'KeyPressFcn',   @onKeyPress, ...
             'KeyReleaseFcn', @onKeyRelease, ...
             'CloseRequestFcn', @onClose);

% ── Vue 3D (occupe les 3/4 gauches de la figure) ───────────────────────────
ax3d = subplot(1, 4, [1 2 3]);
hold(ax3d, 'on');
grid(ax3d, 'on');
box(ax3d, 'on');
ax3d.Color     = [0.05 0.10 0.18];
ax3d.XColor    = [0.8 0.85 0.9];
ax3d.YColor    = [0.8 0.85 0.9];
ax3d.ZColor    = [0.8 0.85 0.9];
ax3d.GridColor = [0.2 0.25 0.35];
ax3d.GridAlpha = 0.6;
xlabel(ax3d, 'x [m]', 'Color', [0.8 0.85 0.9], 'FontSize', 11);
ylabel(ax3d, 'y [m]', 'Color', [0.8 0.85 0.9], 'FontSize', 11);
zlabel(ax3d, 'z [m]  (altitude, 0 = surface)', 'Color', [0.8 0.85 0.9], 'FontSize', 11);
title(ax3d, 'Trajectoire 3D', 'Color', [0.9 0.95 1.0], 'FontSize', 12);
cam_az = -50;                  % azimut caméra [°]   — modifiable avec J/L
cam_el = 18;                   % élévation caméra [°] — modifiable avec I/K
view(ax3d, cam_az, cam_el);    % vue initiale : 3/4 arrière-haut
daspect(ax3d, [1 1 1]);        % proportions réelles (pas de déformation)

% Désactiver les interactions par défaut des axes (zoom/rotation) : elles
% peuvent intercepter les touches du clavier et ralentir l'affichage.
% (try/catch : ces fonctions n'existent que depuis MATLAB R2019b)
try
    disableDefaultInteractivity(ax3d);
    ax3d.Toolbar.Visible = 'off';
catch
end

% Plan de la surface de l'eau (z = 0), semi-transparent
patch(ax3d, 'XData', [-20 260 260 -20], 'YData', [-30 -30 70 70], ...
      'ZData', [0 0 0 0], 'FaceColor', [0.25 0.45 0.75], ...
      'FaceAlpha', 0.15, 'EdgeColor', [0.0 0.80 1.0], 'LineWidth', 0.5);

% Waypoints (ligne + marqueurs + labels)
plot3(ax3d, waypoints(:,1), waypoints(:,2), waypoints(:,3), '--', ...
      'Color', [0.35 0.50 0.35], 'LineWidth', 1.0);
plot3(ax3d, waypoints(:,1), waypoints(:,2), waypoints(:,3), 'o', 'MarkerSize', 7, ...
      'MarkerFaceColor', [0.25 0.38 0.25], 'MarkerEdgeColor', [0.5 0.75 0.5], 'LineStyle', 'none');
for k = 1:n_wp
    text(ax3d, waypoints(k,1)+1.5, waypoints(k,2)+1.5, waypoints(k,3)+1.0, sprintf('WP%d',k), ...
         'Color', [0.55 0.80 0.55], 'FontSize', 7, 'FontName', 'Courier New');
end
h_wp_3d = plot3(ax3d, waypoints(1,1), waypoints(1,2), waypoints(1,3), 'o', 'MarkerSize', 13, ...
                'MarkerFaceColor', [1.0 0.85 0.0], 'MarkerEdgeColor', 'w', 'LineWidth', 1.5);

% Trajectoires (3 couleurs)
h_tw = plot3(ax3d, state(1), state(2), state(3), '-', 'Color', [0.2 0.8 1.0], 'LineWidth', 1.8);
h_ta = plot3(ax3d, NaN, NaN, NaN,      '-', 'Color', [1.0 0.8 0.2], 'LineWidth', 1.8);
h_tt = plot3(ax3d, NaN, NaN, NaN,      '-', 'Color', [0.3 1.0 0.4], 'LineWidth', 2.5);

% ── MODÈLE 3D DU DRONE ────────────────────────────────────────────────────
% Repère corps : x = avant, y = gauche, z = haut. Unités : mètres.
VIS_SCALE = 2.0;     % agrandissement visuel : le drone fait 2.5 m, il serait
                     % minuscule dans une boîte de 50 m → on l'affiche ×2
L_f = 2.5;           % longueur du fuselage [m]
R_f = 0.18;          % rayon du fuselage [m]

T_body = hgtransform('Parent', ax3d);   % transformation "maître" du drone

% -- Fuselage : surface de révolution en forme de torpille --
% rayon plein au centre, effilé au nez et à la queue (profil x^4)
xs    = linspace(-L_f/2, L_f/2, 24);                 % stations le long de x
rprof = R_f * sqrt(max(0, 1 - (2*xs/L_f).^4));       % rayon à chaque station
th    = linspace(0, 2*pi, 24);                       % tour complet
Xf = repmat(xs', 1, numel(th));
Yf = rprof' * cos(th);
Zf = rprof' * sin(th);
surface(Xf, Yf, Zf, 'Parent', T_body, 'FaceColor', [1.0 0.6 0.1], ...
        'EdgeColor', 'none', 'FaceLighting', 'gouraud');

% -- Ailes déployables (1 hgtransform par aile pour pouvoir les replier) --
% Repliées : pivotées de 80° vers l'arrière, plaquées le long du fuselage.
wing_chord  = 0.55;                 % corde [m]
wing_span   = 2.2;                  % envergure d'UNE aile [m]
wing_attach = [-0.10, 0, 0.06];     % point d'attache sur le fuselage
wxd = [ wing_chord/2,  wing_chord/2, -wing_chord/2, -wing_chord/2];
T_wingL = hgtransform('Parent', T_body);
T_wingR = hgtransform('Parent', T_body);
patch('Parent', T_wingL, 'XData', wxd, 'YData', [0  wing_span  wing_span 0], ...
      'ZData', [0 0 0 0], 'FaceColor', [0.85 0.85 0.92], ...
      'EdgeColor', [0.45 0.45 0.55], 'FaceLighting', 'gouraud');
patch('Parent', T_wingR, 'XData', wxd, 'YData', [0 -wing_span -wing_span 0], ...
      'ZData', [0 0 0 0], 'FaceColor', [0.85 0.85 0.92], ...
      'EdgeColor', [0.45 0.45 0.55], 'FaceLighting', 'gouraud');

% -- Canards avant (mobiles : suivent la commande de tangage ctrl(2)) --
can_chord  = 0.28;  can_span = 0.55;
can_attach = [0.80, 0, 0];
T_can = hgtransform('Parent', T_body);
patch('Parent', T_can, ...
      'XData', [ can_chord/2,  can_chord/2, -can_chord/2, -can_chord/2], ...
      'YData', [-can_span,     can_span,     can_span,    -can_span], ...
      'ZData', [0 0 0 0], 'FaceColor', [0.95 0.75 0.30], ...
      'EdgeColor', [0.5 0.4 0.2], 'FaceLighting', 'gouraud');

% -- Dérive fixe + gouverne mobile (suit la commande de cap ctrl(3)) --
patch('Parent', T_body, ...                                   % dérive fixe
      'XData', [-0.70 -0.70 -0.95 -0.95], 'YData', [0 0 0 0], ...
      'ZData', [ 0.10  0.45  0.45  0.10], 'FaceColor', [1.0 0.6 0.1], ...
      'EdgeColor', [0.6 0.4 0.1], 'FaceLighting', 'gouraud');
rud_attach = [-0.95, 0, 0];
T_rud = hgtransform('Parent', T_body);
patch('Parent', T_rud, ...                                    % gouverne mobile
      'XData', [0 0 -0.30 -0.30], 'YData', [0 0 0 0], ...
      'ZData', [0.10 0.45 0.45 0.10], 'FaceColor', [0.95 0.75 0.30], ...
      'EdgeColor', [0.5 0.4 0.2], 'FaceLighting', 'gouraud');

% -- Hélice sur le nez (croix de 4 pales, tourne selon la poussée) --
prop_attach = [L_f/2 + 0.06, 0, 0];
T_prop = hgtransform('Parent', T_body);
patch('Parent', T_prop, 'XData', [0 0 0 0], ...               % pales verticales
      'YData', [-0.05 0.05 0.05 -0.05], 'ZData', [-0.5 -0.5 0.5 0.5], ...
      'FaceColor', [0.75 0.85 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.85);
patch('Parent', T_prop, 'XData', [0 0 0 0], ...               % pales horizontales
      'YData', [-0.5 -0.5 0.5 0.5], 'ZData', [-0.05 0.05 0.05 -0.05], ...
      'FaceColor', [0.75 0.85 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.85);

% Position/orientation initiale du drone
set(T_body, 'Matrix', makehgtform('translate', state(1:3), ...
    'zrotate', state(8), 'yrotate', -state(7), 'scale', VIS_SCALE));

% Éclairage de la scène (indispensable pour voir le relief des surfaces)
light(ax3d, 'Style', 'infinite', 'Position', [-2 -3 5]);

% Aide-mémoire commandes (coin bas-gauche, en coordonnées normalisées)
text(ax3d, 0.01, 0.02, 0, sprintf(['↑↓ Canards  W/S Poussée  ←→ Cap\n' ...
    'F/V Ballast   T Trans.eau→air   A Auto/Manuel\n' ...
    'CAMERA : J/L tourner  I/K élévation  +/- zoom\n' ...
    'STALL < %.0f m/s   ESPACE Pause   ESC Stop'], v_stall), ...
    'Units', 'normalized', 'Color', [0.5 0.6 0.7], 'FontSize', 8, ...
    'FontName', 'Courier New', 'VerticalAlignment', 'bottom', 'BackgroundColor', [0.05 0.10 0.18]);

% Fenêtre de "caméra suiveuse" : cube de 50x50x50 m centré sur le drone
CAM_HALF = 25;
xlim(ax3d, [state(1)-CAM_HALF, state(1)+CAM_HALF]);
ylim(ax3d, [state(2)-CAM_HALF, state(2)+CAM_HALF]);
zlim(ax3d, [-30, 20]);

% ── Tableau de bord (colonne droite) ──────────────────────────────────────
ax_dash = subplot(1, 4, 4);
ax_dash.Color = [0.06 0.08 0.12];  ax_dash.Visible = 'off';
axis(ax_dash, [0 1 0 1]);
h_dash = text(ax_dash, 0.06, 0.97, '', 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'FontName', 'Courier New', 'FontSize', 10.5, ...
    'Color', [0.88 0.92 0.98], 'BackgroundColor', [0.06 0.08 0.12], 'Interpreter', 'none');

%% ================================================================
%% 6. TRAJECTOIRES ET COMPTEURS
%% ================================================================
TRAJ_MAX    = 1500;
fold_curr   = 80;    % angle de repli des ailes [°] : 80 = repliées, 0 = déployées
prop_angle  = 0;     % angle de rotation de l'hélice [rad]
traj_xw = state(1);  traj_yw = state(2);  traj_zw = state(3);
traj_xa = [];  traj_ya = [];  traj_za = [];
traj_xt = [];  traj_yt = [];  traj_zt = [];

%% ================================================================
%% 7. BOUCLE PRINCIPALE
%% ================================================================
t = 0;
fprintf('\n  Simulation 3D démarrée — mode SOUS-MARIN\n');
fprintf('  A = activer le mode automatique\n\n');
figure(fig);

while ~gKeys.escape && ishandle(fig) && t < p.t_max
    tic_loop = tic;

    if ~gPause
        dt = p.dt;

        %% ── 7a. Toggle auto/manuel (touche A) ────────────────────────────
        if gKeys.a
            auto_mode = ~auto_mode;
            gKeys.a   = false;

            if auto_mode
                pos_now = state(1:3);

                % Trouver le prochain WP "devant" le drone :
                % 1. Calculer la distance à chaque WP
                % 2. Parmi ceux qui sont devant (dot > 0), prendre le plus proche
                % 3. Sinon prendre le plus proche tout court
                heading2d  = [cos(state(8)), sin(state(8)), 0];
                best_idx   = n_wp;
                best_dist  = Inf;
                for k = 1:n_wp
                    to_wp = waypoints(k,:) - pos_now;
                    d     = norm(to_wp);
                    % Préférer un WP devant le drone
                    if dot(to_wp, heading2d) > 0 && d < best_dist
                        best_dist = d;
                        best_idx  = k;
                    end
                end
                % Fallback : aucun devant → le plus proche
                if isinf(best_dist)
                    dists    = sqrt(sum((waypoints - pos_now).^2, 2));
                    [~, best_idx] = min(dists);
                end

                wp_idx      = best_idx;
                wp_prev_pos = pos_now;    % position actuelle = point de référence du segment
                fprintf('  [t=%.1f s] Mode AUTO → WP%d/%d\n', t, wp_idx, n_wp);
            else
                fprintf('  [t=%.1f s] Mode MANUEL\n', t);
            end
        end

        %% ── 7b. Commandes ────────────────────────────────────────────────
        if auto_mode && (strcmp(mode,'water') || strcmp(mode,'air'))
            %% Mode automatique — appel du régulateur
            wp_curr_target = waypoints(wp_idx,:);
            [ctrl, wp_reached] = waypointController(state, wp_prev_pos, wp_curr_target, mode, p);

            if wp_reached && wp_idx < n_wp
                fprintf('  [t=%.1f s] WP%d validé → WP%d\n', t, wp_idx, wp_idx+1);
                wp_prev_pos = wp_curr_target;   % le WP validé devient le nouveau "précédent"
                wp_idx      = wp_idx + 1;
            end

            % Ballast auto : viser flottabilité neutre (84 %)
            if strcmp(mode,'water')
                state(11) = state(11) + sign(0.84 - state(11)) * p.ballast_rate * dt * 0.2;
                state(11) = max(min(state(11), 1.0), 0.0);
            end

        else
            %% Mode manuel — lecture du clavier
            if gKeys.up,    ctrl(2) = min(ctrl(2) + p.canard_rate*dt,  p.canard_max); end
            if gKeys.down,  ctrl(2) = max(ctrl(2) - p.canard_rate*dt, -p.canard_max); end
            if ~gKeys.up && ~gKeys.down,  ctrl(2) = ctrl(2) * 0.92; end

            if gKeys.right, ctrl(3) = min(ctrl(3) + p.rudder_rate*dt,  p.rudder_max); end
            if gKeys.left,  ctrl(3) = max(ctrl(3) - p.rudder_rate*dt, -p.rudder_max); end
            if ~gKeys.right && ~gKeys.left,  ctrl(3) = ctrl(3) * 0.92; end

            if gKeys.w,  ctrl(1) = min(ctrl(1) + p.thrust_rate*dt, p.T_max); end
            if gKeys.s,  ctrl(1) = max(ctrl(1) - p.thrust_rate*dt, p.T_min); end

            if strcmp(mode,'water')
                if gKeys.f, state(11) = min(state(11) + p.ballast_rate*dt, 1.0); end
                if gKeys.v, state(11) = max(state(11) - p.ballast_rate*dt, 0.0); end
            end
        end

        %% ── 7c. Transition Eau→Air (touche T, toujours en manuel) ────────
        if gKeys.t && strcmp(mode,'water') && state(7) >= p.pitch_trans_min && state(3) >= -5.0
            mode = 'trans_w2a';
            trans_timer = 0;
            state(11)   = 0;
            ctrl(1)     = p.T_max * 0.5;
            fprintf('  [t=%.1f s] TRANSITION Eau→Air déclenchée!\n', t);
        end

        %% ── 7d. Physique selon le mode ───────────────────────────────────
        nx = cos(state(7))*cos(state(8));
        ny = cos(state(7))*sin(state(8));
        nz = sin(state(7));
        m_curr = p.m_dry + state(11)*p.rho_water*p.V_ballast;

        switch mode
            case 'water'
                dstate = dynamicsWater(state, ctrl, p);

            case 'trans_w2a'
                dstate = dynamicsWater(state, ctrl, p);
                F_co2 = p.F_co2_max * max(0, 1 - trans_timer/p.t_co2);
                dstate(4) = dstate(4) + F_co2*nx/m_curr;
                dstate(5) = dstate(5) + F_co2*ny/m_curr;
                dstate(6) = dstate(6) + F_co2*nz/m_curr;
                trans_timer = trans_timer + dt;
                if state(3) > 0.5
                    mode = 'air';  wings_out = true;  ctrl(1) = p.T_max;
                    v_h = sqrt(state(4)^2+state(5)^2);
                    if v_h < p.v_min_air
                        fac = p.v_min_air / max(v_h, 0.01);
                        state(4) = state(4)*fac;  state(5) = state(5)*fac;
                    end
                    state(7) = min(state(7), deg2rad(15));
                    state(6) = min(state(6), 5.0);
                    fprintf('  [t=%.1f s] Mode AÉRIEN — v=%.1f m/s\n', t, ...
                            sqrt(state(4)^2+state(5)^2+state(6)^2));
                end

            case 'air'
                % La stabilité de tangage est déjà incluse dans dynamicsAir.m
                % Ne pas l'ajouter ici une seconde fois (doublon → instabilité)
                dstate = dynamicsAir(state, ctrl, p);
                if state(3) <= 0
                    mode = 'trans_a2w';  trans_timer = 0;
                    wings_out = false;  state(3) = 0;  state(11) = 0.6;
                    fprintf('  [t=%.1f s] TRANSITION Air→Eau — v=%.1f m/s\n', t, ...
                            sqrt(state(4)^2+state(5)^2+state(6)^2));
                end

            case 'trans_a2w'
                p_e = p;
                p_e.CD_water = p.CD_water * (1 + p.CD_entry_factor*max(0,1-trans_timer/p.t_entry));
                dstate = dynamicsWater(state, ctrl, p_e);
                trans_timer = trans_timer + dt;
                if trans_timer >= p.t_entry
                    mode = 'water';
                    fprintf('  [t=%.1f s] Mode SOUS-MARIN activé.\n', t);
                end
        end

        %% ── 7e. Intégration et contraintes ──────────────────────────────
        state = state + dstate' * dt;
        % Normalisation des angles pour éviter l'accumulation numérique infinie
        % (sin/cos perdent leur précision sur des très grands nombres)
        state(7)  = mod(state(7) + pi, 2*pi) - pi;   % pitch dans [-π, π]
        state(8)  = mod(state(8),       2*pi);         % yaw   dans [0,  2π]
        state(11) = max(min(state(11), 1.0), 0.0);
        if strcmp(mode,'water') && state(3) > 0,  state(3) = 0;  state(6) = min(state(6),0);  end
        if strcmp(mode,'air')   && state(3) < 0
            state(3) = 0;
            state(4) = state(4)*0.3;  state(5) = state(5)*0.3;  state(6) = max(state(6),0);
        end
        t = t + dt;

        %% ── 7f. Accumulation trajectoires ────────────────────────────────
        switch mode
            case {'water','trans_a2w'}
                traj_xw(end+1) = state(1);  traj_yw(end+1) = state(2);  traj_zw(end+1) = state(3);
            case 'air'
                traj_xa(end+1) = state(1);  traj_ya(end+1) = state(2);  traj_za(end+1) = state(3);
            otherwise
                traj_xt(end+1) = state(1);  traj_yt(end+1) = state(2);  traj_zt(end+1) = state(3);
        end
    end  % fin ~gPause

    %% ── 7g. Visualisation 3D ─────────────────────────────────────────────
    % Si la fenêtre a été fermée pendant ce tour de boucle, sortir proprement
    % (sinon : erreur "update" sur des objets graphiques supprimés)
    if ~ishandle(fig), break; end

    % Trajectoires 3D
    if ~isempty(traj_xw), i0=max(1,length(traj_xw)-TRAJ_MAX+1); set(h_tw,'XData',traj_xw(i0:end),'YData',traj_yw(i0:end),'ZData',traj_zw(i0:end)); end
    if ~isempty(traj_xa), i0=max(1,length(traj_xa)-TRAJ_MAX+1); set(h_ta,'XData',traj_xa(i0:end),'YData',traj_ya(i0:end),'ZData',traj_za(i0:end)); end
    if ~isempty(traj_xt), i0=max(1,length(traj_xt)-TRAJ_MAX+1); set(h_tt,'XData',traj_xt(i0:end),'YData',traj_yt(i0:end),'ZData',traj_zt(i0:end)); end

    % ── Modèle 3D du drone : une seule matrice à mettre à jour ──────────
    % position (x,y,z) + cap (rotation z) + tangage (rotation y, signe -)
    set(T_body, 'Matrix', makehgtform('translate', state(1:3), ...
        'zrotate', state(8), 'yrotate', -state(7), 'scale', VIS_SCALE));

    % Ailes : déployées en mode aérien, repliées sinon (animation lissée)
    if wings_out, fold_target = 0; else, fold_target = 80; end
    fold_curr = fold_curr + (fold_target - fold_curr) * 0.08;   % lissage
    phi = deg2rad(fold_curr);
    set(T_wingL, 'Matrix', makehgtform('translate', wing_attach, 'zrotate',  phi));
    set(T_wingR, 'Matrix', makehgtform('translate', wing_attach, 'zrotate', -phi));

    % Canards : suivent la commande de tangage (braquage visible)
    set(T_can, 'Matrix', makehgtform('translate', can_attach, 'yrotate', -ctrl(2)));

    % Gouverne : suit la commande de cap
    set(T_rud, 'Matrix', makehgtform('translate', rud_attach, 'zrotate', ctrl(3)));

    % Hélice : vitesse de rotation proportionnelle à la poussée
    prop_angle = prop_angle + 0.15 + 0.02*ctrl(1);
    set(T_prop, 'Matrix', makehgtform('translate', prop_attach, 'xrotate', prop_angle));

    % Waypoint actif (sphère jaune)
    set(h_wp_3d, 'XData', waypoints(wp_idx,1), 'YData', waypoints(wp_idx,2), 'ZData', waypoints(wp_idx,3));

    % ── Caméra orientable au clavier ─────────────────────────────────────
    % J/L : tourner autour du drone   I/K : élévation   +/- : zoom
    % On n'appelle view() QUE si une touche est pressée (sinon re-rendu inutile)
    if gKeys.j || gKeys.l || gKeys.i || gKeys.k
        if gKeys.j, cam_az = cam_az + 1.2; end
        if gKeys.l, cam_az = cam_az - 1.2; end
        if gKeys.i, cam_el = min(cam_el + 0.8,  89); end   % borné : à 90° pile,
        if gKeys.k, cam_el = max(cam_el - 0.8, -89); end   % la vue 3D dégénère
        view(ax3d, cam_az, cam_el);
    end
    if gKeys.plus || gKeys.minus
        if gKeys.plus,  CAM_HALF = max(CAM_HALF*0.97,  8);   end
        if gKeys.minus, CAM_HALF = min(CAM_HALF*1.03, 120);  end
        xlim(ax3d, [state(1)-CAM_HALF, state(1)+CAM_HALF]);   % recentrer la boîte
        ylim(ax3d, [state(2)-CAM_HALF, state(2)+CAM_HALF]);   % avec la nouvelle
        zlim(ax3d, [state(3)-CAM_HALF, state(3)+CAM_HALF]);   % taille de zoom
    end

    % Caméra suiveuse avec "zone morte" : on ne recentre la boîte que si le
    % drone s'approche à moins de 10 m d'un bord (8 m en z). Recentrer à
    % CHAQUE image forçait MATLAB à redessiner toute la grille 3D → gros
    % ralentissement, et le clavier semblait répondre en retard.
    xl = xlim(ax3d);  yl = ylim(ax3d);  zl = zlim(ax3d);
    if state(1) < xl(1)+10 || state(1) > xl(2)-10
        xlim(ax3d, [state(1)-CAM_HALF, state(1)+CAM_HALF]);
    end
    if state(2) < yl(1)+10 || state(2) > yl(2)-10
        ylim(ax3d, [state(2)-CAM_HALF, state(2)+CAM_HALF]);
    end
    if state(3) < zl(1)+8 || state(3) > zl(2)-8   % la caméra suit AUSSI l'altitude
        zlim(ax3d, [state(3)-CAM_HALF, state(3)+CAM_HALF]);
    end

    %% ── 7h. Tableau de bord ──────────────────────────────────────────────
    v_mag = sqrt(state(4)^2+state(5)^2+state(6)^2);

    switch mode
        case 'water',     mode_str = 'SOUS-MARIN  ~~~~';
        case 'trans_w2a', mode_str = sprintf('EAU→AIR CO2 (%.1fs)', max(0,p.t_co2-trans_timer));
        case 'air',       mode_str = 'AERIEN      ~~~~';
        case 'trans_a2w', mode_str = sprintf('AIR→EAU choc(%.1fs)', max(0,p.t_entry-trans_timer));
    end

    m_tot = p.m_dry + state(11)*p.rho_water*p.V_ballast;
    F_net = (p.rho_water*p.g*p.V_hull - m_tot*p.g)/p.g;
    if strcmp(mode,'water') || strcmp(mode,'trans_a2w')
        if F_net>0.1, buoy='MONTE ↑'; elseif F_net<-0.1, buoy='COULE ↓'; else, buoy='NEUTRE'; end
        extra_str = sprintf('│ Ballast   %7.1f  %%     │\n│ Flottab.  %-12s  │\n', state(11)*100, buoy);
    else
        m = v_mag - v_stall;
        if m>5, st='OK'; elseif m>0, st='ATTENTION'; else, st='DECROCHAGE'; end
        extra_str = sprintf('│ Stall     %-12s  │\n│ (marge %+.1f m/s)        │\n', st, m);
    end

    if auto_mode
        wp_c = waypoints(wp_idx,:);
        d_wp = sqrt((state(1)-wp_c(1))^2+(state(2)-wp_c(2))^2+(state(3)-wp_c(3))^2);
        auto_str = sprintf('│ AUTO  WP%d/%d    (%.0f m)  │\n', wp_idx, n_wp, d_wp);
    else
        % sprintf obligatoire : dans une simple chaîne '...', le \n resterait
        % affiché tel quel (les échappements ne sont interprétés que par sprintf)
        auto_str = sprintf('│ MANUEL    (A = auto)        │\n');
    end
    if gPause, pstr='  [■ PAUSE]'; else, pstr=''; end

    info = sprintf([...
        '┌─────────────────────────────┐\n' ...
        '│ MODE : %-22s│\n' ...
        '├─────────────────────────────┤\n' ...
        '│ Temps     %8.1f  s      │\n' ...
        '├─────────────────────────────┤\n' ...
        '│ POSITION                    │\n' ...
        '│  x        %8.1f  m      │\n' ...
        '│  y        %8.1f  m      │\n' ...
        '│  z        %8.2f  m      │\n' ...
        '├─────────────────────────────┤\n' ...
        '│ VITESSE                     │\n' ...
        '│  Totale   %8.2f  m/s    │\n' ...
        '│  Vz       %8.2f  m/s    │\n' ...
        '├─────────────────────────────┤\n' ...
        '│ ORIENTATION                 │\n' ...
        '│  Tangage  %8.1f  °      │\n' ...
        '│  Cap      %8.1f  °      │\n' ...
        '├─────────────────────────────┤\n' ...
        '│ COMMANDES                   │\n' ...
        '│ Poussée   %8.1f  N      │\n' ...
        '│ Canards   %8.1f  °      │\n' ...
        '│ Gouverne  %8.1f  °      │\n' ...
        '├─────────────────────────────┤\n' ...
        '%s' ...
        '%s' ...
        '└─────────────────────────────┘\n' ...
        '%s'], ...
        mode_str, t, state(1), state(2), state(3), v_mag, state(6), ...
        rad2deg(state(7)), rad2deg(mod(state(8)+pi,2*pi)-pi), ...
        ctrl(1), rad2deg(ctrl(2)), rad2deg(ctrl(3)), ...
        extra_str, auto_str, pstr);

    set(h_dash,'String',info);
    drawnow limitrate;
    elapsed = toc(tic_loop);
    if (p.dt-elapsed) > 0.002,  pause(p.dt-elapsed);  end
end

fprintf('\nSimulation terminée — t=%.1f s,  x=%.1f m,  y=%.1f m,  z=%.2f m\n', t, state(1), state(2), state(3));

%% ================================================================
%% CALLBACKS
%% ================================================================
function onKeyPress(~, ev)
    global gKeys gPause;
    switch ev.Key
        case 'uparrow',    gKeys.up    = true;
        case 'downarrow',  gKeys.down  = true;
        case 'leftarrow',  gKeys.left  = true;
        case 'rightarrow', gKeys.right = true;
        case 'w',          gKeys.w     = true;
        case 's',          gKeys.s     = true;
        case 'f',          gKeys.f     = true;
        case 'v',          gKeys.v     = true;
        case 't',          gKeys.t     = true;
        case 'a',          gKeys.a     = true;
        case 'j',          gKeys.j     = true;
        case 'l',          gKeys.l     = true;
        case 'i',          gKeys.i     = true;
        case 'k',          gKeys.k     = true;
        % '+' : 'add' = pavé numérique, 'equal'/'plus' selon le clavier
        case {'add','equal','plus'},        gKeys.plus  = true;
        % '-' : 'subtract' = pavé numérique, 'hyphen'/'minus' selon le clavier
        case {'subtract','hyphen','minus'}, gKeys.minus = true;
        case 'space',      gPause      = ~gPause;
        case 'escape',     gKeys.escape= true;
    end
end

function onKeyRelease(~, ev)
    global gKeys;
    switch ev.Key
        case 'uparrow',    gKeys.up    = false;
        case 'downarrow',  gKeys.down  = false;
        case 'leftarrow',  gKeys.left  = false;
        case 'rightarrow', gKeys.right = false;
        case 'w',          gKeys.w     = false;
        case 's',          gKeys.s     = false;
        case 'f',          gKeys.f     = false;
        case 'v',          gKeys.v     = false;
        case 't',          gKeys.t     = false;
        case 'a',          gKeys.a     = false;
        case 'j',          gKeys.j     = false;
        case 'l',          gKeys.l     = false;
        case 'i',          gKeys.i     = false;
        case 'k',          gKeys.k     = false;
        case {'add','equal','plus'},        gKeys.plus  = false;
        case {'subtract','hyphen','minus'}, gKeys.minus = false;
    end
end

function onClose(src, ~)
    global gKeys;
    gKeys.escape = true;
    delete(src);
end
