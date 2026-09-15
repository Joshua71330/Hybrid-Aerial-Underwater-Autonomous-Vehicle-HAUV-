#  HAUV — Hybrid Aerial-Underwater Autonomous Vehicle

[![School](https://img.shields.io/badge/ENSTA%20campus%20Brest-Projet%20Syst%C3%A8mes-002E5D)]()
[![Status](https://img.shields.io/badge/status-%C3%A9tude%20de%20faisabilit%C3%A9-orange)]()
[![Simulation](https://img.shields.io/badge/simulation-MATLAB-orange?logo=mathworks)]()

> 🇫🇷 Conception d'un drone hybride autonome capable de voler à voilure fixe puis de plonger pour explorer un fond marin.
> 🇬🇧 Design of an autonomous hybrid drone able to fly as a fixed-wing aircraft, then dive to explore the seabed.

**[🇫🇷 Version française](#-français)** · **[🇬🇧 English version](#-english)**

![Enveloppe CFD du drone HAUV — vues de dessus, de face et de profil](.Screenshot_4.png)

---

## 🇫🇷 Français

### Aperçu du projet

Ce dépôt documente **HAUV**, un projet de conception réalisé à **l'ENSTA (campus de Brest)** dans le cadre du "Projet Découverte de Systèmes" (année universitaire 2025-2026). L'objectif : concevoir un **drone hybride aéro-sous-marin autonome** capable de :

1. parcourir **~10 km en vol à voilure fixe** ;
2. **plonger jusqu'à 10 m de profondeur** et explorer une zone sous-marine sur **100 m** ;
3. **revenir de façon autonome** à son point de départ, sans intervention de l'opérateur après le lancement.

La mission est définie par des points GPS fournis avant le lancement. Le projet couvre l'ensemble de la chaîne de conception : étude de l'existant, dimensionnement mécanique, architecture électronique, et simulation numérique.

### Solution retenue

- **Inspiration** : le prototype [AquaMAV](https://royalsocietypublishing.org/rsfs/article/7/1/20160085/34991/Wind-and-water-tunnel-testing-of-a-morphing) de l'Imperial College London.
- **Structure** : fuselage cylindrique en carbone de **80 mm de diamètre**, ailes repliables se déployant en vol.
- **Transition eau → air** : éjection d'un ballast par **CO₂ haute pression** (~57 bars).
- **Électronique embarquée** : autopilote **Pixhawk 6C** + **Raspberry Pi 4B**, GPS en phase aérienne, estimation inertielle en immersion, 5 servomoteurs étanches Hitec HS‑5086WP.
- **Simulation** : modèle dynamique 3D sous **MATLAB** (intégration d'Euler) avec une **machine à états finis** à 4 modes, validant qualitativement le comportement et les transitions entre milieux.

### Résultats clés de l'étude de faisabilité

| Critère | Estimation |
|---|---|
| Masse totale | 1,94 kg (< 2 kg cible) |
| Coût matière | ~1 604 € (enveloppe max. 5 000 €) |
| Temps de réalisation estimé | ~420 h (2,6 hommes-mois) |
| Coefficient de sécurité structurel | 4 (formule de Barlow) |

> Le prototype **n'a pas encore été fabriqué** : ce projet correspond à l'étude de conception et de faisabilité (mécanique + électronique + simulation), pas encore à sa réalisation physique. Les prolongements identifiés sont les essais d'éjection CO₂, la calibration du compas sous charge moteur, et la validation expérimentale de la transition eau-air.

### Contenu du dépôt

- [`Drone_ROV_merged.pdf`](./Drone_ROV_merged.pdf) — rapport technique complet (47 pages) : état de l'art, analyse fonctionnelle, dimensionnement aérodynamique et structurel, architecture électronique, simulation MATLAB, synthèse coûts/temps.
- `drone_final_check.png` — visualisation de l'enveloppe CFD finale du drone (vues de dessus, de face et de profil).

Les fichiers CATIA et la simulation complète sont référencés en annexe du rapport (lien SharePoint ENSTA).

### Équipe

Joshua Fadel, Antoine Meron, Paul Mater, Lucien Bottagisio — ENSTA, campus de Brest — promotion 2025-2026.

---

## English

### Project overview

This repository documents **HAUV**, a design project carried out at **ENSTA (Brest campus)** as part of the "Projet Découverte de Systèmes" course (2025-2026 academic year). The goal: design an **autonomous hybrid aerial-underwater drone** able to:

1. cover **~10 km in fixed-wing flight**;
2. **dive down to 10 m** and explore an underwater area over **100 m**;
3. **autonomously return** to its starting point, with no operator input after launch.

The mission is defined by GPS waypoints supplied before launch. The project spans the full design chain: state-of-the-art review, mechanical sizing, electronics architecture, and numerical simulation.

### Chosen solution

- **Inspiration**: the [AquaMAV](https://royalsocietypublishing.org/rsfs/article/7/1/20160085/34991/Wind-and-water-tunnel-testing-of-a-morphing) prototype from Imperial College London.
- **Structure**: **80 mm diameter** cylindrical carbon fuselage with foldable wings that deploy in flight.
- **Water → air transition**: ballast ejection via **high-pressure CO₂** (~57 bar).
- **Onboard electronics**: **Pixhawk 6C** autopilot + **Raspberry Pi 4B**, GPS during flight, inertial estimation while submerged, 5 waterproof Hitec HS‑5086WP servos.
- **Simulation**: a 3D dynamic model in **MATLAB** (Euler integration) with a 4-mode **finite-state machine**, qualitatively validating behaviour and transitions between the two environments.

### Key feasibility results

| Criterion | Estimate |
|---|---|
| Total mass | 1.94 kg (< 2 kg target) |
| Material cost | ~€1,604 (max budget €5,000) |
| Estimated build time | ~420 h (2.6 person-months) |
| Structural safety factor | 4 (Barlow's formula) |

> The prototype **has not been built yet** — this project is the design and feasibility study (mechanical + electronics + simulation), not the physical build. Identified next steps include CO₂ ejection testing, compass calibration under motor load, and experimental validation of the water-air transition.

### Repository contents

- [`Drone_ROV_merged.pdf`](./Drone_ROV_merged.pdf) — full technical report (47 pages): state of the art, functional analysis, aerodynamic and structural sizing, electronics architecture, MATLAB simulation, cost/time summary.
- `drone_final_check.png` — final CFD envelope of the drone (top, front, and side views).

CATIA files and the full simulation are referenced in the report's appendix (ENSTA SharePoint link).

### 👥 Team

Joshua Fadel, Antoine Meron, Paul Mater, Lucien Bottagisio — ENSTA, Brest campus — class of 2025-2026.
