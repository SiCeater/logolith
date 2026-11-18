# Logolith — Architecture du traitement du signal & contexte de développement

> Document de contexte pour l'assistant de développement (Claude dans VS Code) **et** pour lecture humaine.
> Synthèse d'une session de conception du 16–17 nov. 2026.
> Statut des affirmations : **[Décidé]** = choix d'Olivier ; **[Reco]** = recommandation non encore validée ; **[Estim.]** = ordre de grandeur à mesurer ; **[À vérifier]** = point non confirmé.

---

## 0. Consignes pour l'assistant

- Olivier est ingénieur en électronique et automatique. Ne pas simplifier techniquement.
- **Il veut comprendre ce qu'il conçoit.** Avant de proposer du code, expliquer le pourquoi : schéma-bloc, ordres de grandeur chiffrés, compromis.
- Langue : français. Style : dense et structuré. Signaler explicitement les incertitudes plutôt qu'inventer une valeur.
- Contraintes firmware **[Décidé]** :
  - C uniquement, drivers **STM32 LL** en bare metal ;
  - **pas de HAL**, nulle part ;
  - CubeMX sert seulement à générer l'init des périphériques ;
  - pas de RTOS : ISR priorisées et tâches à rythme fixe.
- Logolith est écrit **from scratch**. Betaflight et ArduPilot servent de **références d'architecture**, pas de base de code. Le but est un vol **stable et fiable**, pas l'état de l'art.
- **Portabilité** : LithCore (F405) est la cible actuelle. Un passage en F7/H7 est prévu. Ne pas coder en dur les fréquences : coefficients de filtres et gains discrets calculés à l'init à partir de `fs`. Isoler ce qui dépend de la carte.

---

## 1. Le projet

| Nom | Rôle |
|---|---|
| **Aerolith** | Cinelifter custom (châssis, ESC et FC conçus maison) pour porter un hybride type **Sony A7S III**. Prises de vue cinéma, mais dynamiques. |
| **Logolith** | Firmware de vol d'Aerolith. Repo GitHub, licence MIT. Build : STM32Cube for VS Code + presets CMake. Flash : SWD (ST-Link V3E) ou USB-C DFU. |
| **LithCore** | Contrôleur de vol custom à base de STM32F405RGT6. Cible principale de Logolith. |

---

## 2. Matériel : état actuel vs prévu

| Élément | **Actuel (banc)** | **Prévu** |
|---|---|---|
| MCU | STM32F405RGT6 @168 MHz (HSE 8 MHz + PLL), 1 MB flash, 192 KB RAM | Passage en **F7 ou H7** |
| Configuration | **4 moteurs** (quad X) | **8 moteurs, X8 coaxial** |
| Moteurs / hélices | 3115 640 KV, hélices tripales 9050×3 | idem |
| Batterie | **4S** (banc) | **6S** |
| Protocole ESC | **OneShot125 sur TIM8** (CH1–4), sans retour d'info | **DShot bidirectionnel avec retour eRPM** (ESC maison) |
| IMU | ICM-42688-P sur le PCB FC | idem, déporté sur carte fille avec dampering et temperature de fonctionnement fixée en boucle fermée |
| Baromètre | LPS22HH sur le PCB (SPI3), **non utilisé** | utilisé (alt hold) |
| Magnétomètre | **aucun** | uBlox M10 + Mag module prévu |
| GPS | **non utilisé** | uBlox M10 + Mag module prévu |
| Optical flow | PMW3901 (SPI2 ou 3), **non utilisé** | prévu |
| Hauteur sol (ToF) | VL53L1X (I²C), **non utilisé** | prévu (mise à l'échelle du flow) |
| RC | ELRS 2.4 GHz, CRSF 420 kbaud, modules SpeedyBee Nano | — |
| Blackbox | W25Q128 (SPI2, **partagé avec le PMW3901**) | puce pas encore cablée |
| OSD / vidéo | AT7456E (SPI3), VTX RUSH TANK III, 5725–5875 MHz à 25 mW | — |

**Phase actuelle :** seul l'ICM-42688-P est exploité comme capteur de vol. Pas de baro, mag, GPS, flow ni ToF. L'architecture doit déjà prévoir ces entrées (cf. §8), sans les implémenter.

---

## 3. État du firmware (au 17/11/2026)

**Driver ICM-42688-P (fait, validé sur une vraie puce TDK, WHO_AM_I = 0x47)**
- SPI1 mode 3, 21 MHz, CS = PA4 en GPIO logiciel, INT2 → PC4 / EXTI4 sur front montant. DMA2 Stream0 (RX) / Stream3 (TX), canal 3. Burst de 13 octets depuis 0x1F.
- Gyro : ±2000 dps, **ODR 8 kHz**, AAF Bank 1 ≈ **213 Hz** (DELT=21, DELTSQR=440, BITSHIFT=6), UI filter d'ordre 2.
- Accel : ±16 g, ODR 1 kHz, AAF Bank 2 ≈ 42 Hz, UI filter BW = ODR/20.
- Ne jamais écrire 0x00 dans GYRO_CONFIG_STATIC2 : on garde l'AAF et le notch d'usine.
- Calibration gyro au boot via OFFSET_USER (convention mesurée : sortie = brut + offset). Biais résiduel qui dérive encore de ~0,2–0,3 °/s après la mise sous tension → prévoir un suivi lent du biais à l'arrêt.
- Norme accel de référence mesurée au boot (~1025 mg), désormais calibrée.
- Bruit gyro mesuré : ~0,08–0,10 °/s rms. (Les valeurs de bruits citée ici datent d'un mois, si mesure de bruit plus récentes alors les prendre en compte)

**Driver ELRS/CRSF (fait)**
- DMA1 circulaire sur un buffer de 128 octets, interruption IDLE de l'UART.
- Parser à 3 états, CRC8 DVB-S2.
- Armement conditionné à la réception d'une première trame et à LQ > 70.

**Attitude** **[Décidé]**
- Filtre **Mahony** avec gate sur l'accéléromètre et Ki pour suivre le biais.
- Le BNO085 a été évalué puis abandonné : cohérence entre capteurs, latence, gain marginal.

**Sortie moteurs** : OneShot125 sur TIM8. DShot prévu plus tard.

**Fréquence de la boucle rapide :** actuellement réglée à **8 kHz** par Olivier. **Recommandation : revenir à 1 kHz** (cf. §7).

---

## 4. Firmwares open source de référence (état nov. 2026)

| Firmware | Version | Rôle comme référence pour Logolith |
|---|---|---|
| **Betaflight** | 2026.6 stable ; 2026.12 en RC (deux versions par an, juin/déc.) | **Boucle rapide** : chaîne de filtrage gyro, RPM filter, chaîne D, feedforward, I-term relax, chaîne du mixer. La 2026.6 apporte un estimateur de position 3D (GPS / accel / flow / rangefinder / mag) partagé entre GPS Rescue et Position Hold. Autopilote encore **expérimental**. |
| **ArduPilot Copter** | 4.7.1 stable ; 4.7.2 en bêta | **Couches hautes** : EKF3 (fusion flow + ToF + baro + GPS avec sources commutables), cascade position → vitesse → accélération → attitude en quaternion, mixer avec priorités, linéarisation de poussée, compensation de tension, failsafes, X8 coaxial natif. Sur F405 (1 MB), ArduPilot n'embarque pas toutes ses fonctionnalités. Ça ne concerne pas Logolith, mais ça confirme que le F405 est juste pour une stack « complète ». |
| INAV | 9.1.0 | Compromis navigation GPS. Pas d'avantage décisif ici. |
| PX4 | — | Écarté : visé sur H7 / FMU. |

**Synthèse :** prendre la boucle rapide et la chaîne du mixer chez Betaflight, et tout ce qui est au-dessus de la boucle de vitesse angulaire chez ArduPilot.

**Fichiers à lire :**
- Betaflight : `sensors/gyro.c`, `flight/pid.c`, `flight/mixer.c`, `flight/rpm_filter.c`, `flight/dyn_notch_filter.c`, `flight/imu.c`, `fc/rc.c`, `drivers/dshot.c` ;
- ArduPilot : `libraries/AC_AttitudeControl/`, `libraries/AP_Motors/` (`AP_MotorsMulticopter`, `AP_MotorsMatrix`), `libraries/AP_NavEKF3/`.

---

## 5. Lecture du signal flow Betaflight

Source : diagramme « Betaflight Signal Flow v1.0 », vérifié contre `master @ 617e84f11f` (2026-09-16, 2026.12.0-alpha), dérivé du `betaflight_processing` de mzRobot62, GPL-3.0.

### 5.1 Les chaînes, dans l'ordre

**Chemin RC / consigne (`fc/rc.c`, puis `flight/pid.c`)**
- RX `rcData[]` → `rcCommand` (deadband, centrage) → rates et expo (déflexion → deg/s) → rate limit.
- → FPV camera mix (option) → RC smoothing (PT3 à la fréquence PID, option).
- → **setpoint conditioning**. En mode Angle : P × (cible − attitude). C'est aussi là que se font le levelling, l'acro trainer et le launch control.
- → **Σ erreur** = setpoint − gyroADCf.
- Entrées externes : GPS rescue / autopilote injectent une consigne de lacet en amont du rate limit, et un angle roulis/tangage dans le setpoint conditioning.

**Feedforward (chemin séparé)**
- `rawSetpoint` → d(setpoint)/dt à chaque paquet RX (PT1, puis boost, jitter, max rate, yaw hold, moyenne).
- → FF smoothing : PT3 à la fréquence PID, même coupure que le RC smoothing.
- → terme **F** = Kf × FF lissé. Il vaut 0 en roulis/tangage en mode Angle.

**Chaîne de filtrage gyro (`sensors/gyro.c` : `gyroUpdate()` à la fréquence gyro, `filterGyro()` à la fréquence PID)**
1. Gyro(s) → alignement et mise à l'échelle (moyenne de N gyros).
2. **Gyro LPF2 = décimateur.** Il tourne à l'ODR brut du capteur, *avant* le RPM filter (note ① du diagramme).
3. **RPM filter**, un bloc pour les 3 axes. Nécessite le DShot bidirectionnel.
4. Notch 1 et 2 statiques (désactivés par défaut).
5. Gyro LPF1. Sa coupure est réglée dynamiquement selon les gaz (note ②).
6. **Dynamic notch** : SDFT, N pics par axe. Betaflight l'impose avec une boucle PID ≥ 2 kHz, mais c'est une contrainte de *son* implémentation.
7. → `gyroADCf`, qui alimente P/I, la chaîne D et le Σ.

**Chaîne D (`flight/pid.c`)**
- `gyroADCf` → D notch (désactivé par défaut) → D LPF1 (dynamique) → D LPF2 (statique) → −d/dt × Kd.
- **D sur la mesure uniquement**, avec un filtrage plus agressif que le chemin P.
- D_MAX : augmente le D quand max(|FF|, |dgyro|) monte. Ensuite × TPA.

**Termes PID**
- **P** : × TPA (si `tpa_mode = PD`), × anti-gravity sur roulis/tangage, PT1 sur le lacet.
- **I** : *I-term relax* (l'erreur est réduite par un passe-haut sur la consigne, contre le rebond après un mouvement rapide), anti-windup en % de `pidsum_limit`.
- **D** et **F** : voir plus haut.
- Σ → somme PID.

**Calculé dans le mixer, utilisé à la boucle suivante** (note ③)
- Anti-gravity : renforce I et P quand les gaz bougent.
- TPA : atténue P (selon le mode) et D quand les gaz montent.
- Cutoffs du LPF dynamique : règlent gyro LPF1 et D LPF1 selon les gaz.

**Attitude (`flight/imu.c`)** (note ⑤)
- Mahony AHRS (gyro + accel ; mag et cap GPS si présents).
- **Le gyro y passe par son propre PT1 à 200 Hz**, distinct de la chaîne PID.
- C'est une boucle externe plus lente, qui ne sert qu'au setpoint conditioning.

**Gaz**
- Côté RC : entrée gaz (0..1000, × LVC quand la tension est critique) → courbe Bézier (`thr_mid`, `thr_expo`, `thr_hover`) → RC smoothing PT3 (option).
- Côté mixer (`mixTable`) :
  1. normalisation (+ compensation d'inclinaison en Angle/Horizon) ;
  2. throttle limit (option) ;
  3. throttle boost (option) ;
  4. **compensation de linéarisation de poussée** (inverse approximatif de la courbe directe, garde les gaz de stationnaire ; désactivée par défaut) ;
  5. RPM limiter (PID sur le RPM moyen ; nécessite le DShot bidir) ;
  6. throttle override (yaw-spin, launch, alt hold / GPS rescue) → gaz 0..1.

**Mixer (`flight/mixer.c`)**
1. **Limite de la somme PID** (appliquée ici, pas dans `pid.c`).
2. Mix moteurs : R/P/Y × table de mixage.
3. **Ajustement de mix / airmode** (note ④) : désaturation *avant* l'ajout des gaz. Si la plage dépasse 1, le mix est divisé par la plage. Gaz bridés pour qu'aucun moteur ne sature. Airmode off : mix × 0,5..1 en dessous de 50 % de gaz.
4. Mix + gaz par moteur (0..1). Si `thrust_linear > 0` : courbe directe.
5. **Mise à l'échelle dans la fenêtre [motorRangeMin .. motorRangeMax]** :
   - **dynamic idle** remonte le plancher (PID sur la fréquence du moteur le plus lent ; nécessite le DShot bidir) ;
   - **compensation de chute de tension** abaisse le plafond (option).
6. Sortie moteur (trame DShot) → ESC et moteurs → **cellule** (physique) → corps en rotation, mesuré par le gyro.

**Retour d'information**
- DShot bidir : eRPM → Hz (÷ nombre de paires de pôles, PT1 par moteur, en début de tâche de filtrage).
- Alimente : RPM filter, dynamic idle, RPM limiter (RPM moyen non filtré).

### 5.2 Les 5 principes d'architecture à retenir

1. **Chaque bloc a sa fréquence.** Les étiquettes RX / GYRO / ATT le montrent. Pour chaque signal, il faut définir qui le produit, à quelle fréquence, qui le lit, et comment on le maintient entre deux mises à jour (ZOH ou lissage). Le RC smoothing tourne à la fréquence PID sur des données qui arrivent à la fréquence RX.
2. **La consigne et la mesure suivent des chemins séparés.** L'erreur n'est formée qu'au Σ. D sur la mesure avec sa propre chaîne de filtres, F sur la dérivée de la consigne avec son propre lissage. On règle donc séparément la **réponse à la consigne** (FF + mise en forme) et le **rejet de perturbations** (P/I/D). C'est essentiel pour des images propres.
3. **Les gains variables sont calculés à l'itération n−1** (anti-gravity, TPA, cutoffs dynamiques). Ça casse la boucle algébrique entre PID et mixer, et le temps d'exécution de la boucle reste déterministe. **À copier tel quel.**
4. **Le mixer est une chaîne, pas une matrice** : limite de somme PID → matrice → désaturation → linéarisation → fenêtre [plancher, plafond] dont les bornes bougent (dynamic idle, compensation de tension). Le retour eRPM arrive en dessous.
5. **L'autopilote est greffé de l'extérieur.** Il injecte des consignes, et l'attitude ne sert qu'au mode Angle. Il n'y a pas de cascade position → vitesse → accélération → attitude autour d'un estimateur. **C'est la limite de Betaflight pour Aerolith** : pour ces couches, la référence est ArduPilot (§8.4).

---

## 6. Analyse spectrale d'Aerolith

### 6.1 Fréquences de rotation

- Rotation : f_rot = RPM / 60.
- Vitesse à vide : RPM₀ ≈ Kv × V. En charge à plein gaz **[Estim.]** : ≈ 0,8 × Kv × V (à mesurer).
- **En stationnaire, le RPM dépend de la poussée nécessaire** (donc de la masse et du nombre de moteurs), **pas de la tension**. Passer de 4S à 6S change le RPM maximal et le % de gaz en stationnaire, pas la fréquence du stationnaire.
- Paires de pôles **[À vérifier]** : un 3115 est typiquement en 12N14P, soit **7 paires de pôles** (eRPM = 7 × RPM mécanique). À confirmer avant la conversion eRPM → Hz.

| | 4S (14,8 V nom.) | 6S (22,2 V nom.) |
|---|---|---|
| Vitesse à vide | 9 470 tr/min → 158 Hz | 14 200 tr/min → 237 Hz |
| Plein gaz en charge **[Estim.]** | ≈ 7 600 tr/min → **≈ 126 Hz** | ≈ 11 400 tr/min → **≈ 189 Hz** |
| Stationnaire | dépend de la masse, à mesurer | idem 4S à masse égale |

**Mesure possible dès maintenant, sans eRPM :** avec un analyseur de spectre audio (micro, appli téléphone), relever la fréquence de passage des pales en fonction des gaz sur banc à 4S. On en déduit la courbe RPM = f(gaz, Vbat) et la constante de temps moteur (échelon de gaz).

### 6.2 Passage des pales sur les bras du châssis

**Géométrie (donnée par Olivier) :**
- Chaque bras est fait de **deux poutres de 4 mm de large**.
- Elles se rejoignent sous le moteur et s'attachent au châssis en deux points distincts.
- Elles sont écartées d'environ **30°**.
- Chaque pale passe donc au-dessus de deux obstacles par tour. La surface exposée est faible (2 × 4 mm).

**Ce que ça donne :**
- Hélice tripale + deux poutres à 30° → **6 impulsions par tour, non équiréparties**. Ce sont des paires d'impulsions séparées de 30°, qui reviennent tous les 120°.
- Le motif reste **périodique à 3 × f_rot** (période 120°). Ce n'est pas un motif à 6 × f_rot : il ne le serait que si l'écart était de 60°.
- Amplitude relative de l'harmonique k du motif, c'est-à-dire de la raie à 3k × f_rot (en supposant deux impulsions identiques) :

  |1 + e^(−j·2π·k·30°/120°)| = |1 + e^(−jkπ/2)| = 2·|cos(kπ/4)|

| Raie | k | Amplitude relative (1 poutre = 1 ; 2 poutres confondues = 2) |
|---|---|---|
| **3 × f_rot** | 1 | **√2 ≈ 1,41** |
| **6 × f_rot** | 2 | **≈ 0** (interférence destructive) |
| **9 × f_rot** | 3 | √2 ≈ 1,41 |
| **12 × f_rot** | 4 | **2** (constructive) |
| 15 × f_rot | 5 | √2 |
| 18 × f_rot | 6 | 0 |

**Conséquences :**
- **On n'attend pas de raie dominante à 6 × f_rot**, contrairement à l'intuition « on voit deux fois chaque passage ». On attend 3×, 9×, **12×** (renforcée), 15×, etc.
- L'annulation à 6× n'est nette que si l'écart vaut exactement 30°, que les deux impulsions sont identiques, et que les poutres sont **radiales par rapport à l'axe moteur** (sommet sous l'axe). Si le sommet est décalé de l'axe, l'écart angulaire vu par la pale varie le long de l'envergure, ce qui brouille l'annulation. **[À vérifier]** sur la géométrie réelle et par blackbox.
- **Impulsions très courtes, donc spectre large.** Exemple à r ≈ 0,1 m et 126 Hz : vitesse de pale ≈ 79 m/s, donc 4 mm sont franchis en ≈ 50 µs. Le contenu harmonique s'étend loin (plusieurs kHz) avec une décroissance lente. **Le vrai risque, ce sont les harmoniques élevées (9×, 12×) qui se replient**, pas seulement la 3×. L'AAF est la défense principale.
- L'amplitude dépend fortement de la **distance entre l'hélice et le bras** (non communiquée).
- La 1 × f_rot (balourd hélice / moteur) reste à traiter indépendamment des bras.
- **Résonances du châssis** : les modes de flexion verticale et de torsion des bras à double poutre (épaisseur et matériau à préciser) amplifient fortement une raie qui tombe dessus à un certain gaz. À mesurer (tap test logué au gyro, ou balayage de gaz sur banc).

### 6.3 Repliement de spectre (boucle à 1 kHz, Nyquist 500 Hz)

Fréquence repliée : f_a = |f − fs × round(f / fs)|. Valeurs **[Estim.]** à plein gaz :

| Raie | 4S : f → repliée à 1 kHz | 6S : f → repliée à 1 kHz | 6S : repliée à 2 kHz |
|---|---|---|---|
| 1× | 126 → 126 | 189 → 189 | 189 |
| 3× | 379 → 379 | **568 → 432** | 568 |
| 6× (faible attendue) | 758 → 242 | 1137 → 137 | 863 |
| 9× | **1137 → 137** | 1705 → 295 | 295 |
| 12× (renforcée) | 1516 → 484 | 2274 → 274 | 274 |

- **Signature d'une raie repliée en blackbox** : sa fréquence **baisse** quand les gaz montent (ou fait un « V »).
- Atténuation par l'AAF gyro à 213 Hz **[À vérifier : ordre de l'AAF ICM]**. Si l'AAF est d'ordre 2 : ≈ −15 dB à 500 Hz, −17 dB à 570 Hz, −29 dB à 1,14 kHz, −34 dB à 1,5 kHz, plus le UI filter.
- **Une raie repliée ne peut plus être retirée par un passe-bas** (elle est dans la bande). Mais **le repliement est déterministe** : une fois f_rot connu (eRPM), on peut placer un notch RPM sur la **fréquence repliée** calculée. Betaflight ne le fait pas, Logolith peut le faire.
- Les raies 9× / 12× repliées à 130–300 Hz tombent dans la zone que voit le D. Il faut surveiller le bruit sur D.

---

## 7. Fréquences de boucle : pourquoi 8 kHz est une mauvaise idée ici

**Situation :** boucle rapide réglée à 8 kHz (ODR gyro 8 kHz). **[Reco] : revenir à 1 kHz.** Passer à 2 kHz seulement si la blackbox montre des raies repliées gênantes (§6.3).

**1. Le gain de phase est négligeable.**
- Retard de la boucle numérique ≈ 1,5 Ts (bloqueur d'ordre zéro + calcul) : 1 kHz → 1,5 ms ; 8 kHz → 0,19 ms.
- À 10 Hz, ça fait ~5° contre ~0,7°. On gagne donc ~5°.
- Ce qui domine vraiment :
  - **constante de temps moteur + hélice 9"** **[Estim.]** τ ≈ 30–60 ms (à mesurer par échelon). Soit 60–75° de phase à 10 Hz. **Fréquence de coupure probable : 5–15 Hz** ;
  - le filtrage (AAF + LPF + notches) : un PT2 à 100 Hz ≈ 3,2 ms, soit ~17° à 15 Hz.
- Betaflight tourne à 8 kHz pour un 5" de course : τ ≈ 10–15 ms, fréquence de coupure 50–100 Hz, passage de pales ~1,5 kHz. Ce n'est pas le cas d'Aerolith.

**2. Aucune information en plus.** Avec un AAF à 213 Hz, il ne reste presque plus de signal au-dessus de 500 Hz. Tourner à 8 kHz traite des données suréchantillonnées déjà filtrées.

**3. Ce qui casse :**
- **OneShot125** : impulsion de 125–250 µs, période de 125 µs à 8 kHz. **Impossible au-delà de ~4 kHz.**
  - Si la période de TIM8 suit les 8 kHz : CCR > ARR dès que le gaz dépasse 0 % → sortie bloquée à l'état haut → l'ESC reçoit un signal invalide. **⚠ À vérifier avant de monter des hélices.**
  - Si TIM8 est resté à 1 kHz : 7 calculs sur 8 ne servent à rien.
- **DShot300 bidir** (futur) : trame + retournement + réponse ≈ 120–130 µs, donc limite à 8 kHz (Betaflight recommande DShot600 à 8k).
- **Terme D** : la dérivée discrète divise par dt, donc le bruit blanc sort ~8× plus fort. Il faut filtrer plus, et on reperd la latence gagnée.
- **Blackbox** : une page W25Q128 (256 o) se programme en ~0,7 ms, soit ~360 ko/s au maximum. À 8 kHz × ~40 o, on fait ~320 ko/s : la flash sature, et si SPI2 est partagé avec le PMW3901. On serait obligé de décimer les logs.
- **CPU / gigue** : ~30–40 µs par itération (RPM filter X8, chaîne D, PID, mixer 8 moteurs, Mahony) sur 125 µs, soit 25–30 % du CPU en priorité maximale. Les tâches de fond (I²C avec errata, SPI flow, OSD, flash) en pâtissent.
- **Précision numérique** : un biquad à 100 Hz à fs = 8 kHz a ses pôles très près du cercle unité (ω₀ ≈ 0,08 rad), ce qui le rend sensible à l'arrondi float32. Si on monte en fréquence un jour : forme directe II transposée.

**4. Décimation par l'ICM [Reco / À vérifier].** Le rôle de « Gyro LPF2 = décimateur » de Betaflight est tenu chez nous par l'AAF + UI filter de l'ICM. Le plus propre : **régler l'ODR gyro directement à 1 kHz**, avec un AAF ~200–250 Hz, pour que la puce décime en un seul endroit, et réveiller la boucle par UI_DRDY à 1 kHz. À vérifier dans la datasheet (schéma « signal path ») : où se trouvent l'AAF et le notch d'usine par rapport à la décimation à l'ODR, quel UI filter choisir à ODR 1 kHz (BW vs latence), et à quelle fréquence se trouve le notch d'usine par défaut (registres NF_COSWZ, Bank 1).

**5. Plus tard (F7/H7, 6S, X8).** Si on a besoin des notches RPM sur la 3× au-dessus de 500 Hz, la bonne approche reste celle de Betaflight : **découpler** la fréquence gyro (2–4 kHz, notches et décimation) de la fréquence PID (1 kHz).

---

## 8. Architecture cible de Logolith

### 8.1 Tâches et fréquences

| Tâche | Fréquence | Déclencheur | Contenu |
|---|---|---|---|
| **Boucle rapide** | **1 kHz** | ISR EXTI UI_DRDY (prio 0) + DMA SPI1 | lecture → biais → notches (RPM / dyn / statiques) → LPF → PID de vitesse angulaire → limite somme PID → chaîne mixer → sortie TIM8 ; **calcul des gains (TPA, cutoffs dyn…) pour l'itération n+1** |
| **Attitude** | 0,5–1 kHz | dans la boucle rapide ou juste après | Mahony sur un gyro **avec son propre filtrage**, gate accel sur ‖a‖ de référence au boot |
| **Angle** | ~250–400 Hz | timer | erreur en quaternion q_err = q_cur⁻¹ ⊗ q_sp → consigne de vitesse angulaire (double buffer vers la boucle rapide) |
| **RX** | fréquence ELRS | IDLE UART | parse CRSF ; la mise en forme de la consigne s'exécute à la fréquence PID |
| **Nav (futur)** | 50–100 Hz | timer | estimateur flow / ToF / baro / mag / GPS ; boucles position et vitesse |
| **Fond** | — | main loop | blackbox W25Q128, CLI, OSD, télémétrie |

Règles :
- **Chaque signal partagé entre tâches** a un producteur unique, une fréquence documentée et un mécanisme de cohérence (double buffer + flag, ou section critique < 1 µs).
- Les filtres et les PID reçoivent `fs` / `dt` en paramètre. Coefficients recalculés à l'init ou au changement de config.

### 8.2 Chaîne de la boucle rapide

**Mesure**
- ICM (décimé à 1 kHz) → calibration (offset au boot + suivi lent du biais à l'arrêt).
- **Phase actuelle, sans eRPM :**
  - LPF (PT1/PT2) + blackbox pour **mesurer** le spectre réel ;
  - ensuite, au choix : un **notch indexé sur les gaz** (courbe RPM = f(gaz, Vbat) mesurée au micro sur banc, harmoniques 1× et 3×), ou un **notch dynamique SDFT** maison sur 0–500 Hz (la contrainte ≥ 2 kHz est propre à l'implémentation Betaflight).
- **Avec DShot bidir :** RPM filter **par moteur**, harmoniques **1×, 3×**, plus 12× si la blackbox le justifie (pondération des harmoniques, façon `rpm_filter_weights`). Notches sur les **fréquences repliées** calculées si besoin. Coût : 8 moteurs × 2 harmoniques × 3 axes = 48 biquads, ≈ 10 µs à 1 kHz sur F405. En X8, les moteurs du bas tournent plus vite dans le souffle de ceux du haut, donc le suivi doit être par moteur.

**P / I**
- Erreur = consigne − gyro filtré.
- I avec anti-windup conditionnel (pas d'intégration si la sortie sature dans le sens de l'erreur) et **I-term relax**.

**D**
- **Sur la mesure uniquement**, avec sa propre chaîne (LPF plus bas que le P, notch éventuel).
- D_MAX non prioritaire.

**F**
- Dérivée de la consigne mise en forme, sur un chemin séparé et lissé.
- **Pour le cinéma, la mise en forme de la consigne est prioritaire** : RC smoothing (PT3) ou mise en forme limitée en jerk, à la façon de l'`ATC_INPUT_TC` d'ArduPilot.

**Mixer** (ordre Betaflight, avec les priorités d'ArduPilot)
1. Limite de la somme PID.
2. Matrice : 4×4 aujourd'hui, **8×4 en X8 coaxial**. En X8, le lacet vient de l'écart haut/bas sur chaque bras.
3. **Désaturation avec priorités** **[Reco]** : roulis/tangage d'abord, puis poussée, puis **lacet en dernier**. On ne copie pas l'airmode Betaflight : sur un X8 coaxial de plusieurs kg, l'autorité en lacet est faible et sature la première. Remonter les gaz pour la préserver ferait monter l'appareil.
4. **Linéarisation de poussée** (T ∝ ω², grosses hélices ; désactivée par défaut dans Betaflight parce que le 5" en a peu besoin).
5. Fenêtre de sortie [plancher, plafond] :
   - plancher relevé par le **dynamic idle** (nécessite eRPM), contre le décrochage et la désynchronisation des grosses hélices en descente ;
   - plafond ou gains ajustés par la **compensation de tension** (6S sous fort courant ; gains à rapporter à Vnom/Vbat, à la façon de `MOT_BAT_VOLT_MAX/MIN` d'ArduPilot).
6. Sortie : OneShot125 TIM8 aujourd'hui, DShot bidir demain (TIM8 + DMA2).

**À ne pas implémenter** (logique FPV de course) : FPV camera mix, throttle boost, launch control, yaw-spin et crash recovery, boost et jitter du feedforward. Anti-gravity : version simple au mieux. TPA : faible priorité.

### 8.3 Attitude

- Deux consommateurs du même gyro, deux filtrages : la **boucle de contrôle** veut le minimum de retard (notches + LPF juste nécessaire), l'**estimateur** veut une intégration sans biais et sans repliement. Betaflight donne à l'IMU son propre PT1 à 200 Hz.
- Mahony gaté : réduire ou couper le gain accel quand ‖a‖ s'écarte de la référence mesurée au boot ; Ki pour suivre le biais.
- Sans mag (phase actuelle), le lacet n'est pas observable en absolu. Il dérive au rythme du biais gyro résiduel. C'est acceptable en acro et en Angle avec un pilote dans la boucle.

### 8.4 Couches hautes (futur, référence ArduPilot)

- **Cascade** : position → vitesse → accélération → attitude (quaternion, sqrt controller + mise en forme de la consigne) → vitesse angulaire.
- **Estimateur** : Mahony aujourd'hui, puis fusion baro + accel verticale (alt hold), puis flow + ToF (le ToF met le flow à l'échelle ; valider le RangeStatus du VL53L1X, qui se dégrade au soleil), puis mag + GPS. À terme, un EKF à la façon de l'EKF3, avec des sources commutables.
- Prévoir dès maintenant les **points d'injection** de consigne (vitesse angulaire de lacet, angle roulis/tangage, gaz), comme les entrées « autopilot » du schéma Betaflight. Les couches hautes viendront s'y brancher sans toucher à la boucle rapide.

### 8.5 Blackbox (indispensable au réglage)

- Champs = les points de piquage du schéma : gyro brut, gyro filtré, consigne, P/I/D/F par axe, somme PID, sorties moteurs, gaz, Vbat. Plus tard : eRPM, puis l'état de l'estimateur.
- Débit à 1 kHz × ~40 o ≈ 40 ko/s : compatible avec la W25Q128. Gérer la cohabitation avec le PMW3901 si partage de bus (séquencement).
- Premier usage : spectrogramme du gyro brut en fonction des gaz → identifier 1×, 3×, 12×, les raies repliées et les résonances du châssis.

---

## 9. Feuille de route [Reco]

1. **Phase 1 (actuelle)** : 4 moteurs, 4S, OneShot125 TIM8, ICM seul, **boucle à 1 kHz**.
   - Boucle de vitesse angulaire (mode ACRO) avec chaînes P/I, D et F séparées, mixer avec limite et désaturation, blackbox.
   - Mesures banc : courbe RPM / gaz, constante de temps moteur, spectre.
   - Puis mode ANGLE (Mahony + erreur en quaternion).
2. **Phase 2** : DShot bidir + eRPM (ESC maison) → RPM filter par moteur, dynamic idle, conversion eRPM → Hz (paires de pôles à confirmer).
3. **Phase 3** : 6S → re-réglage, linéarisation de poussée, compensation de tension, contrôle du repliement de la 3× à plein gaz (passer à 2 kHz côté gyro si besoin).
4. **Phase 4** : baro → alt hold ; flow + ToF → maintien de position ; mag + GPS → estimateur complet et couches hautes.
5. **Phase 5** : X8 coaxial (matrice 8×4, priorités de désaturation, suivi RPM par moteur).
6. **Phase 6** : portage F7/H7 (couche carte isolée ; fréquences paramétrées ; gyro et PID découplés si utile).

---

## 10. Points ouverts / à vérifier

- [ ] **Fréquence de la boucle rapide** : revenir de 8 kHz à 1 kHz (§7). Vérifier la config TIM8 (pas de CCR > ARR).
- [ ] ICM : ODR gyro à 1 kHz vs 8 kHz + lecture à 1 kHz ; ordre de l'AAF ; position de l'AAF et du notch d'usine dans la chaîne ; fréquence par défaut du notch d'usine.
- [ ] Mesurer : RPM = f(gaz) à 4S, RPM en stationnaire (masse réelle), constante de temps moteur τ.
- [ ] Spectre réel : confirmer l'absence de 6× et la présence de 12× (bras à double poutre à 30°), raies repliées, résonances du châssis.
- [ ] Géométrie des bras : le sommet est-il bien sous l'axe moteur ? distance hélice-bras ? épaisseur et matériau des poutres ?
- [ ] Nombre de paires de pôles du 3115 (typiquement 7).
- [ ] Contention sur SPI2 (W25Q128 / PMW3901).
- [ ] Errata I²C du F4 (transactions non bloquantes, timeouts, procédure de récupération du bus) avant d'exploiter le ToF.
