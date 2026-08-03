# Variable Steering — šalabahter za teren

Grana `feature/variable-steering`. Povratak na poznato stanje = flash s `master`.

**Prije polaska:** build + flash doma. Export trenutne konfiguracije.

---

## 0. Doma, prije puta

- [ ] Kompajlira i flesha
- [ ] Boot log: `Variable Steering block initialised to defaults` (samo prvi put)
- [ ] Keya → Variable Steering: sve isključeno, `WAS angle` = `— (uncalibrated)`
- [ ] Graf 17 + 47 → median radi
- [ ] Nosač referentnog IMU-a za kotač spreman ⚠️ *ovo zna pojesti sat vremena*

---

## 1. Snimka šuma — 20 min

Live → Graph, signal **17 (ADS raw)**, 10 Hz.

- [ ] Motor ugašen, 1 min
- [ ] Ler, 1 min
- [ ] Gas gore-dolje, 2 min
- [ ] Export CSV → nosi doma

**Gledaj:** je li baza između impulsa stabilna. Ako baza pliva → sidro je bezvrijedno,
javi i stani s ostatkom.

---

## 2. Masa / konektor — 15 min

- [ ] Ima li impuls i na 5 V napajanju senzora, ili samo na signalnoj žici
- [ ] Konektor WAS-a, masa

Ako je na napajanju → problem je električni i rješava se ovdje, ne softverom.

---

## 3. Kalibracija — 35 min ⚠️ najvažniji korak

**Orbital u 125 ccm. Motor radi. Traktor stoji. Ref IMU na kotaču, "Ref IMU" = OK.**

- [ ] **Dead zone** (motorizirano, sam se vrti) → zapiši rezultat
- [ ] **Sweep**: volan lock-to-lock, polako, 3-4 puta obje strane → Stop
- [ ] Provjeri prije Apply:
  - `samples` **> 0** ⚠️ ako je 0 → ADS se ne čita, stani
  - `fit RMS` — dobro < 1°, prihvatljivo < 2°, iznad → sidro će biti grubo
  - Keya `RMS L/R` kao i inače
- [ ] **Apply** (piše i Keya i WAS kalibraciju odjednom)
- [ ] Ackermann scatter: točke na crvenoj krivulji

**250 ccm se NE kalibrira** — izvodi se (tpd/2, dz×2).
Opcionalno: prebaci na 250, kratki scatter 5 min, provjeri dead zone.

---

## 4. CAN sniff — 10 min, usput

- [ ] CAN plot uključen, prebaci ventil nekoliko puta, gledaj ima li poruke

Čista izvidnica. Ako nema — nema, plan ne ovisi o tome.

---

## 5. Podešavanje u vožnji — 90 min

### 5a. Usporedba (ništa uključeno)
- [ ] Graf: **48** (traktorski WAS) + **22** (Keya) + **49** (innovation)
- [ ] Poklapaju se? Konstantan razmak = zero. Razmak raste s kutom = skala.

### 5b. Fuzija
- [ ] `Enable WAS fusion` ON
- [ ] Gledaj `Gate rejects/s` — **nije nula i to je dobro**, gate radi
- [ ] `WAS offset` treba puzati, ne skakati
- [ ] Ako offset trči u limit → gate preširok ili kalibracija ne valja

### 5c. Orbital, observe-only
- [ ] `Twin orbital handling` ON, `Active ratio` = **125**, `Auto-switch` **OFF**
- [ ] Prebaci ventil ~10× (u mjestu, u vožnji, u zavoju)
- [ ] `Ratio estimate` skoči na ~2? Koliko joj treba? Griješi li ikad?
- [ ] Ako 100 % i brzo → `Auto-switch` ON, odvozi krug

### 5d. Zero iz WAS-a
- [ ] `Zero from WAS` OFF još — samo gledaj `Initial zero` redak kroz par paljenja
- [ ] Usporedi prikazani WAS zero s onim što GPS naknadno ispadne
- [ ] Poklapa se unutar ~1° kroz nekoliko paljenja → upali

---

## 6. Rezerva + kraj — 30 min

- [ ] Export svih logova / CSV-ova
- [ ] Zapiši finalne parametre
- [ ] Ostavi u stanju koje si **stigao provjeriti**

---

## Ako nešto pukne

1. `Enable WAS fusion` **OFF** → vraća se na čisti enkoder + GPS, kao dosad
2. Ne pomaže → flash s `master`
3. Nemoj debugirati fuzijsku petlju u redu voćnjaka. Isključi, odvozi, nosi logove doma.

**Na terenu mijenjaj samo konstante i pragove. Ne prestrukturiraj.**

---

## Brojke koje trebaš znati

| Što | Default | Bilješka |
|---|---|---|
| `Innovation gate` | 4° | veći ako je WAS jako nelinearan |
| `Correction beta` | 0.01 | ≈ 5 s; manji = sporije, mirnije |
| `Max offset rate` | 0.5 °/s | koliko brzo offset smije trčati |
| `Travel per estimate` | 8° | veći = pouzdanija procjena, treba veći zakret |
| `Max angle to zero at` | 4° | drži zero neovisnim o omjeru |

**Signali:** 17 raw ADS · 47 median · 48 WAS kut · 49 innovation · 50 offset · 51 omjer · 52 rejects/s
