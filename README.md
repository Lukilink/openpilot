# openpilot für den VW T3 (Branch `t3`)

Dieser Fork ist **openpilot v0.11.2** (comma `release-tizi`, für den comma 3X) mit wenigen kleinen Änderungen.
Damit läuft openpilot in einem **VW T3**, der mit Toyota-Teilen und eigenen Arduino-Steuergeräten umgebaut ist.

> ⚠️ Eigenbau, nicht von comma unterstützt. Nur auf abgesperrtem Gelände testen. Du bist selbst verantwortlich.

Die originale openpilot-Beschreibung steht in [README_openpilot.md](README_openpilot.md).

## Die Idee in einem Satz
Der T3 tut auf dem CAN-Bus so, als wäre er ein **Toyota Corolla 2017**. openpilot glaubt also, es fährt einen Corolla, und lenkt, gibt Gas und bremst.

## Was im Auto steckt
| Teil | Aufgabe |
|---|---|
| Toyota Corolla Lenkung (EPS) | lenkt, wenn openpilot es sagt |
| Toyota Lenkwinkelsensor | meldet den Lenkwinkel (im T3 **kopfüber** eingebaut) |
| Toyota Radar | erkennt Autos vor dir |
| MAIN-Arduino | Tempo, Tasten, Blinker; spielt Motorsteuerung, Karosserie und DSU |
| GAS-Arduino | öffnet die Drosselklappe |
| BRAKE-Arduino | tritt die Bremse |
| DSU/CAM-Arduino | „weckt“ das Radar und ersetzt die Toyota-Kamera |

Die Arduino-Programme liegen getrennt im Projektordner (`ECU/MAIN`, `ECU/GAS`, `ECU/BRAKE`, `ECU/DSU_CAM`).

## Was in diesem Fork geändert ist
Nur 5 Dateien. Jede Änderung ist im Code mit `VW T3 retrofit` markiert.

1. **`launch_env.sh`** – openpilot startet immer als Corolla 2017
   (`FINGERPRINT="TOYOTA_COROLLA"`). Grund: Im T3 gibt es keine Toyota-Motorsteuerung, die sich bei openpilot melden könnte.

2. **`opendbc_repo/opendbc/car/toyota/interface.py`**
   - openpilot regelt **Gas und Bremse** selbst. Beim echten Corolla 2017 macht das openpilot heute nicht mehr, darum diese Zeile.
   - **Stop-and-Go** an: Die Bremse hält den T3 im Stand.
   - Werte vom T3 aus dem alten Fork `084_OLD_CAR`: Radstand **2,455 m**, Lenkübersetzung **14**, Lenk-Regler **PID** mit kp 0,7 / ki 0,01 / kf 0,00007818594.

3. **`opendbc_repo/opendbc/car/toyota/carstate.py`**
   - Der Lenkwinkelsensor ist **kopfüber** eingebaut und 23° verdreht. Der Winkel wird deshalb umgedreht: `-(Winkel + 23)` (wie im alten Fork). Auch die Lenkgeschwindigkeit wird umgedreht.
   - Der **Abstandstaster** (Taster 4 am MAIN-Arduino) kommt über ein freies Bit (`LIGHT_STALK → AUTO_HIGH_BEAM`) und schaltet den Fahrstil (nah / normal / weit).

4. **`opendbc_repo/opendbc/car/toyota/radar_interface.py`** – Wenn das Radar „nicht bereit“ meldet, blockiert openpilot nicht, sondern fährt nur mit der Kamera.

5. **`openpilot/selfdrive/controls/controlsd.py`** – Die Lenkübersetzung bleibt fest bei 14 (wie im alten Fork), denn der Lenk-Regler wurde dafür eingestellt. Zum Ausprobieren der gelernten Übersetzung: diese 2 Zeilen löschen.

**Bewusst nicht übernommen** aus `084_OLD_CAR`: abgeschaltete panda-Sicherheit (`allOutput`), „CAN immer gültig“, comma pedal (gibt es nicht mehr), eigenes Auto `OLD_CAR`. Die Sicherheitsprüfungen von openpilot und panda bleiben voll an.

## Installieren auf dem comma 3X
1. Am 3X: Einstellungen → Software → deinstallieren (oder neues Gerät).
2. Bei der Einrichtung als Software-URL eingeben: `installer.comma.ai/Lukilink/t3`
3. Fertig. Updates kommen von diesem Branch.

## Bedienung im T3
| Taster | Funktion |
|---|---|
| 1 | Tempomat-Hauptschalter ein/aus |
| 2 | **SET**: openpilot an (Tempo + 3 km/h). Wenn an: −5 km/h |
| 3 | **RESUME**: openpilot an mit altem Tempo. Wenn an: +5 km/h |
| 4 | Abstand / Fahrstil wechseln |

openpilot geht aus bei: Bremse, Kupplung, Taster 1, Fehler. Gas geben pausiert nur die Längsregelung, die Lenkung bleibt an.
Einschalten geht erst ab 30 km/h (Schaltgetriebe).

## Auf eine neue openpilot-Version updaten
Den einen Commit `VW T3 retrofit …` auf den neuen `release-tizi` übertragen:
```bash
git fetch upstream release-tizi
git rebase upstream/release-tizi
```
Wenn comma die Toyota-Dateien umbaut, gibt es Konflikte – dann die markierten Stellen von Hand neu einfügen.
