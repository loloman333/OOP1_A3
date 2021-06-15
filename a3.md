# Assignment 3 (20 Punkte)

## Vorraussetzungen

mind. **15** bestandene Test Cases bei Assignment 2

## Beschreibung

Dieses Aufbaubeispiel besteht aus zwei Teilen, in welchen das Spiel um weitere Befehle erweitert wird.

## Teil 1: Path-Finding-Befehl (5 Punkte)
### Neuer Befehl
In Teil 1 ist der neue Befehl
```
go <row> <col>
```

zu implementierten. Dabei sind `<row>` und `<col>` Platzhalter für Koordinaten einer Gangkarte (`Tile`). Ein gültiger Befehl wäre somit z. B. `go 3 4`.

### Ausführung des Befehls
Die Eingabe des Befehls führt zur Ausführung eines Path-Finding-Algorithmus. Dabei wird untersucht, ob es einen Weg zwischen
  - jener Gangkarte, auf dem der ziehende `Player` momentan steht, und
  - jener Gangkarte, welche durch die Koordinaten `<row>` und `<col>` beschrieben wird,

gibt.

Existiert ein solcher Weg, so wird der ziehende `Player` an die im Befehl angegebende Stelle auf dem Spielfeld bewegt. Wird der Befehl falsch eingegeben (z. B. mit einer falschen Anzahl an Parametern) oder existiert kein Weg zwischen `Player` und gewünschter Gangkarte, so soll die entsprechende Fehlermeldung ausgegeben (siehe Milestone 2 von Assignment 2).

### Erreichbare Punkte
 - _5 Punkte_ – abhängig von der Anzahl der bestandenen Test Cases


## Teil 2: Künstliche Intelligenz (15 Punkte)

In diesem Teil wird Ihre aktuell existierende Lösung um eine künstliche Intelligenz (KI) erweitert.

Sobald der Befehl `play` eingegeben wird, soll die KI einspringen und die Eingabe für einen Spielzug übernehmen.
Hierzu ist eine einfache KI zu entwickeln, welche die Entscheidungen für Aktionen trifft. 
Die KI berechnet, wo eine Karte in das Spielfeld geschoben wird, wohin sich der ziehende `Player` bewegt, etc. Es wird dabei ohne Bonus-Task-Items gespielt.

### Erreichbare Punkte
Insgesamt können mit der KI 15 Punkte erreicht werden:
 - _bis zu 10 Punkte_ für das Besiegen einer trivialen KI (abhängig von der Anzahl der benötigten Züge)
 - _bis zu weitere 5 Punkte_ für das Besiegen einer nicht-trivialen KI des LV-Teams


#### play

```
play
```

Der Befehl `play` führt einen gesamten Zug des ziehenden `Player`s aus. Dabei kann der Zug aus einer beliebigen Aneinanderreihung der bisher implementierten Commands (`rotate`, `insert`/`i`, `go`, `finish`/`f`, …) bestehen.

Beachten Sie, dass der `play` Befehl den Zug **komplett** ausführt, das heißt von jeder aktuellen Lage bis zum Ende des Spielzuges (Wechsel der/des Spielenden oder Spielende – je nachdem welches Ereignis zuerst eintritt). Der von der KI ausgeführte Zug beendet also auch den aktuellen Zug mit `finish`.

In anderen Worten: `play` erstellt eine Reihe von Befehlen, welche ausgeführt werden sollen. Diese beenden auch den aktuellen Zug mit `finish`, wobei `finish` definitiv der letzte Befehl sein muss und nur einmal vorkommen darf.

Jeder erfolgreich ausgeführte Befehl im Zuge des `play`-Befehls muss in folgender Form aufgelistet werden:

```
[INFO] Command: <ausgeführter Befehl>
```

Es werden **zuerst** alle Befehle (ohne irgendwelche Zwischenausgaben) aufgelistet und erst danach werden sie ausgeführt.

**Wichtig:**
  - Der `play` Befehl darf nicht mehr als 3 Sekunden Berechnungszeit benötigen.
  - Triviale Lösungen (z. B. jeder Zug der KI besteht lediglich aus dem Beenden des Zuges) resultieren in **0 Punkten für diesen Teil des Assignments**.
  - Es ist der KI **nicht** erlaubt, die gegnerischen verdeckten Schatzkarten auszelesen! (Das heißt, die KI darf nicht schummeln!)

##### Beispiel

(RAND_SEED=0, 2 Spieler, gleich nach dem Starten (also ohne z.B. `gamefield off`))

```
RED > play
[INFO] Command: insert top 2
[INFO] Command: go right 1
[INFO] Command: go down 1
[INFO] Command: finish
Player Red(R)    |                 |                 |    Player Yellow(Y)
Treasure: 0/12   V                 V                 V    Treasure: 0/12
        1        2        3        4        5        6        7    
    █████████████████████████████     █████████████     ███████████
    ██                         ██     ██                         ██
   1██ (R)      T21      T01   ██     ██   T02      T15      (Y) ██
    ██ PR                      ██     ██                     PY  ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     ████     █████████████     ██
           ████     ██       ████     ██       ██         ██     ██
-->2   T20 ████     ██   T19 ████     ██   T18 ██         ██     ██<--
           ████     ██       ████     ██       ██         ██     ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     █████████████     ████     ██
    ██       ██     ████              ██                ██       ██
   3██ T03   ██     ████ T04          ██   T05          ██   T06 ██
    ██       ██     ████              ██                ██       ██
    ██     ████     ████     █████████████     █████████████     ██
    ██     ████     ████     ████     ████     ████████████████████
           ████     ████              ██       ██                  
-->4       ████     ████          T24 ██       ██                  <--
           ████     ████              ██       ██                  
    ███████████     █████████████     █████████████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██              ██                ██       ████     ██       ██
   5██ T07      T22 ██   T08      T16 ██   T09 ████     ██   T10 ██
    ██              ██                ██       ████     ██       ██
    ██     ████     █████████████     ████     ████     ████     ██
    ██     ████     ████     ████     ████     ████     ████     ██
           ██       ██       ██       ██       ████              ██
-->6       ██   T17 ██       ██   T23 ██   T14 ████              ██<--
           ██       ██       ██       ██       ████              ██
    ███████████     █████████████     ████     ████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██       ██     ██                           ██              ██
   7██ (G)   ██     ██   T11      T13      T12   ██          (B) ██
    ██       ██     ██                           ██              ██
    ███████████     ███████████████████████████████████████████████
                 Ʌ                 Ʌ                 Ʌ
                 |                 |                 |
Player Red(R)    |                 |                 |    Player Yellow(Y)
Treasure: 0/12   V                 V                 V    Treasure: 0/12
        1        2        3        4        5        6        7    
    █████████████████████████████     █████████████     ███████████
    ██                         ██     ██                         ██
   1██ (R)      T21      T01   ██     ██   T02      T15      (Y) ██
    ██          PR             ██     ██                     PY  ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     ████     █████████████     ██
           ████     ██       ████     ██       ██         ██     ██
-->2   T20 ████     ██   T19 ████     ██   T18 ██         ██     ██<--
           ████     ██       ████     ██       ██         ██     ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     █████████████     ████     ██
    ██       ██     ████              ██                ██       ██
   3██ T03   ██     ████ T04          ██   T05          ██   T06 ██
    ██       ██     ████              ██                ██       ██
    ██     ████     ████     █████████████     █████████████     ██
    ██     ████     ████     ████     ████     ████████████████████
           ████     ████              ██       ██                  
-->4       ████     ████          T24 ██       ██                  <--
           ████     ████              ██       ██                  
    ███████████     █████████████     █████████████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██              ██                ██       ████     ██       ██
   5██ T07      T22 ██   T08      T16 ██   T09 ████     ██   T10 ██
    ██              ██                ██       ████     ██       ██
    ██     ████     █████████████     ████     ████     ████     ██
    ██     ████     ████     ████     ████     ████     ████     ██
           ██       ██       ██       ██       ████              ██
-->6       ██   T17 ██       ██   T23 ██   T14 ████              ██<--
           ██       ██       ██       ██       ████              ██
    ███████████     █████████████     ████     ████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██       ██     ██                           ██              ██
   7██ (G)   ██     ██   T11      T13      T12   ██          (B) ██
    ██       ██     ██                           ██              ██
    ███████████     ███████████████████████████████████████████████
                 Ʌ                 Ʌ                 Ʌ
                 |                 |                 |

Player Red(R)    |                 |                 |    Player Yellow(Y)
Treasure: 0/12   V                 V                 V    Treasure: 0/12
        1        2        3        4        5        6        7    
    █████████████████████████████     █████████████     ███████████
    ██                         ██     ██                         ██
   1██ (R)      T21      T01   ██     ██   T02      T15      (Y) ██
    ██                         ██     ██                     PY  ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     ████     █████████████     ██
           ████     ██       ████     ██       ██         ██     ██
-->2   T20 ████     ██   T19 ████     ██   T18 ██         ██     ██<--
           ████ PR  ██       ████     ██       ██         ██     ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     █████████████     ████     ██
    ██       ██     ████              ██                ██       ██
   3██ T03   ██     ████ T04          ██   T05          ██   T06 ██
    ██       ██     ████              ██                ██       ██
    ██     ████     ████     █████████████     █████████████     ██
    ██     ████     ████     ████     ████     ████████████████████
           ████     ████              ██       ██                  
-->4       ████     ████          T24 ██       ██                  <--
           ████     ████              ██       ██                  
    ███████████     █████████████     █████████████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██              ██                ██       ████     ██       ██
   5██ T07      T22 ██   T08      T16 ██   T09 ████     ██   T10 ██
    ██              ██                ██       ████     ██       ██
    ██     ████     █████████████     ████     ████     ████     ██
    ██     ████     ████     ████     ████     ████     ████     ██
           ██       ██       ██       ██       ████              ██
-->6       ██   T17 ██       ██   T23 ██   T14 ████              ██<--
           ██       ██       ██       ██       ████              ██
    ███████████     █████████████     ████     ████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██       ██     ██                           ██              ██
   7██ (G)   ██     ██   T11      T13      T12   ██          (B) ██
    ██       ██     ██                           ██              ██
    ███████████     ███████████████████████████████████████████████
                 Ʌ                 Ʌ                 Ʌ
                 |                 |                 |
Player Red(R)    |                 |                 |    Player Yellow(Y)
Treasure: 0/12   V                 V                 V    Treasure: 0/12
        1        2        3        4        5        6        7    
    █████████████████████████████     █████████████     ███████████
    ██                         ██     ██                         ██
   1██ (R)      T21      T01   ██     ██   T02      T15      (Y) ██
    ██                         ██     ██                     PY  ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     ████     █████████████     ██
           ████     ██       ████     ██       ██         ██     ██
-->2   T20 ████     ██   T19 ████     ██   T18 ██         ██     ██<--
           ████ PR  ██       ████     ██       ██         ██     ██
    ██     ████     ████     ████     ████     █████████████     ██
    ██     ████     ████     ████     █████████████     ████     ██
    ██       ██     ████              ██                ██       ██
   3██ T03   ██     ████ T04          ██   T05          ██   T06 ██
    ██       ██     ████              ██                ██       ██
    ██     ████     ████     █████████████     █████████████     ██
    ██     ████     ████     ████     ████     ████████████████████
           ████     ████              ██       ██                  
-->4       ████     ████          T24 ██       ██                  <--
           ████     ████              ██       ██                  
    ███████████     █████████████     █████████████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██              ██                ██       ████     ██       ██
   5██ T07      T22 ██   T08      T16 ██   T09 ████     ██   T10 ██
    ██              ██                ██       ████     ██       ██
    ██     ████     █████████████     ████     ████     ████     ██
    ██     ████     ████     ████     ████     ████     ████     ██
           ██       ██       ██       ██       ████              ██
-->6       ██   T17 ██       ██   T23 ██   T14 ████              ██<--
           ██       ██       ██       ██       ████              ██
    ███████████     █████████████     ████     ████████████████████
    ██     ████     ████     ████     ████     ████     ████     ██
    ██       ██     ██                           ██              ██
   7██ (G)   ██     ██   T11      T13      T12   ██          (B) ██
    ██       ██     ██                           ██              ██
    ███████████     ███████████████████████████████████████████████
                 Ʌ                 Ʌ                 Ʌ
                 |                 |                 |
YELLOW >
```


#### whoami

```
whoami
```

Der Befehl gibt den Namen Ihrer KI in der folgenden Zeile aus:

```
[INFO] Team: <NAME>
```

Dabei ist `<NAME>` ein Platzhalter und soll durch den eigenen Teamnamen ersetzt werden. Dieser Name darf beliebig gewählt werden (max. 20 ASCII oder UTF-8 kodierte Zeichen, Kreativität ist sehr erwünscht!) und erscheint später in einer Highscore-Liste, welche das LV-Team verwaltet. Ihr Programm soll beim whoami-Befehl immer den gleichen Teamnamen ausgeben - egal welche\*r Spieler\*in den Befehl ausführt.


##### Beispiel

```
YELLOW > whoami\n
[INFO] Team: Mazerunner\n
YELLOW > i
```


## Spezifikation
### Erlaubte Bibliotheken

-  alle Bibliotheken der C++ Standard Library

### Abgabe
 - 27. Juni um 23:59 (in Österreich gültige Zeit)
 - **im Git-Repository für Assignment 3 am Branch `assignment3`** (der letzte Commit vor der Deadline wird bewertet)
