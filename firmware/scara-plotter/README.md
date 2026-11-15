# Firmware plotter SCARA (Arduino Mega + RAMPS)

Marlin 2.1.2.6 con la cinematica SCARA adattata a un braccio da disegno, più un bridge WiFi per ESP32.
Accetta G-code da USB oppure da un ESP32 collegato in wireless al programma sul PC.

```
 PC ──USB──────────────────────────────┐
                                       ├─> Mega 2560 + RAMPS (Marlin SCARA) ─> spalla, gomito, servo penna
 PC ──WiFi/TCP:8888──> ESP32 ──UART────┘
```

Non si tiene una copia di Marlin nel repository. `prepare.py` scarica la versione fissata, applica una patch
piccola e riscrive `Configuration.h` con i valori calcolati da `machine.json`. Gli stessi parametri dello
studio interattivo (`tools/scara/index.html`) vanno in `machine.json`, quindi il firmware ha le misure
che hai provato nel modello.

## Cosa fa la patch

Marlin ha già `MORGAN_SCARA`, ma pensato per stampanti. Per un plotter con finecorsa reali servivano queste
modifiche (`patches/marlin-2.1.2.6-scara-plotter.patch`, tutte attive solo con `SCARA_JOINT_HOMING`):

| Problema in Marlin | Modifica |
|---|---|
| `G28` non muove i giunti: con Morgan `homeaxis()` rifiuta X e Y | Homing in spazio giunti, uno alla volta, contro i finecorsa; poi movimento di parcheggio al centro del foglio |
| Ogni `G1` finiva in (0,0) con i limiti software attivi (il raggio massimo non viene mai inizializzato su SCARA) | Limite rettangolare sul foglio al posto di quello circolare |
| Theta salta di 360° quando il bersaglio attraversa la direzione −X: il braccio farebbe un giro intero | Theta viene tenuto a ±180° dal centro della sua corsa (`SCARA_THETA_CENTER`) |
| L'asse Y è sempre l'angolo assoluto dell'avambraccio (motori entrambi alla base) | `SCARA_ELBOW_RELATIVE`: asse Y = angolo del gomito rispetto al braccio 1 (motore sul braccio) |
| Solo gomito con θ2 > 0 | `SCARA_ELBOW_NEGATIVE`: gomito con θ2 < 0 |

## Convenzioni (le stesse dello studio interattivo)

- **θ1** angolo della spalla rispetto alla direzione di montaggio, antiorario positivo. **θ2** angolo del gomito rispetto al braccio 1.
- Il **G-code ha l'origine nell'angolo in basso a sinistra del foglio**, con gli assi paralleli ai lati. Se nello studio il foglio è ruotato, il G-code ruota con lui.
- Asse X di Marlin = angolo della spalla. Asse Y = angolo del gomito: assoluto (`motore_gomito: "base"`, avambraccio mosso con cinghia dal motore alla base) o relativo (`"giunto"`, motore sul braccio 1).
- `G1` segue una retta sul foglio. `G0` è un movimento diretto nello spazio giunti: va bene per spostarsi con la penna alzata, ma la traiettoria non è una retta. Per disegnare usa sempre `G1`.

## Configurare la macchina

1. Nello studio interattivo regola bracci, finecorsa, foglio e premi **Copia configurazione**.
2. Incolla il JSON in `machine.json` (i campi `bracci_mm`, `finecorsa_deg`, `foglio`, … hanno già gli stessi nomi) e controlla la sezione `hardware`:

| Campo | Significato |
|---|---|
| `driver` | `A4988`, `DRV8825`, `TMC2209`… (stesso nome di Marlin) |
| `passi_motore`, `microstep` | 200 e 16 con i tre ponticelli sotto il driver A4988 |
| `rapporto_spalla`, `rapporto_gomito` | Riduzione tra motore e giunto (cinghia, riduttore). Passi per grado = passi × microstep × rapporto / 360 |
| `motore_gomito` | `"base"` o `"giunto"`, vedi sopra |
| `inverti_direzione` | Inverte il verso del motore. Si verifica al primo avvio |
| `homing` | Lato del finecorsa di ogni giunto (`"min"` o `"max"`) e, facoltativo, `posa`: gli angoli del braccio quando i finecorsa scattano |
| `finecorsa_normalmente_chiusi` | `true` se l'interruttore è collegato tra massa e segnale in modo NC (il caso più comune), `false` per NO |
| `finecorsa_massimo_attivo` | Usa anche i finecorsa dell'altro lato come protezione |
| `velocita_*`, `accelerazione_*` | In gradi/s e gradi/s² per i giunti; in mm/s² per il disegno |
| `seriale` | Baud USB e baud/porta dell'ESP32 (`esp32_porta`: 1, 2, 3 o `null`) |

Poi:

```bash
python3 prepare.py --check      # controlla solo la geometria
python3 prepare.py              # prepara build/Marlin
pio run -d build/Marlin -e mega2560 -t upload
```

`--check` stampa la torre in coordinate G-code, gli angoli che il foglio richiede a ogni giunto, la posa di homing e dice
se il foglio non sta in zona sicura. **Segnala un errore anche se il finecorsa di homing cade dentro il campo di lavoro**:
con il gomito mosso dalla base, un finecorsa a 105° assoluti non permette di raggiungere punti che richiedono 127°.
In quel caso cambia lato di homing o la `posa`.

Serve PlatformIO (`pip install platformio`), git e Python 3.

## Collegamenti RAMPS 1.4

| Funzione | Dove |
|---|---|
| Motore spalla | Slot driver X |
| Motore gomito | Slot driver Y |
| Finecorsa spalla | X-MIN (pin 3) e X-MAX (pin 2) |
| Finecorsa gomito | Y-MIN (pin 14) e Y-MAX (pin 15) |
| Servo penna | Header SERVO, segnale D11 (`M280 P0`) |
| ESP32 | Pin 18 (TX1) e 19 (RX1) del Mega, che sulla RAMPS sono i segnali Z-MIN e Z-MAX. Per questo i finecorsa Z sono disattivati |

Lo slot Z resta senza motore: l'asse Z esiste solo perché Marlin lo richiede per i limiti software, e le parole `Z` nel G-code non muovono nulla.

Il collegamento ESP32 (livelli logici diversi, 5 V sul Mega e 3,3 V sull'ESP32):

```
ESP32 GPIO17 (TX2) ─────────────────> Mega pin 19 (RX1)      diretto, 3,3 V è letto come alto
Mega pin 18 (TX1) ──[1k]──┬─────────> ESP32 GPIO16 (RX2)     partitore: 1k in serie, 2k a massa
                          └──[2k]── GND
GND ESP32 ───────────────────────── GND Mega
```

Alimenta l'ESP32 dal suo USB oppure dal 5 V della RAMPS (pin VIN/5V della scheda), non dal 3,3 V.
Non collegare mai il TX a 5 V del Mega direttamente a un pin dell'ESP32.

## Bridge ESP32

```bash
cd esp32-bridge
cp include/secrets.example.h include/secrets.h      # rete WiFi e password OTA
pio run -t upload
```

Il bridge passa i byte nei due sensi senza interpretarli. Il programma sul PC si collega a `scara-plotter.local:8888`
(o all'IP mostrato sulla seriale a 115200) e parla con Marlin come se fosse una porta USB.
La pagina `http://scara-plotter.local/` mostra lo stato, `/log` le ultime risposte della macchina.
Se la rete non si trova entro 20 s parte un punto di accesso `SCARA-PLOTTER` su 192.168.4.1.
Aggiornamenti via WiFi: `pio run -t upload --upload-port scara-plotter.local`.

Il bridge non ha autenticazione sul G-code: usalo solo su una rete di cui ti fidi. Usa **un solo canale alla volta**
(USB oppure WiFi) durante un disegno: Marlin accetta comandi da entrambi e le righe si mescolerebbero.

## Inviare un disegno

```bash
python3 tools/gcode_stream.py disegno.gcode --port /dev/ttyACM0 --init G28      # USB
python3 tools/gcode_stream.py disegno.gcode --tcp scara-plotter.local --init G28 # WiFi
```

Lo script tiene in volo fino a 4 righe e 120 byte (i limiti di Marlin), così la coda dei movimenti non si svuota nemmeno con la
latenza del WiFi. Ctrl-C ferma subito la macchina (`M410`). Anche UGS, Pronterface, bCNC e simili funzionano:
è una normale seriale Marlin a 115200 baud (USB) o una porta TCP (con un adattatore seriale-su-TCP).

Il G-code per un plotter di solito comincia con:

```gcode
G28                  ; homing: i due giunti vanno ai finecorsa, poi al centro del foglio
G90                  ; coordinate assolute, origine = angolo in basso a sinistra del foglio
M280 P0 S60          ; penna su  (usa i tuoi angoli)
G0 X20 Y20 F6000     ; spostamento a penna alzata
M280 P0 S90          ; penna giù
G4 P150              ; attesa 150 ms perché il servo arrivi
G1 X120 Y20 F3000    ; traccia
```

| Comando | Effetto |
|---|---|
| `G28` | Homing dei giunti + parcheggio al centro. Senza homing Marlin rifiuta i movimenti |
| `G1 X Y F` | Retta sul foglio (F in mm/min) |
| `G0 X Y` | Movimento diretto dei giunti |
| `M280 P0 S<gradi>` | Servo penna |
| `M114` | Posizione corrente (X Y in mm; in coda gli angoli Theta e Psi) |
| `M119` | Stato dei finecorsa |
| `M92`, `M203`, `M201` | Passi/grado, velocità, accelerazioni (valgono fino al riavvio, i valori durevoli vanno in `machine.json`) |
| `M410` | Ferma subito e svuota la coda |
| `M112` | Arresto di emergenza (poi serve il reset) |

## Primo avvio

1. **Senza braccio collegato ai motori**, o con i motori scollegati dalla trasmissione, carica il firmware e collegati con un terminale a 115200.
2. `M119` deve dire `open` per tutti i finecorsa. Premine uno a mano e rileggi: se è invertito, cambia `finecorsa_normalmente_chiusi`.
3. Alimenta i motori, regola la corrente dei driver, poi con il braccio libero e lontano dagli estremi lancia `G28` tenendo la mano sull'alimentazione:
   - Se un giunto va dal lato opposto al suo finecorsa, inverti `inverti_direzione` per quel giunto. L'homing ha comunque un tetto di corsa e dà `Homing Failed` invece di girare all'infinito.
   - Se non si ferma al finecorsa, controlla il cablaggio e `M119`.
4. Dopo `G28`, `M114` deve riportare il centro del foglio (nell'esempio di default X148.5 Y105). Muovi la penna con `G1` ai quattro angoli del foglio (a 10 mm dal bordo) e misura gli errori. Se non tornano, controlla `L1`, `L2`, i rapporti di trasmissione e la posizione della torre.
5. Solo dopo, collega la penna e prova un disegno.

## Come è stato verificato

Non c'è stato un braccio vero. Il firmware compilato è stato eseguito in un Mega2560 simulato (simavr) con la RAMPS,
i finecorsa e un braccio virtuale. Il banco di prova `tests/run_sim.py` invia G-code come farebbe un host e controlla che la punta
arrivi dove richiesto, con l'homing incluso. Provato su cinque configurazioni: quella di default (gomito alla base), gomito sul giunto,
gomito negativo, foglio ruotato con motori invertiti, finecorsa NO e pochi passi/grado, foglio oltre la direzione −X.
Errore massimo della punta 0,19 mm.

PlatformIO non era raggiungibile nell'ambiente in cui è stato scritto. La compilazione è stata verificata con avr-gcc 7.3
(la stessa versione del toolchain di PlatformIO) e il core Arduino AVR 1.8.6, senza passare da `pio`: il comando `pio run`
qui sopra è quello standard di Marlin ma non è stato eseguito.

Quello che il simulatore non copre: coppia e velocità reali dei motori, rimbalzi dei finecorsa, gioco meccanico, il servo,
l'ESP32 (compilato solo contro stub, non su una scheda) e il timing dei driver. La precisione reale dipende dalla meccanica.

```bash
python3 -m unittest discover -s tests -v        # generatore e streamer, senza hardware
# simulazione (serve: apt install simavr libsimavr-dev libelf-dev gcc)
python3 prepare.py && pio run -d build/Marlin -e mega2560
python3 tests/run_sim.py --elf build/Marlin/.pio/build/mega2560/firmware.elf
# altre configurazioni: prepare.py --config tests/configs/X.json --out build/X ; run_sim.py --config tests/configs/X.json
```

## Limiti noti

- Un solo verso del gomito per disegno (nessun ribaltamento a metà tratto).
- Con θ1 e θ2 vicini al limite dei finecorsa, la zona sicura è quella dello studio (margine `margine_finecorsa_deg`). Marlin limita il movimento al rettangolo del foglio, non agli angoli: gli angoli sono protetti dai finecorsa fisici, che quindi vanno montati.
- La penna ha il servo ma non c'è un comando di alzata automatico dal G-code: usa `M280`.
- Marlin è fissato alla 2.1.2.6. Con un'altra versione la patch potrebbe non applicarsi.
