## Doel
Het leren werken met een breadboard door zelf fysiek een LED te implementeren.

## Files
- n20_blink-external-led
- n25_antwoord-blink-external-led

## Details
Een LED wordt door middel van een + en een - pool verbonden met elektriciteit. Je wilt de LED aan en uit kunnen zetten. Dat gebeurt via een "pin" van de Pico, die op commando een + (= HIGH) signaal uitstuurt of niets (= LOW). De LED is gevoelig voor de richting van de stroom. Hij heeft een + pool (anode genoemd) en een - pool (cathode). De anode moet verbonden worden met een pin van de Pico, cathode met de aarde. Zie het schema hieronder.

![blink-external-led_bb](uploads/2e0753eae667c6811d520caec119ebc2/blink-external-led_bb.png)

Een LED mag je NOOIT zomaar met een pin en de aarde verbinden: dat veroorzaakt een plof en een kapotte Pico (= 6 euro + verzendkosten en *zelf* solderen!). Zet ALTIJD een weerstand tussen de pin en de anode: dit zie je in het schema terug. De waarde van de weerstand is te bepalen met de wet van Ohm. Gegeven een spanning van 3.3V en een maximale stroom van 20 mA = 0.02 A en de wet van Ohm:

```
    V   3.30
R = - = ---- = 160 ohm
    I   0.02
```
De weerstand die daar het meest mee overeenkomt is 220 Ohm.

## Variaties
Probeer dezelfde sketch uit door de LED met pin 8 te verbinden. Als je de Pico aanzet zou de LED nu niet meer mogen oplichten. Verbindt hem met pin 6: probeer je teamgenoten uit te leggen waarom je ziet wat je ziet.

### PAS OP: 
Als er staat dat je de LED moet verbinden met een nieuwe pin kan dat echt niet zonder het weerstandje!