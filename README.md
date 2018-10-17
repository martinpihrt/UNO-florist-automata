UNO florist automata Readme
====

## UNO florist automata - Zavlažovač květin je založený na platformě Arduino a desce Arduino UNO<br>
(procesor ATMEGA328 a vlastní deska spojů)

Celý projekt je založen jako volný zdroj (lze použít pro nekomerční použití a přizpůsobit si k obrazu svému).<br>
K řídící desce je připojeno teplotní čidlo DS18B20, které měří teplotu půdy (pro možnost regulace zavlažování dle teploty)<br>
K řídící desce je také připojeno čidlo vlhkosti s frekvenčním výstupem 50Hz-5kHz (pro možnost regulace zavlažování dle vlhkosti) https://pihrt.com/elektronika/272-snimac-vlhkosti-pro-zalevaci-automat<br>
Veškeré nastavení automatu se provádí pomocí tlačítek a zobrazení údajů na 16x2 LCD displeji.<br>

## Pro více informací navštivte...
Martin Pihrt - www.pihrt.com: https://pihrt.com/elektronika/400-uno-florist-automata<br>


## Pohled na desku a připojení čidel
<a href="photos\fw1.00\foto (25).png"><img src="photos\fw1.00\foto (25).png" width="80%"></a>
<a href="photos\fw1.00\foto (29).png"><img src="photos\fw1.00\foto (29).png" width="80%"></a>

## Nastavení zavlažování dle dnů od pondělí do neděle
<a href="photos\fw1.00\foto (6).png"><img src="photos\fw1.00\foto (6).png" width="50%"></a>

## Nastavení 3 nezávislých časů pro zavlažování s různou dobou provozu čerpadla
<a href="photos\fw1.00\foto (7).png"><img src="photos\fw1.00\foto (7).png" width="50%"></a>
<a href="photos\fw1.00\foto (8).png"><img src="photos\fw1.00\foto (8).png" width="50%"></a>
<a href="photos\fw1.00\foto (9).png"><img src="photos\fw1.00\foto (9).png" width="50%"></a>
<a href="photos\fw1.00\foto (10).png"><img src="photos\fw1.00\foto (10).png" width="50%"></a>
<a href="photos\fw1.00\foto (11).png"><img src="photos\fw1.00\foto (11).png" width="50%"></a>
<a href="photos\fw1.00\foto (12).png"><img src="photos\fw1.00\foto (12).png" width="50%"></a>

## Regulace dle vlhkosti
a) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost < 40% bude výsledná doba trvání závlahy 1/2 nastavené doby v menu.<br>
b) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost >= 40% a < 60% bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
c) pokud je daný den a je čas programu 1 nebo je čas programu 2 a vlhkost >= 60% a < 80% bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
d) pokud je daný den a je čas programu 1 a vlhkost >= 60% bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
<a href="photos\fw1.00\foto (17).png"><img src="photos\fw1.00\foto (17).png" width="50%"></a>

## Regulece dle teploty
a) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota > 40°C bude výsledná doba trvání závlahy 1/2 nastavené doby v menu.<br>
b) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota >= 30°C a < 40°C bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
c) pokud je daný den a je čas programu 1 nebo je čas programu 2 a teplota >= 20°C a < 30°C bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
d) pokud je daný den a je čas programu 1 a teplota >= 5°C a < 20°C bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
<a href="photos\fw1.00\foto (16).png"><img src="photos\fw1.00\foto (16).png" width="50%"></a>

## Cyklování výstupu
V menu lze nastavit po jaké době běhu programu (1, 2, 3) se má výstup čerpadla vypnout a jako dobu má čekat vypnutý. Tato funkce umožňuje cyklování výstupu pro lepší vsakování vody do půdy (nedojde k přetékání květináče, ale k postupnému vsaku vody)<br>
<a href="photos\fw1.00\foto (13).png"><img src="photos\fw1.00\foto (13).png" width="50%"></a>
<a href="photos\fw1.00\foto (14).png"><img src="photos\fw1.00\foto (14).png" width="50%"></a>

## Smazání do továrního nastavení
Pokud chceme zařízení vymazat do továrního nastavení stiskneme v menu jako níže na obrázku "ANO".<br>
<a href="photos\fw1.00\foto (19).png"><img src="photos\fw1.00\foto (19).png" width="50%"></a><br>
* Výchozí údaje po resetu:<br>
* program 1 čas pro zavlažování 6:00
* doba 30s
* program 2 čas pro zavlažování 18:00 
* doba 30s
* program 3 čas pro zavlažování 9:00 
* doba 30s
* vypnout čerpadlo po 5s
* vypnout na dobu 20s
* nepoužívat regulaci teplotou
* nepoužívat regulaci vhlkostí
* zavlažovat ve dnech po-ne

## Připojení vstupů a výstupů  
* LCD:                    pin 8, 9,  2,  3,  6,  7 (Rs, E,DB4,DB5,DB6,DB7)
* LCD podsvícení:         pin 10
* Tlačítka IN:            pin 14 (A0 dělič 5 tlačítek)
* Čidlo vlhka IN:         pin 5 (měří se frekvence ze snímače vlhkosti)
* Serial:                 pin 0, 1
* I2C:                    pin 18 (A4 SDA), 19 (A5 SCL) používá DS1307 adr. 0x68
* Teplota:                pin 16  DS18B20 rudá +5V, černá GND, modrá (bílá) Data
* Čerpadlo:               pin 17 (pro výstup - čerpadlo)
