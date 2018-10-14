UNO florist automata Readme
====

## UNO florist automata - Zavlažovač květin je založený na platformě Arduino a desce Arduino UNO (tedy procesoru ATMEGA328 a vlastní desce spojů).

Celý projekt je založen jako volný zdroj (lze použít pro nekomerční použití a přizpůsobit si k obrazu svému).<br>
K řídící desce je připojeno teplotní čidlo DS18B20, které měří teplotu půdy (pro možnost regulace zavlažování dle teploty)<br>
K řídící desce je také připojeno čidlo vlhkosti s frekvenčním výstupem 50Hz-5kHz (pro možnost regulace zavlažování dle vlhkosti) https://pihrt.com/elektronika/272-snimac-vlhkosti-pro-zalevaci-automat<br>
Veškeré nastavení automatu se provádí pomocí tlačítek a zobrazení údajů na 16x2 LCD displeji.<br>

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
a) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost < 40 bude výsledná doba trvání závlahy 50% nastavené doby v menu.<br>
b) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost >= 40 a < 60 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
c) pokud je daný den a je čas programu 1 nebo je čas programu 2 a vlhkost >= 60 a < 80 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
d) pokud je daný den a je čas programu 1 a vlhkost >= 60 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
<a href="photos\fw1.00\foto (17).png"><img src="photos\fw1.00\foto (17).png" width="50%"></a>

## Regulece dle teploty
a) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota > 40 bude výsledná doba trvání závlahy 50% nastavené doby v menu.<br>
b) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota >= 30 a < 40 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
c) pokud je daný den a je čas programu 1 nebo je čas programu 2 a teplota >= 20 a < 30 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
d) pokud je daný den a je čas programu 1 a teplota >= 5 a < 20 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu.<br>
<a href="photos\fw1.00\foto (16).png"><img src="photos\fw1.00\foto (16).png" width="50%"></a>

## Cyklování výstupu
v menu lze nastavit po jaké době běhu programu (1, 2, 3) se má výstup čerpadla vypnout a jako dobu má čekat vypnutý. Tato funkce umožňuje cyklování výstupu pro lepší vsakování vody do půdy (nedojde k přetékání květináče, ale k postupnému vasku vody)<br>
<a href="photos\fw1.00\foto (13).png"><img src="photos\fw1.00\foto (13).png" width="50%"></a>
<a href="photos\fw1.00\foto (14).png"><img src="photos\fw1.00\foto (14).png" width="50%"></a>

## Pro více informací navštivte
Martin Pihrt - www.pihrt.com: https://www.pihrt.com<br>


Ve výstavbě...
