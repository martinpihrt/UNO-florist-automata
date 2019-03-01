UNO florist automata Readme  
====

## UNO florist automata - Zavlažovač květin je založený na platformě Arduino a desce Arduino UNO<br>
(procesor ATMEGA328 a vlastní deska spojů)

Celý projekt je založen jako volný zdroj (lze použít pro nekomerční použití a přizpůsobit si k obrazu svému).<br>
K řídící desce je připojeno teplotní čidlo DS18B20, které měří teplotu půdy (pro možnost regulace zavlažování dle teploty)<br>
K řídící desce je také připojeno čidlo vlhkosti s frekvenčním výstupem 50Hz-5kHz (pro možnost regulace zavlažování dle vlhkosti) https://pihrt.com/elektronika/272-snimac-vlhkosti-pro-zalevaci-automat<br>
Veškeré nastavení automatu se provádí pomocí tlačítek a zobrazení údajů na 16x2 LCD displeji.<br>
Automat byl vyvinutý pro vertikální zavlažování rostlin, které má svá specifika zavlažování.<br>

## Pro více informací navštivte web...
Martin Pihrt - www.pihrt.com: https://pihrt.com/elektronika/400-uno-florist-automata<br>


## Pohled na desku a připojení čidel
<a href="photos\fw1.00\foto (25).jpg"><img src="photos\fw1.00\foto (25).jpg" width="80%"></a>
<a href="photos\fw1.00\foto (29).jpg"><img src="photos\fw1.00\foto (29).jpg" width="80%"></a>

## Video (fw 1.00 beta)
[![Ukázka chodu](https://www.pihrt.com/images/videa/automata_1.00.mp4)](https://www.pihrt.com/images/videa/automata_1.00.mp4)

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
a) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost < 40% bude výsledná doba trvání závlahy + 1/2 trvání závlahy nastavené doby v menu. Tj. 150% času.<br>
b) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost >= 40% a < 60% bude výsledná doba trvání závlahy stejná jako nastavená doba v menu. Tj. 100% času.<br>
c) pokud je daný den a je čas programu 1 nebo je čas programu 2 a vlhkost >= 60% a < 80% bude výsledná doba trvání závlahy stejná jako nastavená doba v menu. Tj. 100% času.<br>
d) pokud je daný den a je čas programu 1 a vlhkost >= 60% bude výsledná doba trvání závlahy stejná jako nastavená doba v menu. Tj. 100% času.<br>
Z výše uvedeného vyplývá, že některé programy se s ohledem na vlhkost v určitém rozsahu vůbec nespustí, nebo bude výsledný čas upraven.<br>
<a href="photos\fw1.00\foto (17).png"><img src="photos\fw1.00\foto (17).png" width="50%"></a>
<b>V žádném případě nepoužívejte obě regulace současně!</b>

## Regulece dle teploty
a) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota > 40°C bude výsledná doba trvání závlahy + 1/2 trvání závlahy nastavené doby v menu. Tj. 150% času.<br>
b) pokud je daný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota >= 30°C a < 40°C bude výsledná doba trvání závlahy stejná jako nastavená doba v menu. Tj. 100% času.<br>
c) pokud je daný den a je čas programu 1 nebo je čas programu 2 a teplota >= 20°C a < 30°C bude výsledná doba trvání závlahy stejná jako nastavená doba v menu. Tj. 100% času.<br>
d) pokud je daný den a je čas programu 1 a teplota >= 5°C a < 20°C bude výsledná doba trvání závlahy stejná jako nastavená doba v menu. Tj. 100% času.<br>
Z výše uvedeného vyplývá, že některé programy se s ohledem na teplotu v určitém rozsahu vůbec nespustí, nebo bude výsledný čas upraven.<br>
<a href="photos\fw1.00\foto (16).png"><img src="photos\fw1.00\foto (16).png" width="50%"></a>
<b>V žádném případě nepoužívejte obě regulace současně!</b>

## Cyklování výstupu
V menu lze nastavit po jaké době běhu programu (1, 2, 3) se má výstup čerpadla vypnout a jako dobu má čekat vypnutý. Tato funkce umožňuje cyklování výstupu pro lepší vsakování vody do půdy (nedojde k přetékání květináče, ale k postupnému vsaku vody)<br>
<a href="photos\fw1.00\foto (13).png"><img src="photos\fw1.00\foto (13).png" width="50%"></a>
<a href="photos\fw1.00\foto (14).png"><img src="photos\fw1.00\foto (14).png" width="50%"></a>
Pokud nepožadujeme funkci cyklování nastavíme v menu "vypnout na dobu" na hodnotu 0s" a "vypnout čerpadlo po" na maximální hodnotu 250s.<br>
V grafu je vidět spínání výstupu (zap/vyp cyklování) s ohledem na vlhkost.<br>
<a href="photos\fw1.00\prubeh_menu_vystup.png"><img src="photos\fw1.00\prubeh_menu_vystup.png" width="50%"></a> 

## Nastavení datumu a času
Zleva nastavujeme den, měsíc, rok, hodina a minuta. Vteřina se při uložení nastaví na "0". Čas je ukládán do obvodu RTC DS1307.
<a href="photos\fw1.00\foto (18).png"><img src="photos\fw1.00\foto (18).png" width="50%"></a>

## Smazání do továrního nastavení
Pokud chceme zařízení vymazat do továrního nastavení stiskneme v menu jako níže na obrázku "ANO".<br>
<a href="photos\fw1.00\foto (19).png"><img src="photos\fw1.00\foto (19).png" width="50%"></a><br>
* Výchozí údaje po resetu:<br>
* program 1 čas pro zavlažování 6:00<br>
* doba 30s<br>
* program 2 čas pro zavlažování 18:00<br>
* doba 30s<br>
* program 3 čas pro zavlažování 9:00<br> 
* doba 30s<br>
* vypnout čerpadlo po 5s<br>
* vypnout na dobu 20s<br>
* nepoužívat regulaci teplotou<br>
* nepoužívat regulaci vhlkostí<br>
* zavlažovat ve dnech po-ne<br>

## Uložit nastavení
Všechny parametry se ukládají do trvalé paměti EEPROM.
<a href="photos\fw1.00\foto (20).png"><img src="photos\fw1.00\foto (20).png" width="50%"></a>
<a href="photos\fw1.00\foto (21).png"><img src="photos\fw1.00\foto (21).png" width="50%"></a>
<a href="photos\fw1.00\foto (22).png"><img src="photos\fw1.00\foto (22).png" width="50%"></a>

## Připojení vstupů a výstupů  
* LCD:                    pin 8, 9,  2,  3,  6,  7 (Rs, E,DB4,DB5,DB6,DB7)<br>
* LCD podsvícení:         pin 10<br>
* Tlačítka IN:            pin 14 (A0 dělič 5 tlačítek)<br>
* Čidlo vlhka IN:         pin 5 (měří se frekvence ze snímače vlhkosti)<br>
* Serial:                 pin 0, 1<br>
* I2C:                    pin 18 (A4 SDA), 19 (A5 SCL) používá DS1307 adr. 0x68<br>
* Teplota:                pin 16  DS18B20 rudá +5V, černá GND, modrá (bílá) Data<br>
* Čerpadlo:               pin 17 (pro výstup - čerpadlo)<br>

## Verze FW, HW
* 1.00 výchozí verze desky a programu (měření vlhkosti pomocí frekvence)
* 1.10 upravená verze HW (přidán konektor na vstup A1) a FW (možnost měření vlhkosti pomocí sondy s napěťovým výstupem) příklad použité sondy: https://www.diymore.cc/products/2pcs-capacitive-soil-moisture-sensor-v1-2-analog-corrosion-resistant-dc-3-3-5-5v)