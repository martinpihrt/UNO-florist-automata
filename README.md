UNO florist automata Readme
====

UNO florist automata - Zavlažovač květin založený na platformě Arduino a desce Arduino UNO (tedy procesoru ATMEGA328).

Regulace dle vlhkosti:
a) pokud je danný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost < 40 bude výsledná doba trvání závlahy 50% nastavené doby v menu
b) pokud je danný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a vlhkost >= 40 a < 60 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu
c) pokud je danný den a je čas programu 1 nebo je čas programu 2 a vlhkost >= 60 a < 80 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu
d) pokud je danný den a je čas programu 1 a vlhkost >= 60 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu             

Regulece dle teploty:
a) pokud je danný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota > 40 bude výsledná doba trvání závlahy 50% nastavené doby v menu
b) pokud je danný den a je čas programu 1 nebo je čas programu 2 nebo je čas programu 3 a teplota >= 30 a < 40 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu               
c) pokud je danný den a je čas programu 1 nebo je čas programu 2 a teplota >= 20 a < 30 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu
d) pokud je danný den a je čas programu 1 a teplota >= 5 a < 20 bude výsledná doba trvání závlahy stejná jako nastavená doba v menu

Cyklování výstupu (relé, čerpadlo...):
v menu lze nastavit po jaké době běhu programu (1, 2, 3) se má výstup čerpadla vypnout a jako dobu má čekat vypnutý. Tato funkce umožňuje cyklování výstupu pro lepší vsakování vody do půdy (nedojde k přetékání květináče, ale k postupnému vasku vody)

Ve výstavbě...

## Pro více informací navštivte web autora Martin Pihrt - www.pihrt.com

