# st3215_esphome_klapka v0.4

Změny proti v0.3:

- `move_to_angle()` zůstává čistě úhlové: BOX1 nula + úhel/360.
- Rychlost z HA nyní nastavuje i `position_speed`, takže platí pro BOX1/BOX2 a cover.
- Rampová křivka je upravená pro krátký rozsah klapky. Původní brzdná zóna byla roletková a pro 0.1–0.25 otáčky byla moc velká.

Doporučení testu:

1. Nastavit `Klapka rychlost` třeba 300, 700, 1500 a zkusit BOX1/BOX2.
2. Nastavit `Klapka rampová křivka` 0.6, 1.4, 2.5 a sledovat dojezd.
3. Pokud bude dojezd moc ostrý, zvýšit rampu. Pokud moc líný, snížit rampu.
