# st3215_esphome_klapka v0.7

Změny:
- Safety stop už neukládá aktuální pozici do flash, aby se po chybě nezapsala falešná poloha 8–10 %.
- Doporučeno v YAML nastavit `preferences: flash_write_interval: 5s` a po kalibraci počkat na zápis flash.
