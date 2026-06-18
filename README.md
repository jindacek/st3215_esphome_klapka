# st3215_esphome_klapka v0.2

ESPHome externí komponenta pro ST3215 jako klapku/výhybku.

## Změna v0.2

Kalibrace je zjednodušená pro klapku:

- tlačítko `start_calibration()` uloží aktuální polohu jako BOX1 / 0°
- druhá poloha se už nekalibruje
- poloha BOX2 se určuje přes `max_angle` a `move_to_angle(degrees)`

Příklad: `max_angle: 90`, `move_to_angle(62)` pošle servo na 62° od uložené nuly.
