# st3215_esphome_klapka v0.3

Komponenta pro klapku/výhybku se servem ST3215.

Změna v0.3:
- `move_to_angle()` už nejde přes procenta.
- BOX2 cíl se počítá přímo: `target_turns = angle / 360`.
- V logu je `MOVE ANGLE: requested=..., target_turns=...`.
- Kalibrace je pouze uložení aktuální polohy jako BOX1 / 0°.
