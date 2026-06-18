# st3215_esphome_klapka

ESPHome externí komponenta pro klapku se servem ST3215.

Změny proti dveřní verzi:
- komponenta se jmenuje `st3215_servo_klapka`
- kalibrace dovolí malý rozsah pohybu klapky (`min_calib_span_turns`, default 0.02 ot.)
- procenta jsou normálně: 0 % = nulová/svislá poloha, 100 % = kalibrovaná vyklopená poloha
- přidána metoda `move_to_angle(deg)` pro lambda ovládání 0–`max_angle`
- `position_speed` je nastavitelný v YAML

