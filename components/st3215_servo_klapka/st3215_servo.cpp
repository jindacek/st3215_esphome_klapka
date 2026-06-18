#include "st3215_servo.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <cmath>

namespace esphome {
namespace st3215_servo_klapka {

static const char *const TAG = "st3215_servo_klapka";

// ===== PARAMETRY RAMPY =====
static constexpr int   SPEED_MAX     = 3000;
static constexpr int   SPEED_MIN     = 100;
static constexpr int   ACCEL_RATE    = 95;     // změna rychlosti za krok
static constexpr uint32_t RAMP_DT_MS = 30;     // perioda rampy
static constexpr float DECEL_ZONE    = 0.90f;  // kdy začít brzdit (otáčky před koncem)
static constexpr float STOP_EPS      = 0.01f;  // hystereze koncáku
// position_speed_ je nastavitelný z YAML

// ================= TORQUE SWITCH =================
void St3215TorqueSwitch::write_state(bool state) {
  if (parent_) parent_->set_torque_from_switch(state);
  publish_state(state);
}

// ================= AUTO UNLOCK SWITCH =================
void St3215AutoUnlockSwitch::write_state(bool state) {
  if (parent_) parent_->set_auto_unlock_from_switch(state);
  publish_state(state);
}

// ================= CALIB STATE =================
void St3215Servo::update_calib_state_(CalibState s) {
  calib_state_ = s;
  if (calib_state_sensor_) {
    calib_state_sensor_->publish_state((int) s);
  }
}

// ================= PERSISTENTNÍ ÚLOŽIŠTĚ =================
//
// Ukládáme:
//  - zero_offset_ (horní poloha)
//  - max_turns_   (plný rozsah)
//  - turns_unwrapped_ (aktuální absolutní poloha)
// Každé servo má svůj blok podle servo_id_.
//
bool St3215Servo::load_calibration_() {
  const uint32_t base = 0x1000u + static_cast<uint32_t>(servo_id_) * 3u;

  auto pref_zero = global_preferences->make_preference<float>(base + 0);
  auto pref_max  = global_preferences->make_preference<float>(base + 1);
  auto pref_pos  = global_preferences->make_preference<float>(base + 2);

  float z = 0.0f;
  float m = 0.0f;
  float p = 0.0f;

  if (!pref_zero.load(&z) || !pref_max.load(&m) || !pref_pos.load(&p)) {
    ESP_LOGI(TAG, "No stored calibration for servo %u", servo_id_);
    return false;
  }

  if (m < min_calib_span_turns_) {
    ESP_LOGW(TAG, "Stored calibration invalid (max_turns=%.3f) – ignoring", m);
    return false;
  }

  zero_offset_ = z;
  max_turns_   = m;
  has_zero_    = true;
  has_max_     = true;

  // uložená absolutní poloha
  float loaded = p;

  stored_turns_ = loaded;
  has_stored_turns_ = true;


  ESP_LOGI(TAG, "Loaded calibration from flash: zero=%.3f, max=%.3f, pos=%.3f",
           zero_offset_, max_turns_, stored_turns_);
  return true;
}

void St3215Servo::save_calibration_() {
  if (!has_zero_ || !has_max_) {
    ESP_LOGW(TAG, "Cannot save calibration – flags not set (has_zero=%d, has_max=%d)",
             has_zero_, has_max_);
    return;
  }

  const uint32_t base = 0x1000u + static_cast<uint32_t>(servo_id_) * 3u;

  auto pref_zero = global_preferences->make_preference<float>(base + 0);
  auto pref_max  = global_preferences->make_preference<float>(base + 1);
  auto pref_pos  = global_preferences->make_preference<float>(base + 2);

  pref_zero.save(&zero_offset_);
  pref_max.save(&max_turns_);

  float pos = turns_unwrapped_;

  pref_pos.save(&pos);


  ESP_LOGI(TAG, "Calibration+position saved to flash: zero=%.3f, max=%.3f, pos=%.3f",
           zero_offset_, max_turns_, pos);
}

// ================= CHECKSUM =================
uint8_t St3215Servo::checksum_(const uint8_t *data, size_t len) {
  uint16_t sum = 0;
  for (size_t i = 2; i < len; i++) sum += data[i];
  return (~sum) & 0xFF;
}

// ================= SEND PACKET =================
void St3215Servo::send_packet_(uint8_t id, uint8_t cmd,
                               const std::vector<uint8_t> &params) {
  std::vector<uint8_t> p;
  p.reserve(6 + params.size());

  p.push_back(0xFF);
  p.push_back(0xFF);
  p.push_back(id);
  p.push_back(params.size() + 2);
  p.push_back(cmd);
  for (auto b : params) p.push_back(b);
  p.push_back(checksum_(p.data(), p.size()));

  write_array(p);
  flush();
}

// ================= READ REGISTERS =================
bool St3215Servo::read_registers_(uint8_t id, uint8_t addr, uint8_t len,
                                  std::vector<uint8_t> &out) {
  // Nečistíme agresivně RX buffer, jen pošleme dotaz
  send_packet_(id, 0x02, {addr, len});

  uint32_t start = millis();
  std::vector<uint8_t> buf;
  buf.reserve(32);

  while (millis() - start < 150) {

    while (available()) {
      buf.push_back(read());
      if (buf.size() > 64)
        buf.erase(buf.begin(), buf.begin() + (buf.size() - 64));
    }

    while (buf.size() >= 2 && !(buf[0] == 0xFF && buf[1] == 0xFF))
      buf.erase(buf.begin());

    if (buf.size() < 6) continue;

    uint8_t rid = buf[2];
    uint8_t rlen = buf[3];

    if (rlen < 2 || rlen > 20) {
      buf.erase(buf.begin());
      continue;
    }

    size_t full_len = rlen + 4;
    if (buf.size() < full_len) continue;

    uint8_t err = buf[4];
    if (rid != id || err != 0) {
      buf.erase(buf.begin(), buf.begin() + full_len);
      continue;
    }

    uint8_t chk = buf[full_len - 1];
    uint8_t calc = checksum_(buf.data(), full_len - 1);
    if (chk != calc) {
      buf.erase(buf.begin(), buf.begin() + full_len);
      continue;
    }

    uint8_t data_len = rlen - 2;
    if (data_len < len) {
      buf.erase(buf.begin(), buf.begin() + full_len);
      continue;
    }

    out.assign(buf.begin() + 5, buf.begin() + 5 + len);
    return true;
  }
  return false;
}

// ================= STATE SENSOR =================
void St3215Servo::publish_state_(const std::string &s) {
  if (state_sensor_ == nullptr) return;
  if (s == last_state_) return;
  last_state_ = s;
  state_sensor_->publish_state(s);
}

// ================= SETUP =================
void St3215Servo::setup() {
  ESP_LOGI(TAG, "ST3215 init ID=%u invert=%d", servo_id_, invert_direction_);

  // Nastavení motoru do "motor mode"
  // (původně byl posílán napevno paket s checksumem spočítaným jen pro ID=1)
  send_packet_(servo_id_, 0x03, {0x21, 0x01});

  // Zapnutí krouticího momentu (torque ON)
  send_packet_(servo_id_, 0x03, {0x28, 0x01});

  torque_on_ = false;

  // Zkusíme načíst kalibraci + polohu z flash
  bool loaded = this->load_calibration_();

  // Klapkový režim: pokud není uložený rozsah, vezmeme ho z max_angle_.
  // Např. max_angle 90° = 0.25 otáčky.
  if (!has_max_) {
    max_turns_ = max_angle_ / 360.0f;
    if (max_turns_ < min_calib_span_turns_)
      max_turns_ = min_calib_span_turns_;
    has_max_ = true;
  }

  // Zkusíme načíst ramp_factor_ z flash (samostatný blok per-servo)
  // {
    // const uint32_t rbase = 0x2000u + static_cast<uint32_t>(servo_id_) * 10u;
    // auto pref_ramp = global_preferences->make_preference<float>(rbase);
    // float rf = 0.0f;
    // if (pref_ramp.load(&rf)) {
      // ramp_factor_ = rf;
      // ESP_LOGI(TAG, "Ramp factor loaded from flash: %.2f", ramp_factor_);
    // } else {
      // ESP_LOGI(TAG, "No stored ramp factor, using default %.2f", ramp_factor_);
    // }
  // }
  
  // ===== INIT STAVU KALIBRACE =====
  if (!has_zero_) {
    update_calib_state_(CALIB_IDLE);
    ESP_LOGI(TAG, loaded
                 ? "Stored calibration incomplete – nutná kalibrace BOX1"
                 : "Nutná kalibrace BOX1");
  } else {
    update_calib_state_(CALIB_DONE);
    ESP_LOGI(TAG, "Klapka připravena (zero=%.3f, max_turns=%.3f, max_angle=%.1f°)",
             zero_offset_, max_turns_, max_angle_);
  }
}

// ================= CONFIG =================
void St3215Servo::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 Servo ID=%u invert=%d", servo_id_, invert_direction_);
}

// ================= UPDATE =================
void St3215Servo::update() {
  // ===== UART/ENCODER FAULT RECOVERY =====
  uint32_t &last_uart_recovery = last_uart_recovery_;

  if (encoder_fault_) {
    // Zkusíme jednou za 5 s obnovit komunikaci na sběrnici
    uint32_t now = millis();
    if (now - last_uart_recovery >= 5000) {
      ESP_LOGW(TAG, "Trying UART recovery after encoder fault");

      // Vyprázdnit TX, RX buffer
      flush();
      delay(10);
      while (available()) {
        (void) read();
      }

      encoder_fail_count_ = 0;
      encoder_fault_ = false;
      last_uart_recovery = now;
    } else {
      // Zatím jen čekáme na další pokus o recovery
      return;
    }
  }

  std::vector<uint8_t> pos;
  if (!read_registers_(servo_id_, 0x38, 2, pos)) {
    encoder_fail_count_++;

    if (encoder_fail_count_ >= ENCODER_FAIL_LIMIT && !encoder_fault_) {
      ESP_LOGE(TAG, "ENCODER FAULT → EMERGENCY STOP");
      stop();                // okamžitě zastavit
      encoder_fault_ = true; // zbytek logiky přerušíme, dokud se neprovede recovery
    }
    return;
  }

  // pokud se čtení povedlo → reset chyb
  encoder_fail_count_ = 0;

  uint16_t raw = pos[0] | (pos[1] << 8);
  if (raw >= 4096 || raw == 0xFFFF)
    return;

  // ===== VIRTUÁLNÍ ENKODÉR =====
  // vše dál pracuje už jen s logical_raw
  uint16_t logical_raw = invert_direction_ ? (4095 - raw) : raw;


  // První krok po startu – navázání na uloženou polohu
  if (!have_last_) {
    last_raw_ = logical_raw;

    if (has_stored_turns_) {
      // uložená absolutní poloha = turns_base + (raw / RAW_PER_TURN)
      float frac = logical_raw / RAW_PER_TURN;
      // turns_base_ chceme jako integer tak, aby:
      //  stored_turns_ ≈ turns_base_ + frac
      turns_base_ = static_cast<int32_t>(std::round(stored_turns_ - frac));
      turns_unwrapped_ = stored_turns_;
      ESP_LOGI(TAG, "Using stored position: turns_unwrapped=%.3f, base=%ld, raw=%u",
               turns_unwrapped_, (long) turns_base_, raw);
    } else {
      turns_base_ = 0;
      turns_unwrapped_ = logical_raw / RAW_PER_TURN;
      ESP_LOGI(TAG, "No stored position, starting from raw=%.3f turns",
               turns_unwrapped_);
    }

    have_last_ = true;
    return;
  }

  int diff = (int) logical_raw - (int) last_raw_;

  // invertace enkodéru pro otočené servo
  //if (invert_direction_) diff = -diff;

  if (abs(diff) > 3800)
    return;

  if (diff > 2048)
    turns_base_--;
  else if (diff < -2048)
    turns_base_++;


  last_raw_ = logical_raw;
  float frac = logical_raw / RAW_PER_TURN;
  turns_unwrapped_ = turns_base_ + frac;


  float angle = (logical_raw / RAW_PER_TURN) * 360.0f;
  float total = fabsf(turns_unwrapped_ - zero_offset_);

  if (angle_sensor_) angle_sensor_->publish_state(angle);
  if (turns_sensor_) turns_sensor_->publish_state(total);
  if (percent_sensor_ && has_zero_ && has_max_) {
    float pct = (total / max_turns_) * 100.0f;

    // fyzikální clamp
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    // kosmetika pro HA – žádné 1 % / 99 %
    if (pct < 2.0f) pct = 0.0f;
    if (pct > 98.0f) pct = 100.0f;

    percent_sensor_->publish_state(pct);

    // ===== AUTO STOP NA CÍLI (POSITION MODE) =====
    if (position_mode_) {
      if (fabs(pct - target_percent_) < 1.5f) {
        ESP_LOGI(TAG, "TARGET REACHED %.1f %%", pct);
        stop();
        position_mode_ = false;
      }
    }
  }

  // ===== SAFETY: ČASOVÝ LIMIT + DETEKCE ZASEKNUTÍ =====
  // Funguje jen mimo kalibraci. Při kalibraci se s dveřmi záměrně najíždí ručně na dorazy.
  if (moving_ && !calibration_active_) {
    uint32_t now_safety = millis();

    // Kdyby pohyb začal dřív než první platné čtení enkodéru, inicializujeme monitor tady.
    if (movement_start_ms_ == 0) {
      reset_safety_monitor_();
    }

    // 1) Tvrdý maximální čas jedné jízdy
    if (max_run_time_ms_ > 0 && (now_safety - movement_start_ms_) > max_run_time_ms_) {
      safety_stop_("Chyba: časový limit pohybu");
      return;
    }

    // 2) Stall ochrana: servo má jet, ale enkodér se skoro nehýbe
    float moved = fabsf(turns_unwrapped_ - stall_last_turns_);
    if (moved >= stall_min_delta_turns_) {
      stall_last_turns_ = turns_unwrapped_;
      stall_last_move_ms_ = now_safety;
    } else if ((now_safety - movement_start_ms_) > stall_grace_ms_ &&
               (now_safety - stall_last_move_ms_) > stall_timeout_ms_) {
      safety_stop_("Chyba: zaseknutá klapka");
      return;
    }
  }

  // ===== SAVE UPDATED RAMP FACTOR =====
  // if (pending_ramp_save_) {
      // const uint32_t rbase = 0x2000u + static_cast<uint32_t>(servo_id_) * 10u;
      // auto pref_ramp = global_preferences->make_preference<float>(rbase);
      // pref_ramp.save(&ramp_factor_);
      // ESP_LOGI(TAG, "Ramp factor saved to flash: %.2f", ramp_factor_);
      // pending_ramp_save_ = false;
  // }
  
  // ===== RAMP ENGINE =====
  uint32_t now = millis();
  if (now - last_ramp_update_ >= RAMP_DT_MS) {
    last_ramp_update_ = now;
  
    // statická proměnná pro prediktivní brzdění (pamatuje si předchozí dist)
    float &last_dist = ramp_last_dist_;
    int   &last_sent_speed = ramp_last_sent_speed_;
    bool  &last_sent_cw = ramp_last_sent_cw_;
  
    // vypočítat vzdálenost ke konci
    float dist = 0.0f;
    if (has_zero_ && has_max_) {
      // dolů (CW) → směrem k max_turns_
      // nahoru (CCW) → směrem k nule
      dist = moving_cw_
          ? fabsf(max_turns_ - total)   // spodní konec
          : total;                      // horní konec
    }
  
    // rychlost přibližování (kladná, když se blížíme k dorazu)
    float dist_delta = last_dist - dist;
    last_dist = dist;
  
    // základní cílová rychlost vychází z uživatelem chtěné rychlosti
    int effective = target_speed_;
  
    if (has_zero_ && has_max_) {
      // 1) Základní brzdění podle vzdálenosti (S-křivka)
      float decel_zone = DECEL_ZONE * ramp_factor_;
    
      if (dist < decel_zone) {
        float k = dist / decel_zone;  // ← používáme zvětšenou/zmenšenou zónu
        if (k < 0) k = 0;
        if (k > 1) k = 1;
    
        float smooth = k * k * k;  // hezky měkké brzdění
        effective = SPEED_MIN + (int)((target_speed_ - SPEED_MIN) * smooth);
      }
    
      // 2) Prediktivní brzdění – když se blížíme moc rychle, uber ještě víc
      float predictive_brake = dist_delta * (1.4f * ramp_factor_);  // koeficient pro doladění
    
      if (predictive_brake > 0.01f) {
        effective -= (int)(predictive_brake * 800.0f * (ramp_factor_ * ramp_factor_));
        if (effective < SPEED_MIN)
          effective = SPEED_MIN;
      }
    }
  
    // === DEBUG RAMPA ===
    // ESP_LOGD(TAG, "[RAMP] dist=%.3f delta=%.4f eff=%d cur=%d moving=%d ",
             // dist, dist_delta, effective, current_speed_, moving_ ? 1 : 0);
  
    // Aplikace rampy (akcelerace/decelerace rychlosti)
    if (current_speed_ < effective) {
      current_speed_ += ACCEL_RATE;
      if (current_speed_ > effective) current_speed_ = effective;
    } else if (current_speed_ > effective) {
      current_speed_ -= ACCEL_RATE;
      if (current_speed_ < effective) current_speed_ = effective;
    }
  
    // Odeslání nové rychlosti do serva
    if (moving_ && current_speed_ >= SPEED_MIN) {
      // logický směr (CW = dolů), fyzický směr = případně invertovaný
      bool phys_cw = invert_direction_ ? !moving_cw_ : moving_cw_;
  
      // === DEBUG SMĚRU A SPEED ===
      // ESP_LOGD(TAG, "[SEND] speed=%d phys_cw=%d", current_speed_, phys_cw ? 1 : 0);
  
      // pošleme nový paket jen pokud se rychlost nebo fyzický směr změnily
      if (current_speed_ != last_sent_speed || phys_cw != last_sent_cw) {
        uint8_t lo = current_speed_ & 0xFF;
        uint8_t hi = (current_speed_ >> 8) & 0x7F;
        if (!phys_cw) hi |= 0x80;  // bit směru
  
        std::vector<uint8_t> p = {0x2E, lo, hi};
        send_packet_(servo_id_, 0x03, p);
  
        last_sent_speed = current_speed_;
        last_sent_cw    = phys_cw;
      }
    } else {
      // když se nemá hýbat, další pohyb začne "od nuly"
      last_sent_speed = -1;
    }
  }

  // ===== SOFT KONCÁKY =====
  // Pozn.: během kalibrace jsou vypnuté, aby neblokovaly dojetí na dorazy
  if (moving_ && !calibration_active_) {

    // HORNÍ KONCÁK – 100 % (nahoru = CCW = moving_cw_ == false)
    if (has_zero_ && total <= STOP_EPS && !moving_cw_) {
      // tvrdý sync na horní doraz
      turns_unwrapped_ = zero_offset_;

      // přepočet base tak, aby platilo:
      // turns_unwrapped_ ≈ turns_base_ + raw/RAW_PER_TURN
      float frac = logical_raw / RAW_PER_TURN;
      turns_base_ = static_cast<int32_t>(std::round(turns_unwrapped_ - frac));

      ESP_LOGI(TAG, "SW KONCÁK: 100 %% – STOP (sync pos=%.3f, base=%ld)",
               turns_unwrapped_, (long) turns_base_);
      
      last_raw_ = logical_raw;
      stop();
    }

    // DOLNÍ KONCÁK – 0 % (dolů = CW = moving_cw_ == true)
    if (has_max_ && total >= (max_turns_ - STOP_EPS) && moving_cw_) {
      // tvrdý sync na spodní doraz
      turns_unwrapped_ = zero_offset_ + max_turns_;

      float frac = logical_raw / RAW_PER_TURN;
      turns_base_ = static_cast<int32_t>(std::round(turns_unwrapped_ - frac));

      ESP_LOGI(TAG, "SW KONCÁK: 0 %% – STOP (sync pos=%.3f, base=%ld)",
               turns_unwrapped_, (long) turns_base_);

      last_raw_ = logical_raw;      
      stop();
    }
  }
}

// ================= SAFETY HELPERS =================
void St3215Servo::reset_safety_monitor_() {
  uint32_t now = millis();
  movement_start_ms_ = now;
  stall_last_move_ms_ = now;
  stall_last_turns_ = turns_unwrapped_;
}

void St3215Servo::safety_stop_(const std::string &reason) {
  ESP_LOGE(TAG, "SAFETY STOP: %s", reason.c_str());

  // Bezpečnost má přednost i před ručním držením torque switchem.
  manual_torque_override_ = false;

  // Nejdřív zastavit pohyb, potom vypnout torque.
  stop();
  set_torque(false);

  publish_state_(reason);
}

// ================= MOVE TO PORCENT =================
void St3215Servo::move_to_percent(float pct) {
  if (!has_zero_ || !has_max_)
    return;

  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;

  target_percent_ = pct;
  position_mode_ = true;

  float current = (percent_sensor_) ? percent_sensor_->state : 0.0f;

  if (fabs(current - pct) < 1.0f) {
    stop();
    position_mode_ = false;
    return;
  }

  if (pct > current) {
    // otevřít (CCW) – NAHORU
    rotate(true, position_speed_);
  } else {
    // zavřít (CW) – DOLŮ
    rotate(false, position_speed_);
  }
}

void St3215Servo::move_to_angle(float degrees) {
  if (max_angle_ <= 0.0f) max_angle_ = 90.0f;
  if (degrees < 0.0f) degrees = 0.0f;
  if (degrees > max_angle_) degrees = max_angle_;
  float pct = (degrees / max_angle_) * 100.0f;
  move_to_percent(pct);
}

// ================= TORQUE =================
void St3215Servo::set_torque(bool on) {
  // Původně dva napevno zadrátované pakety s checksumem jen pro ID=1:
  // const uint8_t torque_on[]  = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x01,0xCE};
  // const uint8_t torque_off[] = {0xFF,0xFF,servo_id_,0x04,0x03,0x28,0x00,0xCF};
  // if (on) write_array(torque_on, sizeof(torque_on));
  // else write_array(torque_off, sizeof(torque_off));
  // flush();

  // Nově používáme send_packet_, který checksum dopočítá správně pro jakékoli servo_id.
  send_packet_(servo_id_, 0x03, {0x28, static_cast<uint8_t>(on ? 0x01 : 0x00)});

  torque_on_ = on;

  if (torque_switch_)
    torque_switch_->publish_state(on);

  // ⬅️ INVERTOVANÝ STAV PRO ZÁMEK  
  if (torque_state_sensor_)
    torque_state_sensor_->publish_state(!on);
}

void St3215Servo::set_torque_from_switch(bool on) {
  // uživatel ručně přepnul torque
  manual_torque_override_ = on;
  set_torque(on);
}

void St3215Servo::set_auto_unlock_from_switch(bool on) {
  auto_unlock_ = on;
  // jen uložíme stav; nic dalšího neděláme
}

// ================= STOP =================
void St3215Servo::stop() {
  target_speed_ = 0;
  current_speed_ = 0;
  position_mode_ = false;

  // Původní napevno zadrátovaný paket s checksumem pro ID=1:
  // const uint8_t stop_cmd[] = {0xFF,0xFF,servo_id_,0x0A,0x03,0x2A,0x32,0x00,0x00,0x03,0x00,0x00,0x00,0x92};
  // write_array(stop_cmd, sizeof(stop_cmd));
  // flush();

  // Nově generujeme paket přes send_packet_ – checksum se spočítá správně pro libovolné servo_id.
  std::vector<uint8_t> params = {
    0x2A, 0x32,
    0x00, 0x00,
    0x03,
    0x00, 0x00, 0x00
  };
  send_packet_(servo_id_, 0x03, params);

  moving_ = false;
  movement_start_ms_ = 0;
  stall_last_move_ms_ = 0;
  publish_state_("Stojí");
  if (open_switch_)  open_switch_->publish_state(false);
  if (close_switch_) close_switch_->publish_state(false);

  // máme kalibraci → uložíme i aktuální polohu
  if (has_zero_ && has_max_) {
    save_calibration_();
  }

  // Auto odblokování: po STOP vypnout torque
  if (auto_unlock_ && !manual_torque_override_) {
    set_torque(false);
  }

  // Uložíme i aktuální ramp_factor_ (nezávisle na kalibraci)
  // {
    // const uint32_t rbase = 0x2000u + static_cast<uint32_t>(servo_id_) * 10u;
    // auto pref_ramp = global_preferences->make_preference<float>(rbase);
    // pref_ramp.save(&ramp_factor_);
    // ESP_LOGI(TAG, "Ramp factor saved to flash: %.2f", ramp_factor_);
  // }
}

// ================= ROTATE =================
void St3215Servo::rotate(bool cw, int speed) {
  // při pohybu vždy zapnout torque (pokud uživatel torque nevypnul ručně, tak to není relevantní;
  // ruční override je jen pro "drž ON", ne pro zákaz zapnutí při jízdě)
  set_torque(true);

  // cw = logický směr DOLŮ (bez ohledu na mechaniku / invert_direction)
  moving_ = true;
  moving_cw_ = cw;
  publish_state_(moving_cw_ ? "Jede na BOX2" : "Jede na BOX1");

  if (speed < 0) speed = -speed;
  if (speed > SPEED_MAX) speed = SPEED_MAX;
  target_speed_ = speed;

  // Start bezpečnostního monitoru pro tuto jízdu
  reset_safety_monitor_();
}


// ================= CALIBRATION =================
// Klapková kalibrace v0.2:
// - žádné dvě krajní polohy jako roleta/dveře
// - aktuální poloha se uloží jako BOX1 / 0°
// - rozsah pro BOX2 se bere z max_angle_ (např. 90° = 0.25 otáčky)
void St3215Servo::start_calibration() {
  ESP_LOGI(TAG, "=== KALIBRACE KLAPKY: aktuální poloha = BOX1 / 0° ===");
  set_zero();
}

void St3215Servo::confirm_calibration_step() {
  // Zachováno jen kvůli kompatibilitě se starými YAML tlačítky.
  // U klapky není druhý krok kalibrace potřeba.
  ESP_LOGI(TAG, "Klapka: confirm_calibration_step() není potřeba – kalibrace má jen jeden krok.");
  if (!has_zero_) set_zero();
}

void St3215Servo::set_zero() {
  stop();
  delay(100);

  zero_offset_ = turns_unwrapped_;
  max_turns_ = max_angle_ / 360.0f;
  if (max_turns_ < min_calib_span_turns_)
    max_turns_ = min_calib_span_turns_;

  has_zero_ = true;
  has_max_ = true;
  calibration_active_ = false;
  position_mode_ = false;

  update_calib_state_(CALIB_DONE);
  save_calibration_();

  if (percent_sensor_) percent_sensor_->publish_state(0.0f);

  ESP_LOGI(TAG, "Klapka BOX1/0° uloženo: zero=%.3f, max_turns=%.3f, max_angle=%.1f°",
           zero_offset_, max_turns_, max_angle_);
}

void St3215Servo::set_max() {
  // U klapky nepoužíváme ruční druhou krajní polohu.
  // Rozsah je definovaný přes max_angle_.
  max_turns_ = max_angle_ / 360.0f;
  if (max_turns_ < min_calib_span_turns_)
    max_turns_ = min_calib_span_turns_;
  has_max_ = true;
  if (has_zero_) save_calibration_();
}

// ================= SWITCH =================
void St3215Servo::set_torque_switch(St3215TorqueSwitch *s) {
  torque_switch_ = s;
  torque_switch_->set_parent(this);
  torque_switch_->publish_state(torque_on_);
}

void St3215Servo::set_auto_unlock_switch(St3215AutoUnlockSwitch *s) {
  auto_unlock_switch_ = s;
  auto_unlock_switch_->set_parent(this);
  auto_unlock_switch_->publish_state(auto_unlock_);
}
  
}  // namespace st3215_servo_klapka
}  // namespace esphome
