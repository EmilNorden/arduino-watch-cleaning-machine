#include <Arduino.h>
#include <unordered_map>
#include <memory>

#define BIG_DIR     12
#define BIG_STEP    11
#define BIG_SLEEP   10
#define BIG_RST     9
#define BIG_MS3     8
#define BIG_MS2     7
#define BIG_MS1     6
#define BIG_ENABLE  5



const uint8_t MotorChannel  = 1;
const uint8_t MotorBits     = 8;

enum class MotorState {
  SpinUp,
  Spinning,
  SpinDown,
};

MotorState current_state;
unsigned long state_start_time = 0;

void transition_to_state(MotorState state);
void set_frequency(uint32_t desired_freq);

void setup() {
  Serial.begin(115200);
  pinMode(BIG_ENABLE, OUTPUT);
  pinMode(BIG_MS1, OUTPUT);
  pinMode(BIG_MS2, OUTPUT);
  pinMode(BIG_MS3, OUTPUT);
  pinMode(BIG_RST, OUTPUT);
  pinMode(BIG_SLEEP, OUTPUT);
  pinMode(BIG_STEP, OUTPUT);
  pinMode(BIG_DIR, OUTPUT);

  digitalWrite(BIG_STEP, LOW);
  digitalWrite(BIG_DIR, LOW);
  digitalWrite(BIG_MS1, LOW);
  digitalWrite(BIG_MS2, LOW);
  digitalWrite(BIG_MS3, LOW); // This wasnt here in the example

  digitalWrite(BIG_SLEEP, HIGH);
  digitalWrite(BIG_RST, HIGH);
  //digitalWrite(BIG_EN, HIGH);

  digitalWrite(BIG_ENABLE, LOW);
  digitalWrite(BIG_DIR, LOW);

  ledcAttachPin(13, MotorChannel);

  //ledcSetup(1, 1500, 8);
  ledcSetup(MotorChannel, 200, MotorBits);
  

  ledcWrite(MotorChannel, 127);
  delay(100);
  transition_to_state(MotorState::SpinUp);
}


const unsigned long OscillatePeriodMs = 5000;
const uint32_t FreqStepSize = 200;

uint32_t lerp(uint32_t a, uint32_t b, float factor) {
  return a * (1.0f - factor) + b * factor;
}

uint32_t clamp(uint32_t value, uint32_t min, uint32_t max) {
  if(value < min) {
    return min;
  } else if(value > max) {
    return max;
  }

  return value;
}

/*bool reverse = false;
bool disabled = false;
unsigned int current_index = 0;
unsigned int freqs[] = {
  0,
  180,
  400,
  800,
  1600,
  1600,
  1600,
  800,
  400,
  180,
};
void loop() {
  uint32_t foo = millis() - previous_time;
  if(foo > 30) {
    previous_time = millis();
    current_index = (current_index + 1) % (sizeof(freqs) / sizeof(unsigned int));

    int new_freq = freqs[current_index];
    if(new_freq == 0) {
      reverse = !reverse;
      Serial.println("DISABLING MOTOR?");
      digitalWrite(BIG_ENABLE, HIGH);

      if(reverse) {
        digitalWrite(BIG_DIR, HIGH);
      } else {
        digitalWrite(BIG_DIR, LOW);
      }
      return;
    }
    else {
      digitalWrite(BIG_ENABLE, LOW);
    }


    Serial.print("Current index is ");
    Serial.println(current_index);
    uint32_t result = ledcSetup(MotorChannel, freqs[current_index], MotorBits);
    Serial.print("Tried to set ");
    Serial.print(freqs[current_index]);
    Serial.print(". Got: ");
    Serial.println(result);
  }

}
*/

uint32_t current_freq = 0;
void set_frequency(uint32_t desired_freq) {
  if(desired_freq == current_freq) {
    return;
  }
  uint32_t result = ledcSetup(MotorChannel, desired_freq, MotorBits);
  Serial.print("Tried to set ");
  Serial.print(desired_freq);

  Serial.print(". Got: ");
  Serial.println(result);
  if(result != 0) {
    current_freq = desired_freq;
  }
}
const uint32_t SpinningDuration = 1000;


struct MotorSequencePoint {
  uint32_t frequency;
  unsigned long time_ms;
};

MotorSequencePoint ramping_sequence[] = {
  MotorSequencePoint { .frequency = 180, .time_ms = 0 },
  MotorSequencePoint { .frequency = 500, .time_ms = 20 },
  MotorSequencePoint { .frequency = 750, .time_ms = 40 },
  MotorSequencePoint { .frequency = 1000, .time_ms = 60 },
  MotorSequencePoint { .frequency = 1250, .time_ms = 80 },
  MotorSequencePoint { .frequency = 1500, .time_ms = 100 },
  MotorSequencePoint { .frequency = 1750, .time_ms = 120 },
  MotorSequencePoint { .frequency = 2000, .time_ms = 140 },
  MotorSequencePoint { .frequency = 2250, .time_ms = 160 },
  MotorSequencePoint { .frequency = 2500, .time_ms = 180 },
  MotorSequencePoint { .frequency = 2750, .time_ms = 180 },
};
const int ramping_sequence_length = sizeof(ramping_sequence) / sizeof(MotorSequencePoint);

void transition_to_state(MotorState state) {
  current_state = state;
  state_start_time = millis();
  switch(current_state) {
    case MotorState::SpinUp:
    digitalWrite(BIG_ENABLE, LOW);
    break;
    case MotorState::Spinning:
    set_frequency(ramping_sequence[ramping_sequence_length-1].frequency);
    break;
  }
}




bool reverse = false;
void change_direction() {
  reverse = !reverse;
  if(reverse) {
    digitalWrite(BIG_DIR, HIGH);
  } else {
    digitalWrite(BIG_DIR, LOW);
  }
}

void loop () {
  unsigned long state_elapsed = millis() - state_start_time;
  switch(current_state) {
    case MotorState::SpinUp:
    {
      for(int i = 1; i < ramping_sequence_length; ++i) {
        if(state_elapsed < ramping_sequence[i].time_ms) {
          set_frequency(ramping_sequence[i-1].frequency);
          break;
        } else if (i == ramping_sequence_length - 1) {
          set_frequency(ramping_sequence[i].frequency);
          transition_to_state(MotorState::Spinning);
        }
      }
    }
    break;
    case MotorState::SpinDown:
    {
      MotorSequencePoint last = ramping_sequence[ramping_sequence_length-1];
      for(int i = ramping_sequence_length - 2; i >= 0; --i) {
        if(state_elapsed < last.time_ms - ramping_sequence[i].time_ms) {
          set_frequency(ramping_sequence[i+1].frequency);
          break;
        } else if(i == 0) {
          change_direction();
          set_frequency(ramping_sequence[i].frequency);
          transition_to_state(MotorState::SpinUp);
        }
      } 
    }
    break;
    case MotorState::Spinning:
        if(state_elapsed >= SpinningDuration) {
          transition_to_state(MotorState::SpinDown);
        }
    break;
  }
}

