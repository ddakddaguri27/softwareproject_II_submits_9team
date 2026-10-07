// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 10     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 350     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)
                          // Setting EMA to 1 effectively disables EMA filter.

#define N 30               // amount of last samples

// global variables
unsigned long last_sampling_time;   // unit: msec
float dist_prev = _DIST_MAX;        // Distance last-measured
float dist_ema;                     // EMA distance

bool ema_initialized = false;       // when to calculate
float dist_samples[N];              // array of samples
int sample_count = 0;

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  pinMode(PIN_ECHO,INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float dist_raw, median;
  bool measurement_valid;
  
  // wait until next sampling time. 
  // millis() returns the number of milliseconds since the program started. 
  // will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL)
    return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);


  // Check whether mesurement is valid, and if it is valid, add to sample
  if ((dist_raw == 0.0) || (dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX)) {
    measurement_valid = false;
  }
  else {
    measurement_valid = true;
    addSample(dist_raw);
  }


  // Calculate median
  if (sample_count > 0) {
    median = getMedian();

    // Initialize EMA with the first median value
    if (!ema_initialized) {
      dist_ema = median;
      ema_initialized = true;
    }
    else {
      // EMA equation
      dist_ema = _EMA_ALPHA * median
                 + (1 - _EMA_ALPHA) * dist_ema;
    }
  }
  else {
    // No valid sample has been stored yet
    median = 0.0;
  }


  // output the distance to the serial port
  Serial.print("Min:");   Serial.print(_DIST_MIN);
  Serial.print(",raw:"); Serial.print(min(dist_raw, _DIST_MAX + 100));
  Serial.print(",ema:");  Serial.print(min(dist_ema, _DIST_MAX + 100));
  Serial.print(",median:");  Serial.print(min(median, _DIST_MAX + 100));
  Serial.print(",Max:");  Serial.print(_DIST_MAX);
  Serial.println("");

  // do something here
  if ((dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX))
    digitalWrite(PIN_LED, 1);       // LED OFF
  else
    digitalWrite(PIN_LED, 0);       // LED ON

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

// Add valid sample
void addSample(float value)
{
  // Array is not full
  if (sample_count < N) {

    dist_samples[sample_count] = value;
    sample_count++;

  }

  // Array is full
  else {

    // Shift all values to the left
    for (int i = 0; i < N - 1; i++) {
      dist_samples[i] = dist_samples[i + 1];
    }

    // Add the newest value at the end
    dist_samples[N - 1] = value;
  }
}


// Calculate median
float getMedian()
{
  if (sample_count == 0)
    return 0.0;

  // Copy samples so that original order is preserved
  float temp[N];

  for (int i = 0; i < sample_count; i++) {
    temp[i] = dist_samples[i];
  }

  // Bubble sort
  for (int i = 0; i < sample_count - 1; i++) {

    for (int j = 0; j < sample_count - 1 - i; j++) {

      if (temp[j] > temp[j + 1]) {

        float t = temp[j];
        temp[j] = temp[j + 1];
        temp[j + 1] = t;
      }
    }
  }

  // Odd number of samples
  if (sample_count % 2 == 1) {
    return temp[sample_count / 2];
  }

  // Even number of samples
  else {
    return (temp[sample_count / 2 - 1]
          + temp[sample_count / 2]) / 2.0;
  }
}
