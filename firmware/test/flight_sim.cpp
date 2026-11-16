// flight_sim.cpp
// Laptop test for the flight logic: simulates a whole rocket flight and checks that the chute fires at the right time and never at the wrong time.
// It builds a fake rocket (thrust, drag, chute), fake sensors with noise and failures, and feeds them to the REAL filter.cpp and flight.cpp.
//
// How to run it (from the firmware folder):
//   g++ -std=c++17 -O2 -Wall -Wextra -I test/stubs -I src test/flight_sim.cpp src/flight.cpp src/filter.cpp src/logging.cpp -o sim
//   ./sim              runs every scenario and the random tests
//   ./sim nominal.csv  also saves the nominal flight in the same format as the SD card log

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <random>
#include <string>
#include "filter.h"
#include "flight.h"
#include "logging.h"

// ---------------------------------------------------------------
// WHAT TO SIMULATE
// ---------------------------------------------------------------
struct Scenario {
  const char *name = "nominal";
  unsigned seed = 1;

  // Barometer noise (std dev in meters), chance per sample of a big outlier spike, and when it dies (-1 = never)
  double baroSigma = 0.5;
  double baroOutlierProb = 0.0;
  double baroDeadAtS = -1.0;

  // Accelerometer: noise, constant bias, how far the rocket leans, and whether the IMU is dead (reads zero)
  double accelSigma = 0.3;
  double accelBias = 0.15;
  double tiltDeg = 8.0;

  // When the IMU dies and reads zero from then on (-1 = never, 0 = dead from the start)
  double imuDeadAtS = -1.0;

  // Pad events: when the arm switch goes on (-1 = never), when it goes off again in flight, when ignition happens
  double armOnAtS = 2.0;
  double armOffAtS = -1.0;
  double ignitionAtS = 5.0;

  // A hard bump on the pad (5 g for 100 ms) with no launch, and a loop stall (like a slow SD write)
  double shockAtS = -1.0;
  double stallAtS = -1.0;
  double stallMs = 0.0;

  // Skip the rocket entirely: it just sits on the pad the whole time
  bool noLaunch = false;

  double maxTimeS = 90.0;
};

struct Result {
  bool fired = false;
  double fireT = 0, fireAlt = 0, fireVel = 0;
  int fireState = -1;
  uint8_t events = 0;
  double apogeeT = 0, apogeeAlt = 0;
  double impactSpeed = 0;
  bool landedDetected = false;
  bool firedOnPad = false;
  bool finalStatePad = false;
  double kalmanApogeeErrS = 999;
  double maxFiltErrM = 0;
};

// ---------------------------------------------------------------
// THE ROCKET (a made-up G80 flight, close to the OpenRocket numbers)
// ---------------------------------------------------------------
static double thrust(double tl) {
  if (tl < 0 || tl > 1.2) return 0;
  if (tl < 0.15) return 110;
  if (tl < 1.0) return 85;
  return 85 * (1.2 - tl) / 0.2;
}

static double massKg(double tl) {
  double burned = tl < 0 ? 0 : (tl > 1.2 ? 1.0 : tl / 1.2);
  return 0.696 - 0.062 * burned;
}

static Result runFlight(const Scenario &sc, const char *csvPath = nullptr) {
  std::mt19937 rng(sc.seed);
  std::normal_distribution<double> gauss(0.0, 1.0);
  std::uniform_real_distribution<double> uni(0.0, 1.0);

  FlightKalmanFilter filter;
  FlightComputer flight;
  Result res;

  FILE *csv = nullptr;
  if (csvPath) {
    csv = fopen(csvPath, "w");
    fprintf(csv, "t_ms,state,pyro,arm,ax,ay,az,gx,gy,gz,hx,hy,hz,press,temp,baro_alt,baro_ok,gps_fix,lat,lon,gps_alt,sats,filt_alt,filt_vel,vert_accel,events\n");
  }

  // True state of the rocket
  double z = 0, v = 0, zPrev = 0;
  double tFire = -1;
  double maxZ = 0;
  bool apogeeSeen = false;
  double lastBaro = 0;
  bool armPrev = false;

  const double dtLoop = 0.010;
  const double g = 9.81;
  double t = 0;
  uint32_t lastMs = 0;
  double lastT = 0;
  double tilt = sc.tiltDeg * M_PI / 180.0;
  double kalmanApogeeT = -1;

  while (t < sc.maxTimeS) {

    // ---- advance the physics one loop (10 sub-steps of 1 ms) ----
    double stepTime = dtLoop;
    bool stalled = (sc.stallAtS >= 0 && t >= sc.stallAtS && t < sc.stallAtS + dtLoop + 1e-9 && sc.stallMs > 0);
    if (stalled) stepTime = sc.stallMs / 1000.0;

    int sub = (int)std::round(stepTime / 0.001);
    double f = g;   // specific force along the rocket axis, m/s^2
    for (int i = 0; i < sub; i++) {
      double tl = sc.noLaunch ? -1.0 : (t + i * 0.001 - sc.ignitionAtS);
      double m = massKg(tl);
      double rho = 1.225 * std::exp(-z / 8500.0);

      // Body drag, then the chute opening up after the fire signal (0.15 s delay, 0.5 s to open)
      double cda = 0.55 * 0.00342;
      if (tFire >= 0) {
        double open = (t + i * 0.001 - (tFire + 0.15)) / 0.5;
        open = open < 0 ? 0 : (open > 1 ? 1 : open);
        cda = cda + open * (0.75 * 0.292 - cda);
      }
      double drag = -0.5 * rho * cda * v * std::fabs(v);
      double Tn = thrust(tl);
      double fnet = Tn + drag - m * g;

      if (z <= 0.0 && v <= 0.0 && fnet < 0) {
        z = 0; v = 0; f = g;
      } else {
        double a = fnet / m;
        v += a * 0.001;
        z += v * 0.001;
        f = (Tn + drag) / m;
        if (z < 0) {
          if (v < -0.5) res.impactSpeed = -v;
          z = 0; v = 0;
        }
      }
      if (z > maxZ) maxZ = z;
      if (!apogeeSeen && maxZ > 30 && v <= 0 && t > sc.ignitionAtS + 1.5) {
        apogeeSeen = true;
        res.apogeeT = t + i * 0.001;
        res.apogeeAlt = z;
      }
    }
    t += stepTime;

    // ---- fake sensors ----
    SensorData s;
    double reading = f * std::cos(tilt) + sc.accelBias + sc.accelSigma * gauss(rng);
    double lateral = f * std::sin(tilt) + sc.accelSigma * gauss(rng);
    if (sc.shockAtS >= 0 && t >= sc.shockAtS && t < sc.shockAtS + 0.1) reading += 5.0 * g;
    if (sc.imuDeadAtS >= 0 && t >= sc.imuDeadAtS) { reading = 0; lateral = 0; }
    double clipLimit = 16.0 * g;
    if (reading > clipLimit) reading = clipLimit;
    if (reading < -clipLimit) reading = -clipLimit;
    s.az = (float)reading;
    s.ax = (float)lateral;
    s.ay = (float)(sc.accelSigma * gauss(rng));
    s.hz = s.az;

    // Baro reads the altitude from one loop ago (the real driver returns the previous measurement)
    double baro = zPrev + sc.baroSigma * gauss(rng);
    if (sc.baroOutlierProb > 0 && uni(rng) < sc.baroOutlierProb) baro += (uni(rng) < 0.5 ? -1 : 1) * (10 + 10 * uni(rng));
    zPrev = z;
    bool baroOk = !(sc.baroDeadAtS >= 0 && t >= sc.baroDeadAtS);
    s.baroValid = baroOk;
    s.baroAlt = baroOk ? (float)baro : (float)lastBaro;
    s.pressure = (float)(101325.0 * std::pow(1.0 - s.baroAlt / 44330.0, 5.255));
    if (baroOk) lastBaro = baro;

    // ---- this is the same logic main.cpp runs each loop ----
    uint32_t nowMs = (uint32_t)std::llround(t * 1000.0);
    float dt = (float)((nowMs - lastMs) / 1000.0);
    if (lastMs == 0) dt = 0.01f;
    lastMs = nowMs;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;

    float vertAccel = flightVerticalAccel(s.az);
    float baroForFilter = s.baroValid ? s.baroAlt : (float)lastBaro;
    filter.update(vertAccel, baroForFilter, dt);

    bool armOn = (sc.armOnAtS >= 0 && t >= sc.armOnAtS) && !(sc.armOffAtS >= 0 && t >= sc.armOffAtS);
    if (armOn && !armPrev && flight.getState() == STATE_PAD) filter.reset();
    armPrev = armOn;

    bool upright = std::fabs(s.az - GRAVITY) < 2.0f;
    bool readyToArm = (flight.getState() != STATE_PAD) || upright;

    FlightInput in;
    in.nowMs = nowMs;
    in.dt = dt;
    in.accelTotal = std::sqrt(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
    in.baroAlt = s.baroAlt;
    in.baroValid = s.baroValid;
    in.filtAlt = filter.getAlt();
    in.filtVel = filter.getVel();
    in.armSwitchOn = armOn;
    in.sensorsOk = readyToArm;
    flight.update(in);
    (void)lastT; lastT = t;

    // ---- record what happened ----
    FlightState st = flight.getState();
    if (flight.pyroActive() && !res.fired) {
      res.fired = true;
      res.fireT = t;
      res.fireAlt = z;
      res.fireVel = v;
      res.fireState = (int)st;
      tFire = t;
    }
    if (flight.pyroActive() && (st == STATE_PAD || st == STATE_ARMED)) res.firedOnPad = true;
    if ((flight.getEvents() & EVENT_KALMAN_APOGEE) && kalmanApogeeT < 0) kalmanApogeeT = t;
    if (st >= STATE_BOOST && std::fabs(in.filtAlt - z) > res.maxFiltErrM) res.maxFiltErrM = std::fabs(in.filtAlt - z);

    if (csv) {
      LogRow row;
      row.tMs = nowMs; row.state = (int)st; row.pyro = flight.pyroActive(); row.arm = armOn;
      row.s = s; row.filtAlt = in.filtAlt; row.filtVel = in.filtVel; row.vertAccel = vertAccel; row.events = flight.getEvents();
      char line[400];
      loggingFormatRow(line, sizeof(line), row);
      fputs(line, csv);
    }

    // Stop a few seconds after landing is detected
    if (st == STATE_LANDED) { res.landedDetected = true; if (t > res.fireT + 20 && z == 0) break; }
  }

  res.events = flight.getEvents();
  res.finalStatePad = (flight.getState() == STATE_PAD);
  if (apogeeSeen && kalmanApogeeT >= 0) res.kalmanApogeeErrS = kalmanApogeeT - res.apogeeT;
  if (csv) fclose(csv);
  return res;
}

static const char *sourceName(uint8_t ev) {
  if (ev & EVENT_BARO_FIRE) return "baro";
  if (ev & EVENT_TIMER_FIRE) return "TIMER";
  return "none";
}

int main(int argc, char **argv) {
  int failures = 0;

  auto check = [&](bool ok, const char *msg) {
    if (!ok) { printf("   FAIL: %s\n", msg); failures++; }
  };

  // ---- 1. Nominal flight ----
  {
    Scenario sc;
    sc.name = "nominal";
    Result r = runFlight(sc, argc > 1 ? argv[1] : nullptr);
    printf("[nominal] apogee %.1f m at t=%.2f s (%.2f s after ignition)\n", r.apogeeAlt, r.apogeeT, r.apogeeT - sc.ignitionAtS);
    printf("          fired at t=%.2f s by %s, alt %.1f m, true vertical speed %.2f m/s, %.2f s after apogee\n", r.fireT, sourceName(r.events), r.fireAlt, r.fireVel, r.fireT - r.apogeeT);
    printf("          impact speed %.1f m/s, landed detected: %d, Kalman apogee error %.2f s, worst filter altitude error %.1f m\n", r.impactSpeed, r.landedDetected, r.kalmanApogeeErrS, r.maxFiltErrM);
    check(r.fired, "chute never fired");
    check(r.fireVel < 3.0, "fired while still going up fast");
    check(r.impactSpeed < 10.0, "hit the ground too fast");
    check(r.landedDetected, "landing not detected");
    check(!r.firedOnPad, "fired on the pad");
  }

  // ---- 2. Failure scenarios ----
  struct Case { Scenario sc; const char *expect; };
  std::vector<Case> cases;
  { Scenario s; s.name = "switch never turned on (ground test)"; s.armOnAtS = -1; cases.push_back({s, "NOFIRE"}); }
  { Scenario s; s.name = "switch off during flight (must still deploy)"; s.armOffAtS = 8.0; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "baro dies after arming, before launch (accel confirms launch, backup timer fires)"; s.baroDeadAtS = 3.0; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "baro dies right at launch"; s.baroDeadAtS = 5.5; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "IMU dies after arming (baro launch detection must catch it)"; s.imuDeadAtS = 3.0; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "IMU dead from power-up (must refuse to arm)"; s.imuDeadAtS = 0.0; cases.push_back({s, "NOARM"}); }
  { Scenario s; s.name = "baro dead from power-up (must refuse to arm)"; s.baroDeadAtS = 0.5; cases.push_back({s, "NOARM"}); }
  { Scenario s; s.name = "rocket lying down when the switch is flipped (must refuse to arm)"; s.tiltDeg = 90; s.noLaunch = true; s.maxTimeS = 15; cases.push_back({s, "NOARM"}); }
  { Scenario s; s.name = "5 g bump on the pad, then real launch"; s.shockAtS = 3.0; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "5 g bump 1 s before ignition (launch clock must not start early)"; s.shockAtS = 4.0; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "5 g bump on the pad, NO launch"; s.shockAtS = 3.0; s.noLaunch = true; s.maxTimeS = 20; cases.push_back({s, "NOFIRE"}); }
  { Scenario s; s.name = "rocket sitting armed with no launch for 60 s"; s.noLaunch = true; s.maxTimeS = 60; cases.push_back({s, "NOFIRE"}); }
  { Scenario s; s.name = "80 ms loop stall near apogee"; s.stallAtS = 5.0 + 8.8; s.stallMs = 80; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "very noisy baro (3 m) with outliers"; s.baroSigma = 3.0; s.baroOutlierProb = 0.005; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "rocket leaning 25 degrees"; s.tiltDeg = 25; cases.push_back({s, "FIRE"}); }
  { Scenario s; s.name = "accelerometer bias of 2 m/s^2"; s.accelBias = 2.0; cases.push_back({s, "FIRE"}); }

  for (auto &c : cases) {
    Result r = runFlight(c.sc);
    printf("[%s]\n", c.sc.name);
    printf("          fired=%d by %s at t=%.2f s (%.2f s after apogee), true speed %.1f m/s, alt %.0f m, impact %.1f m/s, fired on pad=%d\n",
           r.fired, sourceName(r.events), r.fireT, r.fireT - r.apogeeT, r.fireVel, r.fireAlt, r.impactSpeed, r.firedOnPad);
    check(!r.firedOnPad, "fired on the pad or while armed");
    if (std::string(c.expect) == "NOFIRE") {
      check(!r.fired, "fired when it must not");
    } else if (std::string(c.expect) == "NOARM") {
      check(!r.fired, "fired when it must not");
      check(r.finalStatePad, "armed when it should have refused");
    } else {
      check(r.fired, "never fired");
      check(r.fireVel < 3.0, "fired while still going up fast");
      check(r.impactSpeed < 12.0, "hit the ground too fast");
    }
  }

  // ---- 3. Random tests: many flights with random noise, lean, bias, and outliers ----
  {
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    int runs = 500, early = 0, notFired = 0, fast = 0, timerFires = 0, notLanded = 0;
    double worstFireVel = -1e9, worstImpact = 0, sumDelay = 0, worstDelay = -1e9, bestDelay = 1e9;
    for (int i = 0; i < runs; i++) {
      Scenario s;
      s.name = "random";
      s.seed = 1000 + i;
      s.baroSigma = 0.3 + 2.7 * uni(rng);
      s.baroOutlierProb = uni(rng) < 0.5 ? 0.0 : 0.005 * uni(rng);
      s.accelSigma = 0.1 + 0.9 * uni(rng);
      s.accelBias = -1.0 + 2.0 * uni(rng);
      s.tiltDeg = 15.0 * uni(rng);
      Result r = runFlight(s);
      if (!r.fired) { notFired++; continue; }
      if (r.fireVel >= 3.0) early++;
      if (r.fireVel > worstFireVel) worstFireVel = r.fireVel;
      if (r.impactSpeed > worstImpact) worstImpact = r.impactSpeed;
      if (r.impactSpeed >= 12.0) fast++;
      if (r.events & EVENT_TIMER_FIRE) timerFires++;
      if (!r.landedDetected) notLanded++;
      double delay = r.fireT - r.apogeeT;
      sumDelay += delay;
      if (delay > worstDelay) worstDelay = delay;
      if (delay < bestDelay) bestDelay = delay;
    }
    printf("[random x%d] never fired: %d, fired early (still rising > 3 m/s): %d, hit ground > 12 m/s: %d, landing missed: %d, fired by timer: %d\n",
           runs, notFired, early, fast, notLanded, timerFires);
    printf("          chute delay after true apogee: best %.2f s, average %.2f s, worst %.2f s; worst true speed at fire %.2f m/s; worst impact %.1f m/s\n",
           bestDelay, sumDelay / (runs - notFired), worstDelay, worstFireVel, worstImpact);
    check(notFired == 0, "random runs where the chute never fired");
    check(early == 0, "random runs where the chute fired early");
    check(fast == 0, "random runs that hit the ground fast");
  }

  printf("\n%s (%d failures)\n", failures == 0 ? "ALL CHECKS PASSED" : "SOME CHECKS FAILED", failures);
  return failures == 0 ? 0 : 1;
}
