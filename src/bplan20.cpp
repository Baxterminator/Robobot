/*
 *
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
 *
 * The MIT License (MIT)  https://mit-license.org/
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. */

#include "cedge.h"
#include "cmixer.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "mpose.h"
#include "sencoder.h"
#include "steensy.h"
#include "uservice.h"
#include "utime.h"
#include <math.h>
#include <string.h>
#include <string>
#include <unistd.h>

#include "bplan20.h"

// create class object
BPlan20 plan20;

void BPlan20::setup() { // ensure there is default values in ini-file
  if (not ini["plan20"].has(
          "log")) { // no data yet, so generate some default values
    ini["plan20"]["log"] = "true";
    ini["plan20"]["run"] = "false";
    ini["plan20"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan20"]["print"] == "true";
  //
  if (ini["plan20"]["log"] == "true") { // open logfile
    std::string fn = service.logPath + "log_plan20.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan20 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan20::~BPlan20() { terminate(); }

constexpr float DEG_90{1.5707};
constexpr float ROBOT_SPEED{0.3};
constexpr float ROBOT_TR{1.0};

void BPlan20::run() {
  if (not setupDone)
    setup();
  if (ini["plan20"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 11;
  oldstate = state;
  //
  toLog("Backup plan started");
  //
  while (not finished and not lost and not service.stop) {
    switch (state) {

    // Advance forward
    case 11:
      pose.resetPose();
      pose.dist = 0;
      mixer.setVelocity(ROBOT_SPEED);
      state = 12;
      break;

    // Wait for distance to complete and turn right (align to first gate)
    case 12:
      if (pose.dist > 0.75) {
        pose.resetPose();
        pose.turned = 0;
        mixer.setVelocity(0.0);
        mixer.setTurnrate(-ROBOT_TR);
        state = 14;
      }
      break;

    // Wait for angle and go forward (pass first gate)
    case 14:
      if (std::fabs(pose.turned) > DEG_90) {
        mixer.setTurnrate(0.0);
        pose.resetPose();
        pose.dist = 0;
        mixer.setVelocity(ROBOT_SPEED);
        state = 16;
      }
      break;

    // Wait for distance and turn left (align for going by the crossing path)
    case 16:
      if (pose.dist > 2.5) {
        pose.resetPose();
        pose.dist = 0;
        pose.turned = 0;
        mixer.setVelocity(0.0);
        mixer.setTurnrate(ROBOT_TR);
        state = 18;
      }
      break;

    // Wait for angle and go straight (long crossing run)
    case 18:
      if (std::fabs(pose.turned) > 0.7) {
        pose.resetPose();
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0.0);
        mixer.setVelocity(ROBOT_SPEED);
      }
      break;

    // Wait for the distance and go turn left (align for the goal)
    case 20:
      if (pose.dist > 5.0) {
        pose.resetPose();
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(ROBOT_TR);
        mixer.setVelocity(0.0);
      }
      break;

    // Wait for the angle, and go straight (to hit the goal)
    case 22:
      if (std::fabs(pose.turned) > DEG_90) {
        pose.resetPose();
        pose.dist = 0;
        pose.turned = 0;
        mixer.setTurnrate(0.0);
        mixer.setVelocity(ROBOT_SPEED);
      }
      break;

    // Wait for goal hitting
    case 24:
      if (pose.dist > 3.0) {
        finished = true;
      }
      break;

    default:
      toLog("Unknown state");
      lost = true;
      break;
    }
    if (state != oldstate) {
      oldstate = state;
      toLog("state start");
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);
  }
  if (lost) { // there may be better options, but for now - stop
    toLog("Plan20 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  } else
    toLog("Plan20 finished");
}

void BPlan20::terminate() { //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan20::toLog(const char *message) {
  UTime t("now");
  if (logfile != nullptr) {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
            oldstate, message);
  }
  if (toConsole) {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100, oldstate,
           message);
  }
}
