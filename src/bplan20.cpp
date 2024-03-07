/*  
 * 
 * Copyright © 2023 DTU,
 * Author:
 * Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */

#include <string>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"

#include "bplan20.h"

// create class object
BPlan20 plan20;


void BPlan20::setup()
{ // ensure there is default values in ini-file
  if (not ini["plan20"].has("log"))
  { // no data yet, so generate some default values
    ini["plan20"]["log"] = "true";
    ini["plan20"]["run"] = "false";
    ini["plan20"]["print"] = "true";
  }
  // get values from ini-file
  toConsole = ini["plan20"]["print"] == "true";
  //
  if (ini["plan20"]["log"] == "true")
  { // open logfile
    std::string fn = service.logPath + "log_plan20.txt";
    logfile = fopen(fn.c_str(), "w");
    fprintf(logfile, "%% Mission plan20 logfile\n");
    fprintf(logfile, "%% 1 \tTime (sec)\n");
    fprintf(logfile, "%% 2 \tMission state\n");
    fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
  }
  setupDone = true;
}

BPlan20::~BPlan20()
{
  terminate();
}


void BPlan20::run()
{
  if (not setupDone)
    setup();
  if (ini["plan20"]["run"] == "false")
    return;
  //
  UTime t("now");
  bool finished = false;
  bool lost = false;
  state = 10;
  oldstate = state;
  //
  toLog("Plan20 started");
  //
  while (not finished and not lost and not service.stop)
  {
    switch (state)
    { 
      case 10: // Starting the track >> Pass the gate >> see the intersection
        pose.resetPose();
        pose.dist = 0;
        toLog("starting at 0.3m/s");
        mixer.setEdgeMode(false /* right */, -0.03 /* offset */);
        mixer.setVelocity(0.3);
        state = 20;
        break;

      case 20: // Go to the right path
        pose.resetPose();
        pose.dist = 0;
        tolog("See intersection, go to the right, reduce the speed to 0.2");
        mixer.setEdgeMode(false/* right */, -0.03 /* offset */);
        mixer.setVelocity(0.2);
        state = 30;
        break;

      case 30: // Reaching the Slope
        pose.resetPose();
        pose.dist = 0;
        tolog("Climbing up the slope");
        mixer.setEdgeMode(false/* right */, -0.03/* offset */);
        mixer.setVelocity(0.2);
        state = 40;
        break;

      case 40: // Finish Climbing up
        pose.resetPose();
        pose.dist = 0;
        tolog("Continue straight to the bar");
        mixer.setEdgeMode(false/* left */, -0.03/* offset */);
        mixer.setVelocity(0.2);
        state = 50;
        break;

      case 50: // Turn left to the bar
        pose.resetPose();
        pose.dist = 0;
        tolog("Go left to the bar with very low speed");
        mixer.setEdgeMode(false/* left */, -0.03/* offset */);
        mixer.setVelocity(0.15);
        mixer.setTurnrate(1.0); //Turn left
        break;

      case 60: // stop turning and go straight to the bar
        if(pose.Turned > 1.7)
        {
          mixer.setTurnrate(0.0) // Stop turning
          mixer.setEdgeMode(false/* left */, -0.03/* offset */);
          mixer.setVelocity(0.2); 
  
        
        }
        



    }
    if (state != oldstate)
    {
      oldstate = state;
      toLog("state start");
      // reset time in new state
      t.now();
    }
    // wait a bit to offload CPU
    usleep(2000);
  }
  if (lost)
  { // there may be better options, but for now - stop
    toLog("Plan20 got lost");
    mixer.setVelocity(0);
    mixer.setTurnrate(0);
  }
  else
    toLog("Plan20 finished");
}


void BPlan20::terminate()
{ //
  if (logfile != nullptr)
    fclose(logfile);
  logfile = nullptr;
}

void BPlan20::toLog(const char* message)
{
  UTime t("now");
  if (logfile != nullptr)
  {
    fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
            oldstate,
            message);
  }
  if (toConsole)
  {
    printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec()/100,
           oldstate,
           message);
  }
}
