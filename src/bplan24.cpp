#include <string>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "mpose.h"
#include "steensy.h"
#include "sdist.h"
#include "uservice.h"
#include "sencoder.h"
#include "utime.h"
#include "cmotor.h"
#include "cservo.h"
#include "medge.h"
#include "cedge.h"
#include "cmixer.h"

#include "bplan24.h"

// create class object
BPlan24 plan24;




void BPlan24::setup() 
    { // ensure there is default values in ini-file
        if (not ini["plan24"].has("log"))
        { // no data yet, so generate some default values
            ini["plan24"]["log"] = "true";
            ini["plan24"]["run"] = "true";
            ini["plan24"]["print"] = "true";
        }
        // get values from ini-file
        toConsole = ini["plan24"]["print"] == "true";
        //
        if (ini["plan24"]["log"] == "true")
        { // open logfile
            std::string fn = service.logPath + "log_plan24.txt";
            logfile = fopen(fn.c_str(), "w");
            fprintf(logfile, "%% Mission plan24 logfile\n");
            fprintf(logfile, "%% 1 \tTime (sec)\n");
            fprintf(logfile, "%% 2 \tMission state\n");
            fprintf(logfile, "%% 3 \t%% Mission status (mostly for debug)\n");
        }
        setupDone = true;
    }

    BPlan24::~BPlan24()
    {
        terminate();
    }

void BPlan24::run() {
    {
        if (not setupDone)
            setup();
        // if (ini["plan24"]["run"] == "false")
        //     return;
        //
        UTime t("now");
        bool finished = false;
        bool lost = false;
        state = 240;
        oldstate = state;
        //
        toLog("Plan24 started");
        //
        while (not finished and not lost and not service.stop)
        {
            float distance = dist.dist[0];
            std::cout << "Got sensor valuw = " << distance << std::endl;
            switch (state)
            { // make a shift in heading-mission
            case 240:
                pose.resetPose();
                toLog("Checking for obstacles.");
                state = 241;
                break;
            case 241:
                if (distance > 0.08) {
                    toLog("Path is clear, moving forward.");
                    mixer.setVelocity(0.3); // 前进速度
                    state = 242;
                }
                break;
            case 242:
                // 前进中检查障碍物
                if (distance <= 0.08) {
                    toLog("Obstacle detected, stopping.");
                    mixer.setVelocity(0); // 停止
                    obstacleDetected = true;
                    state = 243;
                }
                break;
            case 243:
                // 等待障碍物消失
                if (distance > 0.08) {
                    toLog("Obstacle cleared, continuing.");
                    mixer.setVelocity(0.3); // 继续前进
                    state = 242;
                    obstacleDetected = false;
                }
                break;
            default:
                toLog("Unknown state");
                finished = true; 
                break;

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
            toLog("Plan24 got lost");
            mixer.setVelocity(0);
            mixer.setTurnrate(0);
        }
        else
            toLog("Plan24 finished");
    }

}

void BPlan24::terminate()
{ //
    if (logfile != nullptr)
        fclose(logfile);
    logfile = nullptr;
}

void BPlan24::toLog(const char* message)
{
    UTime t("now");
    if (logfile != nullptr)
    {
        fprintf(logfile, "%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
            oldstate,
            message);
    }
    if (toConsole)
    {
        printf("%lu.%04ld %d %% %s\n", t.getSec(), t.getMicrosec() / 100,
            oldstate,
            message);
    }
}