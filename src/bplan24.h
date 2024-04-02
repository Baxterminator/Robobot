#pragma once


using namespace std;

/**
 * Class intended to accomplish a short mission,
 * e.g. one challenge or part of a challenge
 * */
class BPlan24
{
public:
    /**
     * destructor */
    ~BPlan24();
    /** setup and request data */
    void setup();
    /**
     * run this mission */
    void run();
    /**
     * terminate */
    void terminate();

private:
    /**
     * Write a timestamped message to log */
    void toLog(const char* message);
    /// added to log
    int state, oldstate;
    /// private stuff
    // debug print to console
    bool toConsole = true;
    // logfile
    FILE* logfile = nullptr;
    bool setupDone = false;
    bool obstacleDetected = false;
};

/**
 * Make this visible to the rest of the software */
extern BPlan24 plan24;