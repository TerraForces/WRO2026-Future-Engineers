#define VERSION "0.0.0"

#include <algorithm>
#include <numeric>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <thread>
#include <vector>

#include "main.hpp"

struct RESULT {
    uint8_t pts;
    uint32_t id; // 28^4 options * 2 directions
    uint32_t us;
    double minDist; // mm
};

void simulate(RESULT* result) {
    setup();
    loop();
}

#define OPTION(A, a, s) else if((c == A) || (c == a) || ((c == '-') && (strcasecmp(arg + 2, s) == 0)))

int main(int argc, char* argv[]) {
    std::vector<RESULT> results;
    uint64_t threadLimit = 0;
    bool cw = 0, ccw = 0;
    for(uint_fast32_t i = 2; i < argc; i++) {
        char* arg = argv[i];
        char c = arg[1];
        if(*arg != '-') {
            //printf("warning: unexpected command-line argument: %s\n", arg);
        }
        OPTION('D', 'd', "dir") {
            if(++i < argc) {
                if(strcasecmp(argv[i], "ccw") == 0) cw = 0;
                else if(strcasecmp(argv[i], "cw") == 0) ccw = 0;
                else printf("warning: invalid direction, both directions will be tested\n");
            }
            else printf("warning: no direction specified, both directions will be tested\n");
        }
        OPTION('H', 'h', "help") {
            printf("Usage: %s [cases] [options]\nOptions:\n  -d --dir [cw/ccw]  only test the specified direction (cw = clockwise or ccw = counterclockwise)\n  -h --help         print this (hopefully helpful) information\n  -t --threads [n]  set a maximum number of threads to use while testing\n  -v --version      print program version\n", argv[1]);
            return 0;
        }
        OPTION('T', 't', "threads") {
            if(++i < argc) threadLimit = atol(argv[i]);
            else printf("warning: no thread limit specified, one thread for every test case will be created\n");
        }
        OPTION('V', 'v', "version") {
            printf(VERSION "\n");
            return 0;
        }
        else printf("warning: unrecognized command-line option: %s\n", arg);
    }
    if(cw && ccw) printf("error: both directions have been disabled (both -d cw and -d ccw have been specified)\n");
    printf("\n id | dir | pts | time (us) | closest (mm)\n----+-----+-----+-----------\n");
    if(results.empty()) for(uint32_t i = 0; i < (1 << 21); i += 2) {
        if(ccw) results.push_back({ .id = i });
        if(cw) results.push_back({ .id = i + 1 });
    }
    uint64_t caseCount = results.size();  
    RESULT* result = results.data();
    for(uint_fast8_t i = 0; i < caseCount; i++) {
        simulate(result);
        printf("%2hhu | %s | %3hhu | %9u | %12f\n", result->id >> 1, (result->id & 1) ? "cw" : "ccw", result->pts, result->us, result->minDist);
        result++;
    }
    if(caseCount > 1) {
        printf("\n  metric      |  min  |  max  |  avg  |  med\n--------+-------+-------\n");
        #define METRIC(name, sName) std::sort(results.begin(), results.end(), [](const RESULT& a, const RESULT& b) -> bool { return a.sName < b.sName; }); printf(" " name " | %5f | %5f | %5f | %5f\n", results.front().sName, results.back().sName, std::accumulate(results.begin(), results.end(), 0.0, [](double a, RESULT b) { return a + b.sName; }) / caseCount, (caseCount & 1) ? (results[caseCount / 2].sName) : ((results[(caseCount / 2) - 1].sName + results[caseCount / 2].sName) / 2.0))
        METRIC("points      ", pts);
        METRIC("time (s)    ", us);
        METRIC("closest (mm)", minDist);
    }
}