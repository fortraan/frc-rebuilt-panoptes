#include "utilities.h"

std::vector<std::string> removeRosArgs(const int argc, const char* const argv[]) {
    std::vector<std::string> ret;
    ret.reserve(argc);

    bool isRosArgs = false;
    for (int i = 0; i < argc; i++) {
        const std::string currentArg = argv[i];

        if (isRosArgs) {
            if (currentArg == "--") isRosArgs = false;
        } else {
            if (currentArg == "--ros-args") {
                isRosArgs = true;
                continue;
            }
            ret.push_back(currentArg);
        }
    }

    return ret;
}