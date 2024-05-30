/* Copyright 2024 The Agilex Robotics Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include <iostream>
#include <string>
#include "getCurrentPath.h"

// #ifdef _WIN32
// #include <windows.h>
// std::string getExecutablePath() {
//     char result[MAX_PATH];
//     GetModuleFileName(NULL, result, MAX_PATH);
//     return std::string(result);
// }
// #else
// #include <unistd.h>
// #include <limits.h>
// std::string getExecutablePath() {
//     char result[PATH_MAX];
//     ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
//     if (count != -1) {
//         return std::string(result, count);
//     }
//     return std::string();
// }
// #endif

// int main() {
//     std::string path = getExecutablePath();
//     std::cout << "Executable path: " << path << std::endl;
//     return 0;
// }

#include <iostream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <libgen.h> // for dirname
#include <string.h> // for strcpy

std::string getExecutablePath() {
    char result[MAX_PATH];
    GetModuleFileName(NULL, result, MAX_PATH);
    result[MAX_PATH-1] = '\0'; // Ensure null termination
    char* dir = dirname(result);
    return std::string(dir);
}

#else
#include <unistd.h>
#include <limits.h>
#include <libgen.h> // for dirname
#include <string.h> // for strcpy

std::string getExecutablePath() {
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    if (count != -1) {
        result[count] = '\0'; // Null-terminate the string
        char *dir = dirname(result);
        return std::string(dir);
    }
    return std::string();
}
#endif

int main() {
    std::string path = getExecutablePath();
    std::cout << "Executable path: " << path << std::endl;
    return 0;
}

