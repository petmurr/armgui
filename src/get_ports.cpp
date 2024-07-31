#if defined(__linux__) || defined(__APPLE__)
#include <iostream>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <vector>

bool isPortOpen(const std::string &portName) {
    int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        // Port is not openable
        return false;
    } else {
        // Port is openable, close it after checking
        close(fd);
        return true;
    }
}

// std::vector<std::string> listPorts(const std::string &baseDir, const std::string &prefix) {
std::vector<std::string> listPorts() {
    const std::string baseDir = "/dev"; // TODO test this on linux
    const std::string prefix = "ttyUSB";
    DIR *dir = opendir(baseDir.c_str());
    if (dir == nullptr) {
        perror("opendir");
    }

    // vector of strings
    std::vector<std::string> dev_vector = {};

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, prefix.c_str(), prefix.size()) == 0) {
            std::string portName = baseDir + "/" + entry->d_name;
            // if (isPortOpen(portName)) {
            //     std::cout << "Open port: " << portName << std::endl;
            // } else {
            //     std::cout << "Closed port: " << portName << std::endl;
            // }

            dev_vector.push_back(portName);
        }
    }

    if (dev_vector.size() == 0) {
        std::string no_ports_availabile = "????????????"; // needs to be same length since it's const!! this is stupid and you need to change this
        dev_vector.push_back(no_ports_availabile);
    }

    closedir(dir);
    return dev_vector;
}

#elif defined(_WIN32) || defined(_WIN64)

#include <iostream>
#include <windows.h>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>

std::wstring s2ws(const std::string& s) {
    int len;
    int slength = static_cast<int>(s.length()) + 1;
    len = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    std::wstring r(len, L'\0');
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, &r[0], len);
    return r;
}

bool isPortOpen(const std::string &portName) {
    // std::wstring wPortName = s2ws(portName);
    std::wstring wPortName = s2ws("\\\\.\\" + portName);
    HANDLE hComm = CreateFileW(wPortName.c_str(),
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              NULL,
                              OPEN_EXISTING,
                              0,
                              NULL);

    if (hComm == INVALID_HANDLE_VALUE) {
        // Port is not openable
        return false;
    } else {
        // Port is openable, close it after checking
        CloseHandle(hComm);
        return true;
    }
}

std::vector<std::string> listPorts() {
    const std::string prefix = "COM";
    std::vector<std::string> dev_vector;
    for (int i = 1; i <= 255; i++) {  // COM ports usually range from COM1 to COM255
        std::ostringstream oss;
        oss << prefix << i;
        std::string portName = oss.str();
        
        if (isPortOpen(portName)) {
            dev_vector.push_back(portName);
        }
    }

    if (dev_vector.empty()) {
        dev_vector.push_back("No ports available!");
    }

    return dev_vector;
}

#endif

