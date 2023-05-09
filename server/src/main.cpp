#include <memory>
#include <thread>

#include "../include/bridge.h"

int main(int argc, char** argv) {

    auto bridge = std::make_shared<rosweb::bridge>();
    std::thread(&rosweb::bridge::run, bridge.get()).detach();

    while (true) {}

    return 0;
}