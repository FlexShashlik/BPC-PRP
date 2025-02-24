#include <iostream>
#include "lib.hpp"

#define A 5

int sum(int a, int b) {
    return a + b;
}

int main() {
    std::cout << "My Cool CLI Compiled Program" << std::endl;
    int b = 10;
    std::cout << "Sum result: " << sum(A, b) << std::endl;

    std::cout << "Max is " << max(A, b) << std::endl;

    return 0;
}

