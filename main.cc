#include "include/model.h"

#include <iostream>
#include <ratio>

int main()
{
    std::cout << "hi" << std::endl;
    std::cout << dyn::model<double, std::ratio<1105, 1000>, std::ratio<1738, 1000>>{} << std::endl;
}
