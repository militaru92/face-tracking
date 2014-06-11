#include <iostream>
#include "PositionModel.hpp"

int main()
{

    std::cout<<"Asa\n";
    PositionModel model;

    model.readDataFromFolders("/media/ace/New Volume/Google Summer/Part1/",150);
    model.calculateMeanFace();
    model.calculateCovariance();

    return 0;
}
