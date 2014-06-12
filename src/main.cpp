#include <iostream>
#include "PositionModel.hpp"

int main()
{

    PositionModel model;

    model.readDataFromFolders("/media/ace/New Volume/Google Summer/Part1/",150);
    model.calculateMeanFace();
    model.calculateCovariance();
    model.readWeights("weights.txt");
    model.calculateModel_S();

    return 0;
}
