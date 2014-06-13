#include <iostream>
#include "PositionModel.hpp"

int main()
{

    int index;
    std::string path;
    PositionModel model;

    std::cout<<"Type the path of the database\n";
    std::getline(cin,path);
    std::cout<<"Type the index of the face to visualze or -1 for the statistical model\n";
    std::cin>>index;




    //model.readDataFromFolders("/media/ace/New Volume/Google Summer/Part1/",150,4);

    model.readDataFromFolders(path,150,4);
    model.debug();
    model.calculateMeanFace();
    model.calculateEigenVectors();
    model.printEigenValues();
    model.readWeights("weights.txt");
    model.calculateModel_S();
    model.viewModel_S(index);


    return 0;
}
