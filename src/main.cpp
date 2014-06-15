#include <iostream>
#include "PositionModel.hpp"

int main()
{

    int index,NumberSamples;
    std::string path,obj_path;
    PositionModel model;

    std::cout<<"Type the path of the database\n";
    std::getline(cin,path);
    std::cout<<"Type the index of the face to visualze or -1 for the statistical model\n";
    std::cin>>index;
    getchar();
    std::cout<<"Type the path for the obj file\n";
    std::getline(cin,obj_path);
    std::cout<<"Type the number of eigenvectors to use\n";
    std::cin>>NumberSamples;

    //std::cout<<"This" << obj_path << std::endl;





    //model.readDataFromFolders("/media/ace/New Volume/Google Summer/Part1/",150,4);

    model.readDataFromFolders(path,150,4);
    model.debug();
    model.calculateMeanFace();
    model.calculateEigenVectors();
    model.printEigenValues();
    //model.readWeights("weights.txt");
    model.calculateRandomWeights(NumberSamples,obj_path);
    model.calculateModel_S();
    //model.viewModel_S(index);
    model.writeModel_S(index,obj_path);


    return 0;
}
