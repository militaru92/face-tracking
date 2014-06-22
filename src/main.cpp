#include <PositionModel.h>

int main(int argc, char** argv)
{

  if(argc < 3)
  {
    PCL_ERROR("You need to pass the path of the data base and the path where you want to store the model\n");
    exit(1);
  }

  std::string database_path,obj_path;

  database_path = argv[1];
  obj_path = argv[2];

  PositionModel model;


  //model.readDataFromFolders("/media/ace/New Volume/Google Summer/Part1/",150,4);

  model.readDataFromFolders(database_path,150,4);
  model.calculateMeanFace();
  model.calculateEigenVectors();
  model.printEigenValues();
  model.calculateRandomWeights(50,obj_path);
  model.calculateModel();
  model.writeModel(-1,obj_path);

  return (0);
}
