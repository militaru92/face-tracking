face-tracking
=============

Type the command : cmake . && make && ./face <path to the Tester directories from the face warehouse database> test_non_rigid -m cloud4.pcd 0.166666666 0.001

in order to write the final result for the cloud4.pcd scan in the test_non_rigid.pcd file

Command for using the Asus Xtion and automatic alignment: ./face <path-to-the-Facewarehouse-DataBase> <name-of-the-output-file-without-extension> <path-to-OpenCV-classifier> 0.166666666 0.001 -c

The last three arguments represent parameters for the filter of the rigid transformation and the option argument to use the 3D Sensor
