#ifndef TXTCREATE_H
#define TXTCREATE_H

#include <Dense>
#include <fstream>

using namespace Eigen;
using namespace std;

void txtcreate(Eigen::Matrix<float,Dynamic,Dynamic> matrix1, Eigen::Matrix<float,Dynamic,Dynamic> matrix2 )
{
    ofstream myfile;

          myfile.open ("Matrixes.txt");
          myfile << "Matrix 1:\n " << matrix1 << endl;
          myfile << "Matrix 2:\n " << matrix2 << endl;

    myfile.close();
}

#endif // TXTCREATE_H
