//============================================================================
// Name        : Draw.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include "opencv/cv.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>				//basic building blocks
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <iostream>
//
using namespace cv;
using namespace std;

/*
 * Abrir una imagen
 */
int main(int argc, char* argv[]) {

	vector<Mat> v_img;
	Mat im;

	String nombre1 = "im";
	int num_tot_im = 16;
	String nombre2 = ".jpg";

	for (int i = 1; i <= num_tot_im; i++){
		String nombre = nombre1 + i + nombre2;
		cout << nombre << endl;
		im = imread(nombre, CV_LOAD_IMAGE_COLOR);
		if(!im.data){
			cout << "error loading image" << endl;
			return 1;
		}
		v_img.push_back(im); //Añade la imagen im al vector v_img

		namedWindow("image", CV_WINDOW_AUTOSIZE);
		imshow("image", im);
	}



	waitKey(0);
	destroyAllWindows();

	return 0;
}
