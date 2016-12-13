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
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
/*
 * Abrir una imagen
 */
int main(int argc, char* argv[]) {

	vector<Mat> v_im;
	vector<vector<KeyPoint> > v_keypoints;
	Mat im;
	vector<KeyPoint> kp;

	String nombre1 = "/home/irenerrrd/Images/im";
	int num_tot_im = 16;
	String nombre2 = ".jpg";

	int minHessian = 1000;
	Ptr<SIFT> detector = SIFT::create(minHessian);

	for (int i = 1; i <= num_tot_im; i++){
		stringstream ss;
		ss << i;

		String nombre = nombre1 + ss.str() + nombre2;
		cout << nombre << endl;
		im = imread(nombre, CV_LOAD_IMAGE_COLOR);
		resize(im, im, Size(), 0.25, 0.25);
		if(!im.data){
			cout << "error loading image" << endl;
			return 1;
		}
		detector->detect(im, kp);
		v_im.push_back(im); //Añade la imagen im al vector v_img
		v_keypoints.push_back(kp);
		namedWindow("image", CV_WINDOW_AUTOSIZE);
		imshow("image", im);

		//-- Draw keypoints
		Mat im_keypoints;

		drawKeypoints(im, kp, im_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

		//-- Show detected (drawn) keypoints
		imshow("Keypoints 1", im_keypoints);

		waitKey(0);
	}

	destroyAllWindows();

	return 0;
}
