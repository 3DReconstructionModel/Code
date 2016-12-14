#include "opencv/cv.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>				//basic building blocks
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <vector>
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
	vector<Mat> v_descriptors;
	Mat im;
	vector<KeyPoint> kp;
	Mat dp;

	ofstream outputfile;
	outputfile.open("/home/irenerrrd/Escritorio/info3d.txt");

	String nombre1 = "/home/irenerrrd/Images/im";
	int num_tot_im = 5;
	String nombre2 = ".jpg";

	int minHessian = 1000;
	Ptr<SIFT> detector = SIFT::create(minHessian);

	//Lee las imagenes y las almacena en v_im
	for (int i = 1; i <= num_tot_im; i++){
		stringstream ss;
		ss << i;

		String nombre = nombre1 + ss.str() + nombre2;
		cout << "Leida " << nombre << endl;
		im = imread(nombre, CV_LOAD_IMAGE_COLOR);
		resize(im, im, Size(), 0.15, 0.15);
		if(!im.data){
			cout << "error loading image" << endl;
			return 1;
		}

		//Añade la imagen im al vector v_img
		v_im.push_back(im);

		//Obtiene los puntos caracteristicos de cada una de las
		//imagenes y los almacena en v_keypoints y v_descriptors
		detector->detect(im, kp);
		detector->compute(im, kp, dp);
		v_keypoints.push_back(kp);
		v_descriptors.push_back(dp);

		//-- Draw keypoints
		Mat im_keypoints;
		drawKeypoints(im, kp, im_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
		//-- Show detected (drawn) keypoints
		imshow("Keypoints 1", im_keypoints);
		waitKey(10);
	}

	cout << "Pulsa cualquier tecla" << endl;
	waitKey(0);

	//Repasa todas las posibles combinaciones de imagenes
	for (int i = 0; i < num_tot_im - 1; i++){
		for (int j = i + 1; j < num_tot_im; j++){
			cout << "Imagen " << i+1 << " con Imagen " << j+1 << endl;

			BFMatcher matcher(NORM_L2,false);
			vector<vector< DMatch > > matches;
			matcher.knnMatch(v_descriptors[i], v_descriptors[j], matches, 2);

			vector<DMatch > Best_Matches;
			for(int k = 0; k < (int)matches.size(); k++){

				float dis1 = matches[k][0].distance ;
				float dis2 = matches[k][1].distance ;
//				cout << dis1 << " " << dis2 << " --- ";
				if( dis1 < 200.0 || dis2 < 200.0 ){			//distancia pequeña entre imagen1 e imagen2[0] e imagen2[1]
					if (  dis2 / dis1 > 1.2){		//la diferencia de distancias es grande, por tanto una de ellas sera buena
						if ( dis1 > dis2 )
							Best_Matches.push_back(matches[k][1]);	//la distancia mas pequeña es la buena
						else Best_Matches.push_back(matches[k][0]);
					}
				}
			}
			Mat im_matches;
			drawMatches( v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches,
					Scalar::all(-1),Scalar::all(-1),vector<char>(),
					DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
			imshow("knnMatches", im_matches);
			waitKey(10);

			vector<Point2f> obj;
			vector<Point2f> scene;

			for( int l = 0; l < (int)Best_Matches.size(); l++ ){
				//-- Get the keypoints from the good matches
				obj.push_back( v_keypoints[i][ Best_Matches[l].trainIdx ].pt );
				scene.push_back( v_keypoints[j][ Best_Matches[l].queryIdx ].pt );
			}

//			cout << "nº puntos encontrados: " << obj.size() << " - " << scene.size() << endl;
//			waitKey(0);

			Mat mask_inliers_f;
			Mat F = findFundamentalMat(obj, scene, CV_FM_RANSAC, 5, 0.99, mask_inliers_f);
			//		cout << "Tiempo de computo f: " << t << endl;
			cout << "Calculada matriz fundamental: " << F << endl;

 //			cout << mask_inliers_f << endl;
//			waitKey(0);
			Mat im_matches_fund;
			drawMatches( v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches_fund,
					Scalar::all(-1),Scalar::all(-1), mask_inliers_f,
					DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

			imshow("F Matches", im_matches_fund);
			cout << "Matches dibujados" << endl;
			waitKey(0);

			Mat E = findEssentialMat(obj, scene);
			Mat R1, R2, t1, R, t;
			decomposeEssentialMat(E, R1, R2, t1);
			recoverPose(E, obj, scene, R, t);
			outputfile << "Im" << i+1 << ", Im" << j+1 << endl;
			outputfile << "E = " << E << endl;
			outputfile << "R = " << R << endl;
			outputfile << "t = " << t << endl;
			outputfile << "--------------------" << endl << endl;


			Mat e;
			SVD::solveZ(F,e);
			cout << "e = " << e << endl;
			waitKey(0);
		}
	}

	destroyAllWindows();

	return 0;
}
