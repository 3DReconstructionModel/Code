/*
 * Geometria3D.h
 *
 *  Created on: 20 dic. 2016
 *      Author: irenerrrd
 */

#ifndef SRC_GEOMETRIA3D_H_
#define SRC_GEOMETRIA3D_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
#include <iostream>
#include <Dense>
#include <opencv2/core/eigen.hpp>


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace Eigen;

template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line)
{
	//Line is given as a*x + b*y + c = 0
	return fabsf(line(0)*point.x + line(1)*point.y + line(2))
			/ std::sqrt(line(0)*line(0)+line(1)*line(1));
}

/**
 * \brief Compute and draw the epipolar lines in two images
 *      associated to each other by a fundamental matrix
 *
 * \param title     Title of the window to display
 * \param F         Fundamental matrix
 * \param img1      First image
 * \param img2      Second image
 * \param points1   Set of points in the first image
 * \param points2   Set of points in the second image matching to the first set
 * \param inlierDistance      Points with a high distance to the epipolar lines are
 *                not displayed. If it is negative, all points are displayed
 **/
template <typename T1, typename T2>
static void drawEpipolarLines(const std::string& title, const cv::Matx<T1,3,3> F,
		const cv::Mat& img1, const cv::Mat& img2,
		const std::vector<cv::Point_<T2> > points1,
		const std::vector<cv::Point_<T2> > points2,
		const float inlierDistance = -1)
{
	CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
	cv::Mat outImg(img1.rows, img1.cols*2, CV_8UC3);
	cv::Rect rect1(0,0, img1.cols, img1.rows);
	cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
	/*
	 * Allow color drawing
	 */
	if (img1.type() == CV_8U)
	{
		cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
		cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
	}
	else
	{
		img1.copyTo(outImg(rect1));
		img2.copyTo(outImg(rect2));
	}
	std::vector<cv::Vec<T2,3> > epilines1, epilines2;
	cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
	cv::computeCorrespondEpilines(points2, 2, F, epilines2);

	CV_Assert(points1.size() == points2.size() &&
			points2.size() == epilines1.size() &&
			epilines1.size() == epilines2.size());

	cv::RNG rng(0);
	for(size_t i=0; i<points1.size(); i++)
	{
		if(inlierDistance > 0)
		{
			if(distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
					distancePointLine(points2[i], epilines1[i]) > inlierDistance)
			{
				//The point match is no inlier
				continue;
			}
		}
		/*
		 * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
		 */
		cv::Scalar color(rng(256),rng(256),rng(256));

		cv::line(outImg(rect2),
				cv::Point(0,-epilines1[i][2]/epilines1[i][1]),
				cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]),
				color);
		cv::circle(outImg(rect1), points1[i], 3, color, -1, CV_AA);

		cv::line(outImg(rect1),
				cv::Point(0,-epilines2[i][2]/epilines2[i][1]),
				cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),
				color);
		cv::circle(outImg(rect2), points2[i], 3, color, -1, CV_AA);
	}
	cv::imshow(title, outImg);
	cv::waitKey(1);
}

void convertHomogeneous(Mat point4D, vector<Point3d> & point3D){
	Eigen::Matrix<float,Dynamic,Dynamic> change;

	cv2eigen(point4D, change);
	//		        MatrixXf mA((triangulation[t].rows -1), triangulation[t].cols);
	for ( int b = 0; b < point4D.cols; b++) {
		Point3d mA;
		if (change(3,b) == 0){
			mA.x = change(0,b);
			mA.y = change(1,b);
			mA.z = change(2,b);
		} else {
			mA.x = change(0,b)/change(3,b);
			mA.y = change(1,b)/change(3,b);
			mA.z = change(2,b)/change(3,b);
		}
		point3D.push_back(mA);
	}
}

void obtainMatches(vector<KeyPoint> kp1, vector<KeyPoint> kp2, Mat descriptors1, Mat descriptors2, vector<Point2d> & points1, vector<Point2d> & points2, vector<int> & points1_idx, vector<int> & points2_idx){

	BFMatcher matcher(NORM_L2,false);
	vector<vector< DMatch > > matches;
	matcher.knnMatch(descriptors1, descriptors2, matches, 2);

	vector<DMatch > Best_Matches;
	for(int k = 0; k < (int)matches.size(); k++){

		float dis1 = matches[k][0].distance ;
		float dis2 = matches[k][1].distance ;
		//				cout << dis1 << " " << dis2 << " --- ";
		if( (dis1 < 400.0 && dis1 > 0) || (dis2 < 400.0 && dis2 > 0) ){			//distancia pequeña entre imagen1 e imagen2[0] e imagen2[1]
			if (  dis2 / dis1 > 1.5){		//la diferencia de distancias es grande, por tanto una de ellas sera buena
				Best_Matches.push_back(matches[k][0]);
			}
		}
	}

	cv::RNG rng(0);

	for( int l = 0; l < (int)Best_Matches.size(); l++ ){
		//-- Get the keypoints from the good matches
		cv::Scalar color(rng(256),rng(256),rng(256));

		points1.push_back( kp1[ Best_Matches[l].queryIdx ].pt );
		points1_idx.push_back(Best_Matches[l].queryIdx);
		//				obj_desc.push_back(v_descriptors[i][ Best_Matches[l].queryIdx ]);
		points2.push_back( kp2[ Best_Matches[l].trainIdx ].pt );
		points2_idx.push_back(Best_Matches[l].trainIdx);

		//				scene_desc.push_back(v_descriptors[i][ Best_Matches[l].trainIdx ]);
	}
}

#endif /* SRC_GEOMETRIA3D_H_ */
