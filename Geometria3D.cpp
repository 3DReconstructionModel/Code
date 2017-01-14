#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
#include <iostream>
#include <Dense>
#include <opencv2/core/eigen.hpp>
#include <cvsba.h>
#include "geometria3d.h"


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace Eigen;
using namespace cvsba;


int main(int argc, char* argv[]) {
    Mat intrinsic = (Mat_<double>(3,3) << 2918.173910427262, 0, 1224.577959082814,
            0, 2712.285042743833, 1598.890793819125,
            0, 0, 1);
    Mat distortion =(Mat_<double>(1,5) << 0.1063833678903079, -0.3218427230614517, 0.001458832745731512, 0.0006713282326283284, 0.3293767665489676);
    Matrix4f Tintrinsic;
    Tintrinsic << 2918.173910427262, 0, 1224.577959082814,0,
            0, 2712.285042743833, 1598.890793819125,0,
            0, 0, 1, 0,
            0, 0, 0, 1;
    vector<Mat> v_im;
    vector<vector<KeyPoint> > v_keypoints;
    vector<Mat> v_descriptors;
    Mat im;
    vector<KeyPoint> kp;
    Mat dp;
    vector<Matrix4f> transfEigen;
    Mat zeros = (Mat_<double>(3, 1) << 0, 0, 0);
    Mat_<double> bottom = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    String nombre1 = "/home/pelayo/Documentos/Images/venga/im";
//	String nombre1 = "/media/datos/irenerrrd/Dropbox/Movil_calib/bodegon/im";
//	String nombre1 = "/media/datos/irenerrrd/Dropbox/Movil_calib/Biblio/im";
//		String nombre1 = "/home/irenerrrd/GitHub/Images/im";

    int num_tot_im = 5;
    String nombre2 = ".jpg";

    //	int minHessian = 500;
    Ptr<SIFT> detector = SIFT::create(0);

    //Lee las imagenes y las almacena en v_im
    for (int i = 1; i <= num_tot_im; i++){
        stringstream ss;
        ss << i;

        String nombre = nombre1 + ss.str() + nombre2;
        cout << "Leida " << nombre << endl;
        im = imread(nombre, CV_LOAD_IMAGE_COLOR);
        if(!im.data){
            cout << "error loading image" << endl;
            return 1;
        }

        //Añade la imagen im al vector v_img
        resize(im, im, Size(), 0.25, 0.25);
        v_im.push_back(im);

        //Obtiene los puntos caracteristicos de cada una de las
        //imagenes y los almacena en v_keypoints y v_descriptors
        detector->detect(im, kp);
        detector->compute(im, kp, dp);
        v_keypoints.push_back(kp);
        v_descriptors.push_back(dp);

        //-- Draw keypoints
//		Mat im_keypoints;
//		drawKeypoints(im, kp, im_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//		//-- Show detected (drawn) keypoints
//		imshow("Detected keypoints", im_keypoints);
//		waitKey(0);
    }

    cout << "Pulsa cualquier tecla" << endl;
    waitKey(10);
    vector<Mat> transf;
    vector<Matrix4f> transf_eigen;
    vector<vector<Point2f> > objects;
    vector<vector<Point2f> > scenes;
    vector<Mat> rotations_rodrigues;
    Mat rodriguesf;
    vector<Mat> ttotal;

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
                if( (dis1 < 400.0 && dis1 > 0) || (dis2 < 400.0 && dis2 > 0) ){			//distancia pequeña entre imagen1 e imagen2[0] e imagen2[1]
                    if (  dis2 / dis1 > 1.5){		//la diferencia de distancias es grande, por tanto una de ellas sera buena
                        Best_Matches.push_back(matches[k][0]);
                    }
                }
            }

//			Mat im_matches;
//			drawMatches( v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches,
//					Scalar::all(-1),Scalar::all(-1),vector<char>(),
//					DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//			resize(im_matches, im_matches, Size(), 0.15, 0.15);
//			imshow("knnMatches", im_matches);
//			waitKey(0);

            vector<Point2f> obj;
            vector<Point2f> scene;
            cv::RNG rng(0);

            for( int l = 0; l < (int)Best_Matches.size(); l++ ){
                //-- Get the keypoints from the good matches
                cv::Scalar color(rng(256),rng(256),rng(256));

                obj.push_back( v_keypoints[i][ Best_Matches[l].queryIdx ].pt );
                scene.push_back( v_keypoints[j][ Best_Matches[l].trainIdx ].pt );
            }

            objects.push_back(obj);
            scenes.push_back(scene);

//			Mat mask_inliers_f;
//			Mat F = findFundamentalMat(obj, scene, CV_FM_RANSAC, 1, 0.999, mask_inliers_f);
//			cout << "Calculada matriz fundamental: " << F << endl;
//			Mat im_matches_fund;
//			drawMatches(v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches_fund,
//					Scalar::all(-1),Scalar::all(-1), mask_inliers_f,
//					DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//			imshow("F Matches", im_matches_fund);
//			cout << "Matches dibujados" << endl;
//			waitKey(0);
//			String title = "Lineas epipolares";
//			Matx33d Fx((double*)F.ptr());
//			cout << "Fx:" << Fx << endl << "F:" << F << endl;
//			drawEpipolarLines(title , Fx, v_im[i], v_im[j], obj, scene, (float)1.0);

            double focal = intrinsic.at<double>(0, 0);
            cv::Point2d pp(intrinsic.at<double>(0, 2), intrinsic.at<double>(1, 2));
            Mat E, R, t, mask;
            undistortPoints(obj, obj, intrinsic, distortion, noArray(), intrinsic);
            undistortPoints(scene, scene, intrinsic, distortion, noArray(), intrinsic);
            E = findEssentialMat(obj, scene, focal, pp, RANSAC, 0.9, 3.0, mask);
            correctMatches(E, obj, scene, obj, scene);
            recoverPose(E, obj, scene, R, t, focal, pp, mask);
            Mat im_matches_e;

            drawMatches( v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches_e,
                    Scalar::all(-1),Scalar::all(-1), mask,
                    DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            resize(im_matches_e, im_matches_e, Size(), 0.25, 0.25);
            imshow("E Matches", im_matches_e);
            waitKey(1);
            Mat tran;
            hconcat(R,t,tran);
            Mat tran1;
            vconcat(tran, bottom, tran1);
            Matrix4f tran_eigen;
            cv2eigen(tran1,tran_eigen);
            transf.push_back(tran);
            transf_eigen.push_back(tran_eigen);
            Rodrigues(R,rodriguesf);
            rotations_rodrigues.push_back(rodriguesf);
            ttotal.push_back(t);
        }
    }

    Mat points4Da;

    //Triangulation of points
    int num_comb = 10;
    MatrixXf A_eigen(4,4), B_eigen(4,4), C_eigen(4,4), D_eigen(4,4);
    Mat A, B, C, D;
    vector<Mat> triangulation;
    MatrixXf points4Da_eigen;
    Point3d points4Db;

    int numero = 0;
    for(int y = 0; y < num_comb; y ++)
    {
        if(y < 4)
        {
            A_eigen = Tintrinsic;
            B_eigen = Tintrinsic*transf_eigen[y];
            A_eigen.conservativeResize(3,4);
            B_eigen.conservativeResize(3,4);
            eigen2cv(A_eigen,A);
            eigen2cv(B_eigen,B);
            triangulatePoints(A, B, objects[y], scenes[y], points4Da);

/*            cv2eigen(points4Da, points4Da_eigen);
            numero = points4Da.cols;
                for(int l= 0; l < numero; l++)
                {
                    points4Db =cv::Point3d(points4Da_eigen(0,0),points4Da_eigen(0,1),points4Da_eigen(0,2));
                    points.push_back(points4Db);
                    cout << l << endl;

                }

*/
            triangulation.push_back(points4Da);
            //points.push_back(points4Db);
        }
        if(y >= 4 && y < 7)
        {
            A_eigen = Tintrinsic*transf_eigen[0];
            B_eigen = Tintrinsic*transf_eigen[y]*transf_eigen[0];
            A_eigen.conservativeResize(3,4);
            B_eigen.conservativeResize(3,4);
            eigen2cv(A_eigen,A);
            eigen2cv(B_eigen,B);
            triangulatePoints(A, B, objects[y], scenes[y], points4Da);
            triangulation.push_back(points4Da);
        }
        if(y >= 7 && y < 9)
        {
            A_eigen = Tintrinsic*transf_eigen[1];
            B_eigen = Tintrinsic*transf_eigen[y]*transf_eigen[1];
            A_eigen.conservativeResize(3,4);
            B_eigen.conservativeResize(3,4);
            eigen2cv(A_eigen,A);
            eigen2cv(B_eigen,B);
            triangulatePoints(A, B, objects[y], scenes[y], points4Da);
            triangulation.push_back(points4Da);
        }
        if(y == 9)
        {
            A_eigen = Tintrinsic*transf_eigen[2];
            B_eigen = Tintrinsic*transf_eigen[y]*transf_eigen[2];
            A_eigen.conservativeResize(3,4);
            B_eigen.conservativeResize(3,4);
            eigen2cv(A_eigen,A);
            eigen2cv(B_eigen,B);
            triangulatePoints(A, B, objects[y], scenes[y], points4Da);
            triangulation.push_back(points4Da);
        }

    }

    //Convert to euclidean coordinates

    Eigen::Matrix<float,Dynamic,Dynamic> change;
    vector < Mat> points3D;
    for(int t = 0; t < num_comb; t++)
    {
        cv2eigen(triangulation[t],change);
        MatrixXf mA((triangulation[t].rows -1), triangulation[t].cols);
        for ( int b = 0; b < triangulation[t].cols; b++)
        {
           if (change(3,b)== 0){
               mA(0,b)= change(0,b);
               mA(1,b)= change(1,b);
               mA(2,b)= change(2,b);
           }
           else
           {
               mA(0,b)= change(0,b)/change(3,b);
               mA(1,b)= change(1,b)/change(3,b);
               mA(2,b)= change(2,b)/change(3,b);
           }
        }
        Mat mA_mat;
        eigen2cv(mA, mA_mat);
        points3D.push_back(mA_mat);
    }

    //Create an array of points

    vector <Point3d> points;
    int cuenta = 0;
    int columns;
    for(int y = 0; y < num_comb; y ++)
    {
        cv2eigen(points3D[y], points4Da_eigen);
        columns = points3D[y].cols;
        int intermedio = 0;
            for(int l= 0; l < columns; l++)
            {
                points4Db =cv::Point3d(points4Da_eigen(0,l),points4Da_eigen(1,l),points4Da_eigen(2,l));
                points.push_back(points4Db);
                intermedio = l + cuenta;
            }
            cuenta = intermedio + 1;
    }

    //Create img point vector

    int num_points = (int) points.size();
    vector<vector<Point2d> > imagePoints;
    vector<Point2d> trans;
    MatrixXf objects0_eigen;
    int count;
    //int num_points = (int) points.size();
    for (int g = 0; g <= num_tot_im - 1; g++){
        cout << "g" << objects[g].size() << endl;
        count = objects[g].size();

           for (int i=0 ; i<count; i++)
            {
               trans.push_back( cv::Point2d( (double)objects[g][0].x, (double)objects[g][0].y  ) );
            }

            imagePoints.push_back(trans);
            trans.pop_back();
            //int limit = (int) objects[0].size();
            //cv2eigen(objects[0][0],objects0_eigen);
            /*for(int f = 0; f < limit; f++ ){
                point = points[f];

            }*/
            count = 0 ;
     }

//Creating the visibility matrix


vector<vector<int> > visibility;
vector < int > fila1(objects[0].size(), 1);
visibility.push_back(fila1);
vector < int > fila2(objects[1].size(), 1);
visibility.push_back(fila2);
vector < int > fila3(objects[2].size(), 1);
visibility.push_back(fila3);
vector < int > fila4(objects[3].size(), 1);
visibility.push_back(fila4);
vector < int > fila5(objects[4].size(), 1);
visibility.push_back(fila5);

/*int yes = 1; int no = 0;
    for (int c = 0; c <= num_tot_im - 1; c++){
        vector<int> row;
           for (int i=0 ; i<points.size(); i++)
            {
               //cout << "dentro2" << endl;
               if(i <= objects[c].size()){
                //cout<< "dentro3" << endl;
                row.push_back(yes);
               }
               else if(i > objects[c].size())
               {
                visibility[c][i]= (int) 0;
                 //row.push_back(no);
               }
            visibility.push_back(row);
           }
}*/


//Creating the camera matrix vector and the distortion coefficients vector

vector<Mat> cameraMatrix;
vector<Mat>distCoeffs;
    for (int i = 0; i <= num_tot_im - 1; i++){
        cameraMatrix.push_back(intrinsic);
        distCoeffs.push_back(distortion);
    }
/*
//Create a txt
    ofstream myfile;
     myfile.open ("Points.txt");
        myfile << "Points: \n" << points << endl;
     int num_image = imagePoints.size();
     for (int i = 0; i < num_image; i++){
       myfile << "Image Points: \n" << imagePoints[i] << endl;
     }

     myfile.close();
*/
    //Size printing
    ofstream myfile;
     myfile.open ("Sizes.txt");
        myfile << "vector <Point3d> points Size\n" << points.size() << endl;
        myfile << "vector<vector<Point2d> > imagePoints Size\n" << imagePoints.size()  << "x" << endl;
        for (int i = 0; i < imagePoints.size() ; i++){
        myfile << " Número i " << i << ":"<< imagePoints[i].size() << endl;
        }
        myfile << "vector<vector<int> > visibility; Size\n" << visibility.size() << "x" << visibility[0].size() << endl;
        for (int i = 0; i < visibility.size(); i++){
        myfile << " Número i " << i << ":" << visibility[i].size() << endl;
        }
        myfile << "vector<Mat> cameraMatrix: Size\n" << cameraMatrix.size() << endl;
        myfile << "vector<Mat> rotations_rodrigues: Size\n" << rotations_rodrigues.size() << endl;
        myfile << "vector<Mat> ttotal: Size\n" << ttotal.size() << endl;
        myfile << "vector<Mat>distCoeffs: Size\n" << distCoeffs.size() << endl;

    // int num_image = imagePoints.size();
     //for (int i = 0; i < num_image; i++){
      // myfile << "Image Points: \n" << imagePoints[i] << endl;
    // }

     myfile.close();

   //Bundle adjustment

    Sba sba;
    sba.run(points, imagePoints, visibility, cameraMatrix, rotations_rodrigues, ttotal, distCoeffs);

        cout<<"Initial error="<<sba.getInitialReprjError()<<". "<<
                    "Final error="<<sba.getFinalReprjError()<<endl;

    waitKey(0);

    destroyAllWindows();

    return(0);
}

