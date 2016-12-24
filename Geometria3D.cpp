#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <vector>
#include <iostream>
#include <Dense>
#include <opencv2/core/eigen.hpp>

#include "geometria3d.h"


using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;
using namespace Eigen;


    int main(int argc, char* argv[]) {
        vector<Mat> v_im;
        vector<vector<KeyPoint> > v_keypoints;
        vector<Mat> v_descriptors;
        Mat im;
        vector<KeyPoint> kp;
        Mat dp;
        vector<Matrix4f> transfEigen;

        String nombre1 = "/home/pelayo/Documentos/Images/hu/im";
        int num_tot_im = 5;
        String nombre2 = ".jpg";

        int minHessian = 500;
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
        vector<Mat> transf;


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
                        if (  dis2 / dis1 > 2){		//la diferencia de distancias es grande, por tanto una de ellas sera buena
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
                  cv::RNG rng(0);

                for( int l = 0; l < (int)Best_Matches.size(); l++ ){
                    //-- Get the keypoints from the good matches
                    cv::Scalar color(rng(256),rng(256),rng(256));

                    obj.push_back( v_keypoints[i][ Best_Matches[l].queryIdx ].pt );
                    scene.push_back( v_keypoints[j][ Best_Matches[l].trainIdx ].pt );
                }

                Mat mask_inliers_f;
                Mat F = findFundamentalMat(obj, scene, CV_FM_RANSAC, 1, 0.999, mask_inliers_f);
                //      cout << "Calculada matriz fundamental: " << F << endl;

                Mat im_matches_fund;

                drawMatches(v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches_fund,
                        Scalar::all(-1),Scalar::all(-1), mask_inliers_f,
                        DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                imshow("F Matches", im_matches_fund);
                cout << "Matches dibujados" << endl;
                waitKey(0);
                String title = "Lineas epipolares";
                Matx33d Fx((double*)F.ptr());
                //cout << "Fx:" << Fx << endl << "F:" << F << endl;
                drawEpipolarLines(title , Fx, v_im[i], v_im[j], obj, scene, (float)1.0);
                double focal = 1.0;
                cv::Point2d pp(0.0, 0.0);
                Mat E, R, t, mask;

                E = findEssentialMat(obj, scene, focal, pp, RANSAC, 0.9, 3.0, mask);
                recoverPose(E, obj, scene, R, t, focal, pp, mask);

                drawMatches( v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches_fund,
                        Scalar::all(-1),Scalar::all(-1), mask,
                        DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                imshow("E Matches", im_matches_fund);

                //cout << "R: " << R << endl << "t:" << t << endl;

                vector<char> datos(4, 0);
                datos[3] = 1;
                Mat tran;


                Mat_<double> bottom = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
                //			bottom.at<char>(0,3) = 1;
                hconcat(R,t,tran);
                vconcat(tran, bottom, tran);
                //cout << "tran"<< tran << endl;
                //Conversion a Eigen
                Eigen::Matrix<float,Dynamic,Dynamic> b;
                cv2eigen(tran,b);
                //cout << "Eigen" << b << endl;
                transfEigen.push_back(b);
                transf.push_back(tran);
                waitKey(0);
            }
        }
        //Se multiplican las matrices de Tf con Eigen tipo T12*T23=T13, T12*T24=T14...
        int comb_sistema = 3;
         vector<Matrix4f> MultMatrix1, MultMatrix2, MultMatrix3, MultMatrix4, MultMatrix5;
         vector<float> constant_relations;
            //Para cada sistema se hacen sus 3 combinaciones
            for(int p = 0; p < comb_sistema; p++){
               //T12*T23, T12*T24...
               MultMatrix1[p] = transfEigen[0]*transfEigen[p+5];
               //T21*T13...
               MultMatrix2[p] = transfEigen[4]*transfEigen[p+1];
               //T31*...
               if( p == 0){
                   MultMatrix3[p] = transfEigen[8]*transfEigen[0];
               }
               MultMatrix3[p] = transfEigen[8]*transfEigen[p+1];
               //T41*...
               if( p == 2){
                   MultMatrix4[p] = transfEigen[12]*transfEigen[3];
               }
               MultMatrix4[p] = transfEigen[12]*transfEigen[p];
               //T51*...
               MultMatrix5[p] = transfEigen[16]*transfEigen[p];
            }
          //Para obtener las relaciones K2/K3;
          for(int r = 0; r < comb_sistema; r++){
              constant_relations[r] = transfEigen[r+1](1,1)/MultMatrix1[r](1,1);
              constant_relations[r+3] = transfEigen[r+5](1,1) /MultMatrix2[r](1,1);
              constant_relations[r+9] = transfEigen[r+9](1,1) /MultMatrix3[r](1,1);
              constant_relations[r+12] = transfEigen[r+13](1,1) /MultMatrix4[r](1,1);
              constant_relations[r+15] = transfEigen[r+17](1,1) /MultMatrix5[r](1,1);
          }
        waitKey(0);
        destroyAllWindows();

        return(0);
    }
