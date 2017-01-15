/*
 * StructureFromMotion.cpp
 *
 *  Created on: 14 ene. 2017
 *
 */


#include "geometria3d.h"


struct CloudPoint {
    cv::Point3d pt;
    std::vector<int>index_of_2d_origin;
};

int main(int argc, char* argv[]) {

    vector<CloudPoint> todos_los_puntos_en_3D;


    Mat intrinsic = (Mat_<double>(3,3) << 2918.173910427262, 0, 1224.577959082814,
            0, 2712.285042743833, 1598.890793819125,
            0, 0, 1);
    double focal = intrinsic.at<double>(0, 0);
    cv::Point2d pp(intrinsic.at<double>(0, 2), intrinsic.at<double>(1, 2));
    Mat distortion =(Mat_<double>(1,5) << 0.1063833678903079, -0.3218427230614517, 0.001458832745731512, 0.0006713282326283284, 0.3293767665489676);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * INICIO DE LA SECCIÓN 1 DEL CODIGO
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
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
    String nombre2 = ".jpg";

    int num_tot_im = 5;

    //	int minHessian = 500;
    Ptr<SIFT> detector = SIFT::create(0);

    //Lee las imagenes y las almacena en v_im
    //Y obtiene los keypoints y sus descriptores y los almacena en v_keypoints y v_descriptors
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
    }
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * FIN DE LA SECCIÓN 1 DEL CODIGO
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    vector<Mat> transf;
    vector<vector<Point2d> > objects;
    vector<vector<Point2d> > scenes;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * INICIO DE LA SECCIÓN 2 DEL CODIGO
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    //Obtención de los puntos 3D entre las dos primeras imágenes
    cout << "Imagen " << 1 << " con Imagen " << 2 << endl;

    vector<Point2d> obj, scene;
    vector<int> obj_idx, scene_idx;
    obtainMatches(v_keypoints[0], v_keypoints[1], v_descriptors[0], v_descriptors[1], obj, scene, obj_idx, scene_idx);
    objects.push_back(obj);
    scenes.push_back(scene);

  //Obtención de la matriz esencial y de la matrix de transformación entre las dos cámaras
    Mat E, R,t, mask;
    undistortPoints(obj, obj, intrinsic, distortion, noArray(), intrinsic);
    undistortPoints(scene, scene, intrinsic, distortion, noArray(), intrinsic);
    E = findEssentialMat(obj, scene, focal, pp, RANSAC, 0.9, 3.0, mask);
/*    SVD svd(E);
    Matx33d W(0,-1,0,//HZ 9.13
              1,0,0,
              0,0,1);
    Mat_<double> R = svd.u * Mat(W) * svd.vt; //HZ 9.19
    Mat_<double> t = svd.u.col(2); //u3
    Matx34d P1( R(0,0),R(0,1), R(0,2), t(0),
    R(1,0),R(1,1), R(1,2), t(1),
    R(2,0),R(2,1), R(2,2), t(2));
*/
    correctMatches(E, obj, scene, obj, scene);
    recoverPose(E, obj, scene, R, t, focal, pp, mask);
   Mat im_matches_e;

    //En good_obj y good_scene se almacenan unicamente los puntos inliers
    vector<Point2d> good_scene, good_obj;
    vector<int> good_obj_idx, good_scene_idx;
    for (int b = 0; b < (int)obj.size(); b++){
        if (mask.at<uchar>(b) == 1){
            good_scene.push_back(scene[b]);
            good_scene_idx.push_back(scene_idx[b]);

            good_obj.push_back(obj[b]);
            good_obj_idx.push_back(obj_idx[b]);
        }
    }

    //Obtención de los puntos en 3D
    Mat tran, tran1, points4D, A, B;

    hconcat(R,t,tran);
    hconcat(intrinsic, zeros, tran1);
    A = tran1;
    B = intrinsic*tran;

    triangulatePoints(A, B, good_obj, good_scene, points4D);
    vector<Point3d> PointCloud;
    convertHomogeneous(points4D, PointCloud);

    //Creación de la nube de puntos 3D inicial
    for (int i = 0; i < points4D.cols; i++){
        CloudPoint point;

        point.pt = PointCloud[i];

        vector<int> indices (2, 0); //Solo se tiene un par de imágenes por lo que habrá que el vector tiene que corresponderse unicamente a estas dos imágenes
        indices[0] = good_obj_idx[i];
        indices[1] = good_scene_idx[i];
        point.index_of_2d_origin = indices;

        todos_los_puntos_en_3D.push_back(point);
    }

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * FIN DE LA SECCIÓN 2 DEL CODIGO
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * INICIO DE LA SECCIÓN 3 DEL CODIGO
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */







    //for(int i = 0; i < num_tot_im-2; i++)
    //{

    //Obtención de los keypoints y descriptors de la nube de puntos anterior

    vector<KeyPoint> keypoint3d;
    KeyPoint key3d;
    Mat descriptors3d;
    int posicion;

        for(unsigned int j = 0; j < todos_los_puntos_en_3D.size(); j ++)
        {   posicion = todos_los_puntos_en_3D[j].index_of_2d_origin[0];
            //cout<< v_descriptors[0].row(posicion)<< endl;
            descriptors3d.push_back(v_descriptors[0].row(posicion));
            key3d = v_keypoints[0][posicion];
            keypoint3d.push_back(key3d);
        }

        //Obtención de los puntos característicos y descriptors a través del matcheado entre la nube de puntos anterior y la imagen actual
        vector<Point2d> obj_new, scene_new;
        vector<int> obj_idx_new, scene_idx_new;
        obtainMatches(keypoint3d, v_keypoints[2], descriptors3d, v_descriptors[2], obj_new, scene_new, obj_idx_new, scene_idx_new);

        //Obtener R y t de la cámara a través de la PnPRansac

        Mat rvec, tvec;
        vector<Point3d> pnpPointcloud_valid;
        Point3d  punto;

        for(unsigned int j = 0; j < obj_idx_new.size(); j ++)
        {
            punto = todos_los_puntos_en_3D[obj_idx_new[j]].pt;
            pnpPointcloud_valid.push_back(punto);
        }

        solvePnPRansac(pnpPointcloud_valid, scene_new, intrinsic, distortion, rvec, tvec);

        //Convertir de formato Rodrigues a matriz de rotación y construir la matriz de Transformación 3x4

        Mat r_mat;
        Rodrigues(rvec,r_mat);
        hconcat(R,t,tran);

        //

    //}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * FIN DE LA SECCIÓN 3 DEL CODIGO
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */


















    //	waitKey(0);
    destroyAllWindows();

    return(0);
}

