/*
 * StructureFromMotion.cpp
 *
 *  Created on: 14 ene. 2017
 *      
 */

#include "Geometria3D.h"


struct CloudPoint {
    cv::Point3d pt;
    std::vector<int>index_of_2d_origin;
    float color;
};

int main(int argc, char* argv[]) {
    ofstream myfile;
    myfile.open("confor.txt");

    vector<CloudPoint> todos_los_puntos_en_3D;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);


    Mat intrinsic = (Mat_<double>(3,3) << 2759.48, 0, 1520.69, 0, 2764.16, 1006.81, 0, 0, 1);
//			2918.173910427262, 0, 1224.577959082814,
//			0, 2712.285042743833, 1598.890793819125,
//			0, 0, 1);
    double focal = intrinsic.at<double>(0, 0);
    cv::Point2d pp(intrinsic.at<double>(0, 2), intrinsic.at<double>(1, 2));
    Mat distortion = (Mat_<double>(1,5) << 0,0,0,0,0);
//	Mat distortion =(Mat_<double>(1,5) << 0.1063833678903079, -0.3218427230614517, 0.001458832745731512, 0.0006713282326283284, 0.3293767665489676);

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * INICIO DE LA SECCIÓN 1 DEL CODIGO
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    vector<Mat> v_im;
    vector<vector<KeyPoint> > v_keypoints;
    vector<Mat> v_descriptors;
    vector<Mat> v_transf;

    if (argc != 3){
    		cout << "Error en el numero de argumentos de entrada" << endl;
    		return -1;
    	}

    Mat im;
    vector<KeyPoint> kp;
    Mat dp;
    vector<Matrix4f> transfEigen;
    Mat zeros = (Mat_<double>(3, 1) << 0, 0, 0);
    Mat_<double> bottom = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

//	String nombre1 = "/home/irenerrrd/Descargas/images/im";
    String nombre0 = argv[2];
    cout << "Carpeta: " << nombre0 << endl;
    String nombre1 = "im";
    String nombre2 = ".png";

    int num_tot_im = atoi(argv[1]);

    //	int minHessian = 500;
    Ptr<SIFT> detector = SIFT::create(0);

    //Lee las imagenes y las almacena en v_im
    //Y obtiene los keypoints y sus descriptores y los almacena en v_keypoints y v_descriptors
    for (int i = 1; i <= num_tot_im; i++){
        stringstream ss;
        ss << i;

        String nombre = nombre0 + nombre1 + ss.str() + nombre2;
        cout << "Leida " << nombre << endl;
        im = imread(nombre, CV_LOAD_IMAGE_COLOR);
        if(!im.data){
            cout << "error loading image" << endl;
            return 1;
        }

        //Añade la imagen im al vector v_img
//        resize(im, im, Size(), 0.25, 0.25);
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
    vector<vector<Point2d> > good_objects, good_scenes;
    vector<vector<int> > good_objects_idx, good_scenes_idx;

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * INICIO DE LA SECCIÓN 2 DEL CODIGO
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    //Obtención de los puntos 3D entre las dos primeras imágenes
    // Hace todos los matches entre pares de imagenes y elimina los outliers
    for (int l = 0; l < num_tot_im - 1; l++){
        vector<Point2d> obj, scene;
        vector<int> obj_idx, scene_idx;
        obtainMatches(v_keypoints[l], v_keypoints[l+1], v_descriptors[l], v_descriptors[l+1], obj, scene, obj_idx, scene_idx);
        cout << "Matches entre la imagen " << l << " y la imagen " << l+1 << ": " << obj.size() << ", " << scene.size() << endl;

        Mat E, mask;
        E = findEssentialMat(obj, scene, focal, pp, RANSAC, 0.99, 1.0, mask);

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
        good_scenes.push_back(good_scene);
        good_scenes_idx.push_back(good_scene_idx);
        good_objects.push_back(good_obj);
        good_objects_idx.push_back(good_obj_idx);
    }


    //Obtención de la matriz esencial y de la matrix de transformación entre las dos cámaras
    Mat E, R, t, mask;
    E = findEssentialMat(good_objects[0], good_scenes[0], focal, pp, RANSAC, 0.9, 3.0, mask);
    recoverPose(E, good_objects[0], good_scenes[0], R, t, focal, pp, mask);


    //Obtención de los puntos en 3D
    Mat tran, tran1, points4D, A, B;

    hconcat(R,t,tran);
    hconcat(intrinsic, zeros, tran1);
    A = tran1;
    B = intrinsic*tran;
    v_transf.push_back(B);

    triangulatePoints(A, B, good_objects[0], good_scenes[0], points4D);
    vector<Point3d> PointCloud;
    convertHomogeneous(points4D, PointCloud);

    //Creación de la nube de puntos 3D inicial
    for (int i = 0; i < points4D.cols; i++){
        CloudPoint point;
	    pcl::PointXYZRGB point_pcl;

        point.pt = PointCloud[i];
        point_pcl.x = point.pt.x;
        point_pcl.y = point.pt.y;
        point_pcl.z = point.pt.z;

        vector<int> indices (num_tot_im, 0);
        indices[0] = good_objects_idx[0][i];
        indices[1] = good_scenes_idx[0][i];
        point.index_of_2d_origin = indices;

        Vec3b color = v_im[0].at<Vec3b>(Point(good_objects[0][i].x,good_objects[0][i].y));
        uint8_t r = (uint8_t)color.val[2];
        uint8_t g = (uint8_t)color.val[1];
        uint8_t b = (uint8_t)color.val[0];
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        point_pcl.rgb = *reinterpret_cast<float*>(&rgb);
        point.color = *reinterpret_cast<float*>(&rgb);

        todos_los_puntos_en_3D.push_back(point);
        point_cloud_ptr->push_back(point_pcl);
    }

    myfile << "points1 = [";
    for (int k = 0; k < (int)todos_los_puntos_en_3D.size(); k++){
        myfile << todos_los_puntos_en_3D[k].pt.x <<", "<< todos_los_puntos_en_3D[k].pt.y <<", "<< todos_los_puntos_en_3D[k].pt.z << endl;
    }
    myfile << "]" << endl;
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * FIN DE LA SECCIÓN 2 DEL CODIGO
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * INICIO DE LA SECCIÓN 3 DEL CODIGO
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    vector<CloudPoint> puntos_en_3D_anteriores = todos_los_puntos_en_3D;

    //Obtención de los keypoints y descriptors de la nube de puntos anterior
    for (int i = 2; i < num_tot_im - 1 /*num_pares_im*/; i++){

        vector<KeyPoint> keypoint3d;
        KeyPoint key3d;
        Mat descriptors3d;
        int posicion;

        for(unsigned int j = 0; j < puntos_en_3D_anteriores.size(); j ++){
            //			cout << i << ", " << j << endl;
            posicion = puntos_en_3D_anteriores[j].index_of_2d_origin[i-1];
            //			cout << "j: " << j << " --  i-1: " << i-1 << " -- posicion: " << posicion << " -- v_desc_tot: " << v_descriptors[i-1].size() << endl;
            descriptors3d.push_back(v_descriptors[i-1].row(posicion));
            //			cout << "C" << endl;
            key3d = v_keypoints[i-1][posicion];
            keypoint3d.push_back(key3d);
        }
        cout << "3d" << keypoint3d.size() << ", " << "kp" << v_keypoints[i].size() << endl;
        //Obtención de los puntos característicos y descriptors a través del matcheado entre la nube de puntos anterior y la imagen actual

        vector<Point2d> obj_new, scene_new;
        vector<int> obj_idx_new, scene_idx_new;
        obtainMatches(keypoint3d, v_keypoints[i], descriptors3d, v_descriptors[i], obj_new, scene_new, obj_idx_new, scene_idx_new);

        //Obtener R y t de la cámara a través de la PnPRansac
        Mat rvec, tvec;
        vector<Point3d> pnpPointcloud_valid;
        Point3d  punto;

        for(unsigned int j = 0; j < obj_idx_new.size(); j ++){
            punto = puntos_en_3D_anteriores[obj_idx_new[j]].pt;
            pnpPointcloud_valid.push_back(punto);
        }
        cout << pnpPointcloud_valid.size() << ", " << scene_new.size() << endl;
        solvePnPRansac(pnpPointcloud_valid, scene_new, intrinsic, distortion, rvec, tvec, false, 300, 3.0);
        cout << "Ransac done!" << endl;
        //Convertir de formato Rodrigues a matriz de rotación y construir la matriz de Transformación 3x4

        Mat r_mat;
        Rodrigues(rvec,r_mat);
        hconcat(r_mat,tvec,tran);

        //Triangulamos los puntos
        B.release();
        B = intrinsic*tran;
        v_transf.push_back(B);

        Mat new_points4D;
        vector<Point3d> new_PointCloud;
        cout << "Imagen " << i-1 << " con imagen " << i << ": " << good_objects[i-1].size() << endl;
        triangulatePoints(v_transf[i-2], v_transf[i-1], good_objects[i-1], good_scenes[i-1], new_points4D);
        //		cout << new_points4D << endl;
        convertHomogeneous(new_points4D, new_PointCloud);

        puntos_en_3D_anteriores.clear();

        for (int j = 0; j < (int)new_PointCloud.size(); j++){

            if ((good_objects_idx[i-1][j] < (int)v_keypoints[i-1].size())
                    && (good_objects_idx[i-1][j] >= 0)
                    && (good_scenes_idx[i-1][j] < (int)v_keypoints[i].size())
                    && (good_scenes_idx[i-1][j] >= 0)){

                CloudPoint point;
        	    pcl::PointXYZRGB point_pcl;

                point.pt = new_PointCloud[j];
                point_pcl.x = point.pt.x;
                point_pcl.y = point.pt.y;
                point_pcl.z = point.pt.z;

                vector<int> indices (num_tot_im, 0); //Solo se tiene un par de imágenes por lo que habrá que el vector tiene que corresponderse unicamente a estas dos imágenes
                indices[i-1] = good_objects_idx[i-1][j];
                indices[i] = good_scenes_idx[i-1][j];
//				cout << point.pt <<  " [" << indices[0] << ", " << indices[1] << ", " << indices[2] << ", " << indices[3] << ", " << indices[4] << "]" << endl;
                point.index_of_2d_origin = indices;

                Vec3b color = v_im[i-1].at<Vec3b>(Point(good_objects[i-1][j].x,good_objects[i-1][j].y));
                uint8_t r = (uint8_t)color.val[2];
                uint8_t g = (uint8_t)color.val[1];
                uint8_t b = (uint8_t)color.val[0];
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                point.color = *reinterpret_cast<float*>(&rgb);
                point_pcl.rgb = *reinterpret_cast<float*>(&rgb);

                todos_los_puntos_en_3D.push_back(point);
                puntos_en_3D_anteriores.push_back(point);
                point_cloud_ptr->push_back(point_pcl);

            }
        }
        myfile << "points" << i << " = [";
        for (int k = 0; k < (int)puntos_en_3D_anteriores.size(); k++){
            myfile << puntos_en_3D_anteriores[k].pt.x <<", "<< puntos_en_3D_anteriores[k].pt.y <<", "<< puntos_en_3D_anteriores[k].pt.z << endl;
        }
        myfile << "]" << endl;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * FIN DE LA SECCIÓN 3 DEL CODIGO
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
   	viewer.showCloud(point_cloud_ptr);
   	while(!viewer.wasStopped()){

   	}
    destroyAllWindows();

    return(0);

}
