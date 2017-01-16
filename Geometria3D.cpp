//
//#include "Geometria3D.h"
//
//struct CloudPoint {
//	cv::Point3d pt;
//	std::vector<int>index_of_2d_origin;
//};
//
//int main(int argc, char* argv[]) {
//    vector<CloudPoint> todos_los_puntos_en_3D;
//
//	Mat intrinsic = (Mat_<double>(3,3) << 2918.173910427262, 0, 1224.577959082814,
//			0, 2712.285042743833, 1598.890793819125,
//			0, 0, 1);
//	Mat distortion =(Mat_<double>(1,5) << 0.1063833678903079, -0.3218427230614517, 0.001458832745731512, 0.0006713282326283284, 0.3293767665489676);
//
//	vector<Mat> v_im;
//	vector<vector<KeyPoint> > v_keypoints;
//	vector<Mat> v_descriptors;
//
//	ofstream myfile;
//	myfile.open ("Matrixes2.txt");
//	//	vector<vector<Point2f> > comun_points;
//	Mat im;
//	vector<KeyPoint> kp;
//	Mat dp;
//	vector<Matrix4f> transfEigen;
//	Mat zeros = (Mat_<double>(3, 1) << 0, 0, 0);
//	Mat_<double> bottom = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
//
//	String nombre1 = "/media/datos/irenerrrd/Dropbox/MovilCalib/im";
//	//	String nombre1 = "/media/datos/irenerrrd/Dropbox/Movil_calib/bodegon/im";
//	//	String nombre1 = "/media/datos/irenerrrd/Dropbox/Movil_calib/Biblio/im";
//	//			String nombre1 = "/home/irenerrrd/GitHub/Images/im";
//
//	int num_tot_im = 5;
//	String nombre2 = ".jpg";
//
//	//	int minHessian = 500;
//	Ptr<SIFT> detector = SIFT::create(0);
//
//	//Lee las imagenes y las almacena en v_im
//	for (int i = 1; i <= num_tot_im; i++){
//		stringstream ss;
//		ss << i;
//
//		String nombre = nombre1 + ss.str() + nombre2;
//		cout << "ok " << nombre << endl;
//		im = imread(nombre, CV_LOAD_IMAGE_COLOR);
//		if(!im.data){
//			cout << "error loading image" << endl;
//			return 1;
//		}
//
//		//Añade la imagen im al vector v_img
////				resize(im, im, Size(), 0.5, 0.5);
//		v_im.push_back(im);
//
//		//Obtiene los puntos caracteristicos de cada una de las
//		//imagenes y los almacena en v_keypoints y v_descriptors
//		detector->detect(im, kp);
//		detector->compute(im, kp, dp);
//		v_keypoints.push_back(kp);
//		v_descriptors.push_back(dp);
//
//		//-- Draw keypoints
//		//		Mat im_keypoints;
//		//		drawKeypoints(im, kp, im_keypoints, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
//		//		//-- Show detected (drawn) keypoints
//		//		imshow("Detected keypoints", im_keypoints);
//		//		waitKey(0);
//	}
//
//	waitKey(10);
//	vector<Mat> transf;
//	vector<Matrix4f> transf_eigen;
//	vector<vector<Point2d> > objects;
//	vector<vector<Point2d> > scenes;
//
//
//	//Repasa todas las posibles combinaciones de imagenes
//	//	for (int i = 0; i < num_tot_im - 1; i++){
//	//		for (int j = i + 1; j < num_tot_im; j++){
//	cout << "Imagen 1 con Imagen 2"  << endl;
//	vector<int> obj_idx, scene_idx;
//	vector<Point2d> obj, scene;
//	obtainMatches(v_keypoints[0], v_keypoints[1], v_descriptors[0], v_descriptors[1], obj, scene, obj_idx, scene_idx);
//
//	objects.push_back(obj);
//	scenes.push_back(scene);
//
//	double focal = intrinsic.at<double>(0, 0);
//	cv::Point2d pp(intrinsic.at<double>(0, 2), intrinsic.at<double>(1, 2));
//	Mat E, R, t, mask;
////	undistortPoints(obj, obj, intrinsic, distortion, noArray(), intrinsic);
////	undistortPoints(scene, scene, intrinsic, distortion, noArray(), intrinsic);
//	E = findEssentialMat(obj, scene, focal, pp, RANSAC, 0.99, 3.0, mask);
////	correctMatches(E, obj, scene, obj, scene);
//	recoverPose(E, obj, scene, R, t, focal, pp, mask);
//	Mat im_matches_e;
//	vector<Point2d> good_scene, good_object;
//	vector<int> good_obj_idx, good_scene_idx;
//
//	for (int b = 0; b < (int)obj.size(); b++){
//		if (mask.at<char>(b) == 1){
//			good_scene.push_back(scene[b]);
//			good_object.push_back(obj[b]);
//			good_scene_idx.push_back(scene_idx[b]);
//			good_obj_idx.push_back(obj_idx[b]);
//		}
//	}
//	cout << obj.size() << endl;
//	//			drawMatches( v_im[i], v_keypoints[i], v_im[j], v_keypoints[j], Best_Matches, im_matches_e,
//	//					Scalar::all(-1),Scalar::all(-1), mask,
//	//					DrawMatchesFlags::DRAW_RICH_KEYPOINTS|DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//	//			resize(im_matches_e, im_matches_e, Size(), 0.25, 0.25);
//	//			imshow("E Matches", im_matches_e);
//	//			//			cout << obj.size() << scene.size() << mask << endl;
//	//			waitKey(1);
//
//	Mat tran, tran1, A, B;
//	hconcat(R,t,tran);
//	hconcat(intrinsic, zeros, tran1);
//
//	//Triangulation of points
//	Mat points4D;
//	vector<Point3d> PointCloud;
//
//	A = tran1;
//	B = intrinsic*tran;
//
//	cout << "A =" << A << "B =" << B << endl;
//	triangulatePoints(A, B, good_object, good_scene, points4D);
//	convertHomogeneous(points4D, PointCloud);
//	myfile << "tr0 = " << PointCloud << endl;
//
//	//Creación de la nube de puntos 3D inicial
//	for (int i = 0; i < points4D.cols; i++){
//		CloudPoint point;
//
//		point.pt = PointCloud[i];
//
//		vector<int> indices (num_tot_im, 0); //Solo se tiene un par de imágenes por lo que habrá que el vector tiene que corresponderse unicamente a estas dos imágenes
//		indices[0] = good_obj_idx[i];
//		indices[1] = good_scene_idx[i];
//		point.index_of_2d_origin = indices;
//
//		todos_los_puntos_en_3D.push_back(point);
//	}
//
//
//	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//	 * FIN DE LA SECCIÓN 2 DEL CODIGO
//	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//
//
//	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
//	 * INICIO DE LA SECCIÓN 3 DEL CODIGO
//	 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//
//	//Obtención de los keypoints y descriptors de la nube de puntos anterior
//
//	vector<KeyPoint> keypoint3d;
//	KeyPoint key3d;
//	Mat descriptors3d;
//	int posicion;
//
//
//	for(unsigned int j = 0; j < todos_los_puntos_en_3D.size(); j ++){
//
//		posicion = todos_los_puntos_en_3D[j].index_of_2d_origin[1];
//		//cout<< v_descriptors[0].row(posicion)<< endl;
//		descriptors3d.push_back(v_descriptors[1].row(posicion));
//		key3d = v_keypoints[1][posicion];
//		keypoint3d.push_back(key3d);
//	}
//
//	//Obtención de los puntos característicos y descriptors a través del matcheado entre la nube de puntos anterior y la imagen actual
//
//	vector<Point2d> obj_new, scene_new;
//	vector<int> obj_idx_new, scene_idx_new;
//	obtainMatches(keypoint3d, v_keypoints[2], descriptors3d, v_descriptors[2], obj_new, scene_new, obj_idx_new, scene_idx_new);
//
//	//Obtener R y t de la cámara a través de la PnPRansac
//
//	Mat rvec, tvec;
//	vector<Point3d> pnpPointcloud_valid;
//	Point3d  punto;
//
//	for(unsigned int j = 0; j < obj_idx_new.size(); j ++){
//		punto = todos_los_puntos_en_3D[obj_idx_new[j]].pt;
//		pnpPointcloud_valid.push_back(punto);
//	}
//
//	solvePnPRansac(pnpPointcloud_valid, scene_new, intrinsic, distortion, rvec, tvec, false, 300, 3.0);
//
//	//Convertir de formato Rodrigues a matriz de rotación y construir la matriz de Transformación 3x4
//
//	Mat r_mat;
//	Rodrigues(rvec,r_mat);
//	hconcat(R,t,tran);
//
//	//Ahora se obtienen Matches de la imagen 2 con la 3
//
//	vector<Point2d> obj_more_points, scene_more_points, good_more_scene, good_more_object;
//	vector<int> obj_idx_more_points, scene_idx_more_points, good_more_scene_idx, good_more_obj_idx;
//
//	obtainMatches(v_keypoints[1], v_keypoints[2], v_descriptors[1], v_descriptors[2], obj_more_points, scene_more_points, obj_idx_more_points, scene_idx_more_points);
////	undistortPoints(obj_more_points, obj_more_points, intrinsic, distortion, noArray(), intrinsic);
////	undistortPoints(scene_more_points, scene_more_points, intrinsic, distortion, noArray(), intrinsic);
//	E = findEssentialMat(obj_more_points, scene_more_points, focal, pp, RANSAC, 0.99, 3.0, mask);
////	correctMatches(E, obj_more_points, scene_more_points, obj_more_points, scene_more_points);
//
//	for (int b = 0; b < (int)obj.size(); b++){
//		if (mask.at<char>(b) == 1){
//			good_more_scene.push_back(scene_more_points[b]);
//			good_more_object.push_back(obj_more_points[b]);
//			good_more_scene_idx.push_back(scene_idx_more_points[b]);
//			good_more_obj_idx.push_back(obj_idx_more_points[b]);
//		}
//	}
//
//	//Triangulamos los puntos
//
//	B = intrinsic*tran;
//
//	Mat new_points4D;
//	vector<Point3d> new_PointCloud;
//	triangulatePoints(A, B, good_more_object, good_more_scene, new_points4D);
//	convertHomogeneous(new_points4D, new_PointCloud);
//
//	for (int i = 0; i < new_points4D.cols; i++){
//		CloudPoint point;
//
//		point.pt = new_PointCloud[i];
//
//		vector<int> indices (num_tot_im, 0); //Solo se tiene un par de imágenes por lo que habrá que el vector tiene que corresponderse unicamente a estas dos imágenes
//		indices[1] = good_obj_idx[i];
//		indices[2] = good_scene_idx[i];
//		point.index_of_2d_origin = indices;
//
//		todos_los_puntos_en_3D.push_back(point);
//	}
//
//	//cuuuuaaaaatroooooo
//	//Obtención de los keypoints y descriptors de la nube de puntos anterior
//
//		keypoint3d.clear();
//		posicion = 0;
//		for(unsigned int j = 0; j < todos_los_puntos_en_3D.size(); j ++){
//			posicion = todos_los_puntos_en_3D[j].index_of_2d_origin[2];
//			//cout<< v_descriptors[0].row(posicion)<< endl;
//			descriptors3d.push_back(v_descriptors[2].row(posicion));
//			key3d = v_keypoints[2][posicion];
//			keypoint3d.push_back(key3d);
//		}
//
//		//Obtención de los puntos característicos y descriptors a través del matcheado entre la nube de puntos anterior y la imagen actual
//
//		obj_new.clear();
//		scene_new.clear();
//		obj_idx_new.clear();
//		scene_idx_new.clear();
//		obtainMatches(keypoint3d, v_keypoints[3], descriptors3d, v_descriptors[3], obj_new, scene_new, obj_idx_new, scene_idx_new);
//
//		//Obtener R y t de la cámara a través de la PnPRansac
//
//		rvec.release();
//		tvec.release();
//		pnpPointcloud_valid.clear();
//
//		for(unsigned int j = 0; j < obj_idx_new.size(); j ++){
//			punto = todos_los_puntos_en_3D[obj_idx_new[j]].pt;
//			pnpPointcloud_valid.push_back(punto);
//		}
//
//		solvePnPRansac(pnpPointcloud_valid, scene_new, intrinsic, distortion, rvec, tvec, false, 300, 3.0);
//
//		//Convertir de formato Rodrigues a matriz de rotación y construir la matriz de Transformación 3x4
//
//		r_mat.release();
//		Rodrigues(rvec,r_mat);
//		hconcat(R,t,tran);
//
//		//Ahora se obtienen Matches de la imagen 2 con la 3
//
//		obj_more_points.clear();
//		scene_more_points.clear();
//		good_more_scene.clear();
//		good_more_object.clear();
//		obj_idx_more_points.clear();
//		scene_idx_more_points.clear();
//		good_more_scene_idx.clear();
//		good_more_obj_idx.clear();
//
//		obtainMatches(v_keypoints[2], v_keypoints[3], v_descriptors[2], v_descriptors[3], obj_more_points, scene_more_points, obj_idx_more_points, scene_idx_more_points);
//	//	undistortPoints(obj_more_points, obj_more_points, intrinsic, distortion, noArray(), intrinsic);
//	//	undistortPoints(scene_more_points, scene_more_points, intrinsic, distortion, noArray(), intrinsic);
//		E = findEssentialMat(obj_more_points, scene_more_points, focal, pp, RANSAC, 0.99, 3.0, mask);
//	//	correctMatches(E, obj_more_points, scene_more_points, obj_more_points, scene_more_points);
//
//		for (int b = 0; b < (int)obj.size(); b++){
//			if (mask.at<char>(b) == 1){
//				good_more_scene.push_back(scene_more_points[b]);
//				good_more_object.push_back(obj_more_points[b]);
//				good_more_scene_idx.push_back(scene_idx_more_points[b]);
//				good_more_obj_idx.push_back(obj_idx_more_points[b]);
//			}
//		}
//
//		//Triangulamos los puntos
//
//		B = intrinsic*tran;
//
//		new_points4D.release();
//		new_PointCloud.clear();
//		triangulatePoints(A, B, good_more_object, good_more_scene, new_points4D);
//		convertHomogeneous(new_points4D, new_PointCloud);
//
//		for (int i = 0; i < new_points4D.cols; i++){
//			CloudPoint point;
//
//			point.pt = new_PointCloud[i];
//
//			vector<int> indices (num_tot_im, 0); //Solo se tiene un par de imágenes por lo que habrá que el vector tiene que corresponderse unicamente a estas dos imágenes
//			indices[1] = good_obj_idx[i];
//			indices[2] = good_scene_idx[i];
//			point.index_of_2d_origin = indices;
//
//			todos_los_puntos_en_3D.push_back(point);
//		}
//
//
//	myfile << "Dpoints = [";
//	for (int i = 0; i < todos_los_puntos_en_3D.size(); i++){
//		myfile << todos_los_puntos_en_3D[i].pt.x <<", "<< todos_los_puntos_en_3D[i].pt.y <<", "<< todos_los_puntos_en_3D[i].pt.z << endl;
//	}
//	myfile << "]" << endl;
//
//	myfile.close();
//
//	//	waitKey(0);
//	destroyAllWindows();
//
//	return(0);
//}
//
