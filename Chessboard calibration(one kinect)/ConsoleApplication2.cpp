// ConsoleApplication2.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\opencv.hpp>
#include <Eigen\Dense>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;
using namespace Eigen;

Matrix4d Projection(char *str);
void Projection_multiple();

int main()
{
	Projection_multiple();
	system("PAUSE");
	return 0;
}

void Projection_multiple(){

	//Set input params..
	int board_w, board_h;
	int n_boards;
	float measure = 0.0235;        // 正方形邊長多少公尺(kinect單位都是公尺)
	Size imageSize;

	board_w = 9;                 // How many cross points of width direction?
	board_h = 6;                 // How many cross points of Height direction?
	n_boards = 20;                // How many board?

	printf("w=%d h=%d n=%d %lfmm\n", board_w, board_h, n_boards, measure);


	//extraction image point and object point

	vector< vector< Point2f> > imagePoints;
	vector< vector< Point3f> > objectPoints;

	Mat img_global;
	for (int i = 0; i< n_boards; ++i)
	{
		char str[100];
		memset(str, 0, 100);

		sprintf(str, "%d.bmp", i + 1);
		printf("%s\n", str);
		Mat img = imread(str);
		imageSize = Size(img.cols, img.rows);
		Mat gray;
		cvtColor(img, gray, CV_RGB2GRAY);

		vector< Point2f> corners;

		//find chessboard corners
		bool sCorner = findChessboardCorners(gray, Size(board_w, board_h), corners);

		//if find corner success, then
		if (sCorner)
		{
			//corner point refine
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			//draw corner
			drawChessboardCorners(img, Size(board_w, board_h), corners, sCorner);
			if (corners.size() == board_w*board_h)
			{
				vector< Point2f> v_tImgPT;
				vector< Point3f> v_tObjPT;
				//save 2d coordenate and world coordinate
				for (int j = 0; j< corners.size(); ++j)
				{
					Point2f tImgPT;
					Point3f tObjPT;

					tImgPT.x = corners[j].x;
					tImgPT.y = corners[j].y;

					tObjPT.x = j / board_w * measure;
					tObjPT.y = j % board_w * measure;
					tObjPT.z = 0;

					v_tImgPT.push_back(tImgPT);
					v_tObjPT.push_back(tObjPT);
				}
				imagePoints.push_back(v_tImgPT);
				objectPoints.push_back(v_tObjPT);
			}
		}


		//cv::circle(img, Point2d(934.694122, 537.066655), 4, Scalar(255, 255, 0), -1);
		imshow("pattern", img);
		img_global = img.clone();
		cvWaitKey(1);
		//system("pause");
	}
	



	//////////////////////////////////////////////////////////////////////////////////////////////////
	//claibration part
	vector< Mat> rvecs, tvecs;
	Mat intrinsic_Matrix(3, 3, CV_64F);
	Mat distortion_coeffs(5, 1, CV_64F);

	calibrateCamera(objectPoints, imagePoints, imageSize, intrinsic_Matrix, distortion_coeffs, rvecs, tvecs);

	cout << "intrinsic:\n" << intrinsic_Matrix << endl;
	cout << "distor:\n" << distortion_coeffs << endl;

	FILE *fout1;
	fout1 = fopen("intrinsic.txt", "w");
	for (int i = 0; i < 3; i++){
	for (int j = 0; j < 3; j++){
	fprintf(fout1, "%lf ", intrinsic_Matrix.at<double>(i, j));
	}
	}
	fclose(fout1);

	FILE *fout2;
	fout2 = fopen("distor.txt", "w");
	for (int i = 0; i < 5; i++){
	fprintf(fout2, "%lf ", distortion_coeffs.at<double>(i, 0));
	}
	fclose(fout2);

	Mat rotation(3, 3, CV_64F);
	Rodrigues(rvecs[0], rotation);
	/*printf("Extrinsic : \n%lf  %lf  %lf  %lf\n", rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), tvecs[0].at<double>(0, 0));
	printf("%lf  %lf  %lf  %lf\n", rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), tvecs[0].at<double>(1, 0));
	printf("%lf  %lf  %lf  %lf\n\n", rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), tvecs[0].at<double>(2, 0));
	*/

	vector<Point2f> projectPoint;
	vector<Point2f> proj_point;
	vector<Point3f> obj_point;

	Mat trans(3, 1, CV_64F);
	Mat rotation_v2(3, 1, CV_64F);
	Mat RT(3, 1, CV_64F);


	trans.at<double>(0, 0) = 0.0;
	trans.at<double>(1, 0) = 0.0;
	trans.at<double>(2, 0) = 0.0;

	rotation_v2.at<double>(0, 0) = 0.0;
	rotation_v2.at<double>(1, 0) = 0.0;
	rotation_v2.at<double>(2, 0) = 0.0;


// ---------------- projection code --------------------//

	for (int i = 0; i < objectPoints[0].size(); i++){
		Point3f obj_temp;
		obj_temp.x = rotation.at<double>(0, 0) * objectPoints[0][i].x + rotation.at<double>(0, 1) * objectPoints[0][i].y + rotation.at<double>(0, 2) * objectPoints[0][i].z + tvecs[0].at<double>(0, 0);
		obj_temp.y = rotation.at<double>(1, 0) * objectPoints[0][i].x + rotation.at<double>(1, 1) * objectPoints[0][i].y + rotation.at<double>(1, 2) * objectPoints[0][i].z + tvecs[0].at<double>(1, 0);
		obj_temp.z = rotation.at<double>(2, 0) * objectPoints[0][i].x + rotation.at<double>(2, 1) * objectPoints[0][i].y + rotation.at<double>(2, 2) * objectPoints[0][i].z + tvecs[0].at<double>(2, 0);
		obj_point.push_back(obj_temp);

		double x, y;
		x = obj_temp.x / obj_temp.z;
		y = obj_temp.y / obj_temp.z;

		double fx = intrinsic_Matrix.at<double>(0, 0);
		double fy = intrinsic_Matrix.at<double>(1, 1);
		double cx = intrinsic_Matrix.at<double>(0, 2);
		double cy = intrinsic_Matrix.at<double>(1, 2);

		double k1 = distortion_coeffs.at<double>(0);
		double k2 = distortion_coeffs.at<double>(1);
		double k3 = distortion_coeffs.at<double>(4);
		double p1 = distortion_coeffs.at<double>(2);
		double p2 = distortion_coeffs.at<double>(3);

		double r2 = x*x + y*y;
		double r_distortion = k1*r2 + k2*r2*r2 + r2*r2*r2*k3;
		double t_distortion_x = 2 * p1*x*y + p2*(r2 + 2 * x*x);
		double t_distortion_y = p1*(r2 + 2 * y*y) + 2 * p2*x*y;

		Point2f proj_temp;
		proj_temp.x = fx*(x + x * r_distortion + t_distortion_x) + cx;
		proj_temp.y = fy*(y + y * r_distortion + t_distortion_y) + cy;
		proj_point.push_back(proj_temp);
	}


	//projectPoints(objectPoints[0], rvecs[0], tvecs[0], intrinsic_Matrix, distortion_coeffs, projectPoint);
	projectPoints(obj_point, rotation_v2, trans, intrinsic_Matrix, distortion_coeffs, projectPoint);


	/*fprintMatrix(intrinsic_Matrix, "intrinsic.txt");
	fprintMatrix(distortion_coeffs, "distortion_coeffs.txt");

	fprintfVectorMat(rvecs, "rotation.txt");
	fprintfVectorMat(tvecs, "translation.txt");

	fprintf3Point(objectPoints, "objectpt.txt");
	fprintf2Point(imagePoints, "imagept.txt");*/

	for (int i = 0; i < 54; i++){
		/*printf("projected[%d]: %lf  %lf\n", i, projectPoint[i].x, projectPoint[i].y);
		circle(img_global, Point2d(projectPoint[i].x, projectPoint[i].y), 2, Scalar(255, 255, 0), -1);*/

		//printf("projected[%d]: %lf  %lf\n", i, proj_point[i].x, proj_point[i].y);
		circle(img_global, Point2d(proj_point[i].x, proj_point[i].y), 2, Scalar(255, 0, 255), -1);
	}

	//imshow("pattern", img_global);
	//cvWaitKey(1);
}





