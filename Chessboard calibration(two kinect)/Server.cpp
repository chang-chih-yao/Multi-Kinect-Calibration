// ConsoleApplication1.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include "Header.h"
#define _WINSOCKAPI_
#define _WINSOCK_DEPRECATED_NO_WARNINGS
#include <iostream>
#include <WinSock2.h>
#include <string>
#include <Kinect.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

//#include <Eigen\Dense>
#include <array>


using namespace std;
using namespace cv;
//using namespace Eigen;

#define calibration_frame 100
// server IP
#define IP "192.168.60.47"
#define port 1234
#define buffer 1024
#define joint_num 5

Mat Projection(Mat &img, ICoordinateMapper* pCMapper);


int main(int argc, char *argv[])
{
	// 1a. Get default Sensor
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
	{
		cerr << "Get Sensor failed" << endl;
		return -1;
	}

	// 1b. Open sensor
	cout << "Try to open sensor" << endl;
	if (pSensor->Open() != S_OK)
	{
		cerr << "Can't open sensor" << endl;
		return -1;
	}

	// 2a. Get frame source
	cout << "Try to get color source" << endl;
	IColorFrameSource* pFrameSource = nullptr;
	if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
	{
		cerr << "Can't get color frame source" << endl;
		return -1;
	}

	// 2b. Get frame description
	cout << "get color frame description" << endl;
	int		iWidth = 0;
	int		iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
	{
		pFrameDescription->get_Width(&iWidth);
		pFrameDescription->get_Height(&iHeight);
	}
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// 3a. get frame reader
	cout << "Try to get color frame reader" << endl;
	IColorFrameReader* pFrameReader = nullptr;
	if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
	{
		cerr << "Can't get color frame reader" << endl;
		return -1;
	}

	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;

	// Prepare OpenCV data
	cv::Mat	mImg(iHeight, iWidth, CV_8UC4);
	UINT uBufferSize = iHeight * iWidth * 4 * sizeof(BYTE);
	//cv::namedWindow("Color Map");



	//---------------------------socket-----------------------------------//

	int r;
	WSAData wsaData;
	WORD DLLVSERION;
	DLLVSERION = MAKEWORD(2, 1);

	r = WSAStartup(DLLVSERION, &wsaData);

	SOCKADDR_IN addr;
	int addrlen = sizeof(addr);

	//建立 socket
	SOCKET sListen; //listening for an incoming connection
	SOCKET sConnect; //operating if a connection was found

	sConnect = socket(AF_INET, SOCK_STREAM, NULL);

	//設定位址資訊的資料
	addr.sin_addr.s_addr = inet_addr(IP);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);

	//設定 Listen
	sListen = socket(AF_INET, SOCK_STREAM, NULL);
	bind(sListen, (SOCKADDR*)&addr, sizeof(addr));
	listen(sListen, SOMAXCONN);//SOMAXCONN: listening without any limit

	SOCKADDR_IN clinetAddr;
	

	cout << "3" << endl;
	Sleep(1000);
	cout << "2" << endl;
	Sleep(1000);
	cout << "1" << endl;
	Sleep(1000);

	// 4. get CoordinateMapper
	ICoordinateMapper* pCoordinateMapper = nullptr;
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cout << "Can't get coordinate mapper" << endl;
		return -1;
	}


	int cou2 = 0;
	while (true)
	{
		if (cou2 > 10) {
			system("pause");
			break;
		}
		// 4a. Get last frame
		IColorFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. Copy to OpenCV image
			if (pFrame->CopyConvertedFrameDataToArray(uBufferSize, mImg.data, ColorImageFormat_Bgra) == S_OK)
			{
				cout << "get new photo" << endl;
				cou2++;
				//cv::imshow("Color Map", mImg);
				//cv::waitKey(1);
				Mat A = Projection(mImg, pCoordinateMapper);  // A is [R|T] extrinsic matrix 4*4  (server)

				while (true)
				{
					cout << "waiting..." << endl;

					char data[buffer];

					if (sConnect = accept(sListen, (SOCKADDR*)&clinetAddr, &addrlen))
					{
						printf("server: got connection from %s\n", inet_ntoa(addr.sin_addr));

						//傳送訊息給 client 端
						char *sendbuf = "this is server";
						send(sConnect, sendbuf, (int)strlen(sendbuf), 0);


						memset(data, 0, buffer);
						r = recv(sConnect, data, sizeof(data), 0);   // client data.   inverse matrix of B
						//cout << data << endl;

					}

					string str(data);
					string s_temp[16];
					for (int i = 0; i < 16; i++) s_temp[i] = "";

					int temp[16];
					for (int i = 0; i < 16 - 1; i++) temp[i] = 0;
					temp[16 - 1] = str.size();

					int cou = 0;
					for (int i = 0; i < str.size(); i++){
						for (int j = i; j < str.size(); j++){
							if (str[i] == '|'){
								temp[cou] = i;
								i = j + 1;
								cou++;
								break;
							}
						}
					}

					for (int i = 0; i < temp[0]; i++) s_temp[0] += str[i];
					for (int i = 0; i < 16; i++){
						for (int j = temp[i] + 1; j < temp[i + 1]; j++){
							s_temp[i + 1] += str[j];
						}
					}

					double point[16];
					for (int i = 0; i < 16; i++) point[i] = atof(s_temp[i].c_str());
					/*for (int i = 0; i < 16; i++) printf("%lf ", point[i]);
					cout << endl;*/

					Mat B_inverse(4, 4, CV_64F);
					int B_cou = 0;
					for (int i = 0; i < 4; i++){
						for (int j = 0; j < 4; j++){
							B_inverse.at<double>(i, j) = point[B_cou];
							B_cou++;
						}
					}
					cout << "B_inverse:\n" << B_inverse << endl;
					Mat AB_inverse = Matix_multiplication(A, B_inverse);
					cout << "AB_inverse:\n" << AB_inverse << endl;
					cout << "----------------------------------------------\n" << endl;

					//----------------------------------- file saved (transformation matrix)----------------------------------------//
					FILE *fout;
					fout = fopen("extrinsic.txt", "w");
					for (int i = 0; i < 4; i++){
						for (int j = 0; j < 4; j++){
							fprintf(fout, "%lf ", AB_inverse.at<double>(i, j));
						}
					}
					fclose(fout);
					
					// ---------------------------------- read file -----------------------------------------//
					/*FILE *fin2;
					fin2 = fopen("C:\\Users\\Owner\\Documents\\Visual Studio 2013\\Projects\\Project\\Project\\intrinsic.txt", "r");
					double in[9];
					for (int i = 0; i<9; i++){
						fscanf(fin2, "%lf", &in[i]);
					}
					fclose(fin2);

					FILE *fin3;
					fin3 = fopen("C:\\Users\\Owner\\Documents\\Visual Studio 2013\\Projects\\Project\\Project\\distor.txt", "r");
					double dis[5];
					for (int i = 0; i<5; i++){
						fscanf(fin3, "%lf", &dis[i]);
					}
					fclose(fin3);


					cv::Mat rvec = AB_inverse(cv::Rect{ 0, 0, 3, 3 });
					cv::Mat tvec = AB_inverse(cv::Rect{ 3, 0, 1, 3 });

					vector<cv::Point3d> cam1space;
					cam1space.push_back({ 0, 0, 0 });
					cam1space.push_back({ 0.05f, 0, 0 });
					cam1space.push_back({ 0, 0.05f, 0 });
					cam1space.push_back({ 0, 0, 0.05f });

					vector<cv::Point2d> proj;
					cv::Mat intrinsic_Matrix(3, 3, CV_64FC1, in);
					cv::Mat distortion_coeffs(5, 1, CV_64FC1, dis);
					cv::projectPoints(cam1space, rvec, tvec, intrinsic_Matrix, distortion_coeffs, proj);


					cv::line(mImg, proj[0], proj[1], Scalar(0, 0, 255));
					cv::line(mImg, proj[0], proj[2], Scalar(0, 255, 0));
					cv::line(mImg, proj[0], proj[3], Scalar(255, 0, 0));*/
					string s = to_string(cou2);
					cv::putText(mImg, s, Point(80, 80), 0, 2, Scalar(0,0,0), 3);
					imshow("pattern", mImg);
					cvWaitKey(1);
					//system("pause");
					break;
					
				}
				
			}

			else cerr << "Data copy error" << endl;

			pFrame->Release();
		}

		// 4f. check keyboard input
		/*if (cv::waitKey(1000) == VK_ESCAPE){
			cv::imwrite("A.bmp",mImg);
			break;
		}*/
		
	}

	cout << "Release frame reader" << endl;
	pFrameReader->Release();
	pFrameReader = nullptr;

	cout << "close sensor" << endl;
	pSensor->Close();

	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}


Mat Projection(Mat &img, ICoordinateMapper* pCMapper){         // [R|T] extrinsic matrix 4*4

	int board_w, board_h;
	int n_boards;
	float measure = 0.0235;        // square edge length
	Size imageSize;

	board_w = 9;                 // How many cross points of width direction?
	board_h = 6;                 // How many cross points of Height direction?
	n_boards = 1;                // How many board?

	//printf("w=%d h=%d n=%d %lfmm\n", board_w, board_h, n_boards, measure);


	imageSize = Size(img.cols, img.rows);
	Mat gray;
	cvtColor(img, gray, CV_RGB2GRAY);

	vector< Point2f> corners;

	bool sCorner = findChessboardCorners(gray, Size(board_w, board_h), corners);


	vector< Point2f> v_tImgPT;
	vector< Point3f> v_tObjPT;
	if (sCorner)
	{
		//corner point refine
		cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
		drawChessboardCorners(img, Size(board_w, board_h), corners, sCorner);
		if (corners.size() == board_w*board_h)
		{
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
		}
	}

	//cv::circle(img, Point2d(934.694122, 537.066655), 4, Scalar(255, 255, 0), -1);
	//imshow("pattern", img);
	//cvWaitKey();


	//------------------------------------------- file read (intrinsic and distortion) ---------------------------------//
	FILE *fin2;
	fin2 = fopen("C:\\Users\\Owner\\Documents\\Visual Studio 2013\\Projects\\Project\\Project\\intrinsic.txt", "r");
	double in[9];
	for (int i = 0; i<9; i++){
		fscanf(fin2, "%lf", &in[i]);
	}
	fclose(fin2);

	FILE *fin3;
	fin3 = fopen("C:\\Users\\Owner\\Documents\\Visual Studio 2013\\Projects\\Project\\Project\\distor.txt", "r");
	double dis[5];
	for (int i = 0; i<5; i++){
		fscanf(fin3, "%lf", &dis[i]);
	}
	fclose(fin3);

	cv::Mat intrinsic_Matrix(3, 3, CV_64FC1, in);
	cv::Mat distortion_coeffs(5, 1, CV_64FC1, dis);
	Mat rvecs, tvecs;

	cv::solvePnP(v_tObjPT, v_tImgPT, intrinsic_Matrix, distortion_coeffs, rvecs, tvecs);

	//cout << "intrinsic:\n" << intrinsic_Matrix << endl;
	//cout << "distor:\n" << distortion_coeffs << endl;


	Mat rotation(3, 3, CV_64F);
	Rodrigues(rvecs, rotation);
	/*printf("Extrinsic : \n%lf  %lf  %lf  %lf\n", rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), tvecs[0].at<double>(0, 0));
	printf("%lf  %lf  %lf  %lf\n", rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), tvecs[0].at<double>(1, 0));
	printf("%lf  %lf  %lf  %lf\n\n", rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), tvecs[0].at<double>(2, 0));*/

	Mat RT(4, 4, CV_64F);
	for (int i = 0; i < 3; i++){
		for (int j = 0; j < 3; j++){
			RT.at<double>(i, j) = rotation.at<double>(i, j);
		}
	}
	RT.at<double>(0, 3) = tvecs.at<double>(0, 0);
	RT.at<double>(1, 3) = tvecs.at<double>(1, 0);
	RT.at<double>(2, 3) = tvecs.at<double>(2, 0);
	RT.at<double>(3, 0) = 0.0;
	RT.at<double>(3, 1) = 0.0;
	RT.at<double>(3, 2) = 0.0;
	RT.at<double>(3, 3) = 1.0;

	
	//////////////////////////////////////////////////////////////////////////////////////////////////


	vector<Point2f> projectPoint;
	vector<Point2f> proj_point;
	vector<Point3f> obj_point;

	Mat trans(3, 1, CV_64F);
	Mat rotation_v2(3, 1, CV_64F);

	trans.at<double>(0, 0) = 0.0;
	trans.at<double>(1, 0) = 0.0;
	trans.at<double>(2, 0) = 0.0;

	rotation_v2.at<double>(0, 0) = 0.0;
	rotation_v2.at<double>(1, 0) = 0.0;
	rotation_v2.at<double>(2, 0) = 0.0;


	for (int i = 0; i < v_tObjPT.size(); i++){
		Point3f obj_temp;
		obj_temp.x = rotation.at<double>(0, 0) * v_tObjPT[i].x + rotation.at<double>(0, 1) * v_tObjPT[i].y + rotation.at<double>(0, 2) * v_tObjPT[i].z + tvecs.at<double>(0, 0);
		obj_temp.y = rotation.at<double>(1, 0) * v_tObjPT[i].x + rotation.at<double>(1, 1) * v_tObjPT[i].y + rotation.at<double>(1, 2) * v_tObjPT[i].z + tvecs.at<double>(1, 0);
		obj_temp.z = rotation.at<double>(2, 0) * v_tObjPT[i].x + rotation.at<double>(2, 1) * v_tObjPT[i].y + rotation.at<double>(2, 2) * v_tObjPT[i].z + tvecs.at<double>(2, 0);
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


	/*cout << "obj_point:" << endl;
	for (int i = 0; i < objectPoints[0].size(); i++){
		cout << obj_point << endl;
	}*/
	

	//projectPoints(objectPoints[0], rvecs[0], tvecs[0], intrinsic_Matrix, distortion_coeffs, projectPoint);
	//projectPoints(obj_point, rotation_v2, trans, intrinsic_Matrix, distortion_coeffs, projectPoint);


	for (int i = 0; i < board_w * board_h; i++){
		/*printf("projected[%d]: %lf  %lf\n", i, projectPoint[i].x, projectPoint[i].y);
		circle(img_global, Point2d(projectPoint[i].x, projectPoint[i].y), 2, Scalar(255, 255, 0), -1);*/

		//printf("projected[%d]: %lf  %lf\n", i, proj_point[i].x, proj_point[i].y);
		//circle(img, Point2d(proj_point[i].x, proj_point[i].y), 2, Scalar(255, 0, 255), -1);
	}

	//imshow("pattern", img_global);
	//cvWaitKey(1);
	//system("pause");
	return RT;
}