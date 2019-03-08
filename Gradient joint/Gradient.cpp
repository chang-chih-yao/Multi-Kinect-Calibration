#include "stdafx.h"

#include <iostream>
#include <Kinect.h>
#include <time.h>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <array>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

#define buffer 1024
#define joint_num 5

void DrawLine(cv::Mat& rImg, const Joint& rJ1, const Joint& rJ2, ICoordinateMapper* pCMapper)
{
	if (rJ1.TrackingState == TrackingState_NotTracked || rJ2.TrackingState == TrackingState_NotTracked)
		return;

	ColorSpacePoint ptJ1, ptJ2;
	pCMapper->MapCameraPointToColorSpace(rJ1.Position, &ptJ1);
	pCMapper->MapCameraPointToColorSpace(rJ2.Position, &ptJ2);

	cv::line(rImg, cv::Point(ptJ1.X, ptJ1.Y), cv::Point(ptJ2.X, ptJ2.Y), cv::Scalar(0, 0, 255), 5);
}

double My_projection(cv::Mat& rImg, const Joint& rJ1, const Joint& rJ2, cv::Mat intrinsic_Matrix, cv::Mat distortion_coeffs, cv::Point2d fix, cv::Point2d truth = cv::Point2d{ 0, 0 }, double *dedx = nullptr, double *dedy = nullptr){

	vector<cv::Point3d> obj_point;
	vector<cv::Point2d> proj_point;
	vector<cv::Point2d> proj_point2;
	obj_point.push_back(cv::Point3d(rJ1.Position.X + fix.x, -rJ1.Position.Y + fix.y, rJ1.Position.Z));
	obj_point.push_back(cv::Point3d(rJ2.Position.X + fix.x, -rJ2.Position.Y + fix.y, rJ2.Position.Z));

	static cv::Mat tvec = (cv::Mat_<double>(3, 1, CV_64FC1) << 0, 0, 0);
	static cv::Mat rvec = (cv::Mat_<double>(3, 1, CV_64FC1) << 0, 0, 0);
	cv::projectPoints(obj_point, rvec, tvec, intrinsic_Matrix, distortion_coeffs, proj_point);
	obj_point[1] = obj_point[0] + cv::Point3d{ 0, 0.001f, 0 };
	obj_point[0] = obj_point[0] + cv::Point3d{ 0.001f, 0, 0 };
	cv::projectPoints(obj_point, rvec, tvec, intrinsic_Matrix, distortion_coeffs, proj_point2);
	/*
	x = obj_point[i].x / obj_point[i].z;
	y = -1*obj_point[i].y / obj_point[i].z;

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

	cv::Point2f proj_temp;
	proj_temp.x = fx*(x + x * r_distortion + t_distortion_x) + cx;
	proj_temp.y = fy*(y + y * r_distortion + t_distortion_y) + cy;
	proj_point.push_back(proj_temp);
	}*/
	cv::line(rImg, cv::Point(proj_point[0].x, proj_point[0].y), cv::Point(proj_point[1].x, proj_point[1].y), cv::Scalar(0, 255, 0), 5);
	cv::Point2d diff = truth - proj_point[0];
	double error = diff.dot(diff);
	if (dedx) {
		cv::Point2d diff_x = truth - proj_point2[0];
		double error_x = diff_x.ddot(diff_x);
		*dedx = (error_x - error);
	}
	if (dedy) {
		cv::Point2d diff_y = truth - proj_point2[1];
		double error_y = diff_y.ddot(diff_y);
		*dedy = (error_y - error);
	}
	return error;
}


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

	// 2. Color Related code
	IColorFrameReader* pColorFrameReader = nullptr;
	cv::Mat	mColorImg;
	UINT uBufferSize = 0;
	{
		// 2a. Get color frame source
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

		// 2c. get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// 2d. release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;


		// Prepare OpenCV data
		mColorImg = cv::Mat(iHeight, iWidth, CV_8UC4);
		uBufferSize = iHeight * iWidth * 4 * sizeof(BYTE);
	}

	// 3. Body related code
	IBodyFrameReader* pBodyFrameReader = nullptr;
	IBody** aBodyData = nullptr;
	INT32 iBodyCount = 0;
	{
		// 3a. Get frame source
		cout << "Try to get body source" << endl;
		IBodyFrameSource* pFrameSource = nullptr;
		if (pSensor->get_BodyFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get body frame source" << endl;
			return -1;
		}

		// 3b. Get the number of body
		if (pFrameSource->get_BodyCount(&iBodyCount) != S_OK)
		{
			cerr << "Can't get body count" << endl;
			return -1;
		}
		cout << " > Can trace " << iBodyCount << " bodies" << endl;
		aBodyData = new IBody*[iBodyCount];
		for (int i = 0; i < iBodyCount; ++i)
			aBodyData[i] = nullptr;

		// 3c. get frame reader
		cout << "Try to get body frame reader" << endl;
		if (pFrameSource->OpenReader(&pBodyFrameReader) != S_OK)
		{
			cerr << "Can't get body frame reader" << endl;
			return -1;
		}

		// 3d. release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. get CoordinateMapper
	ICoordinateMapper* pCoordinateMapper = nullptr;
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cout << "Can't get coordinate mapper" << endl;
		return -1;
	}


	//------------------------------------------- file read (intrinsic and distortion) ---------------------------------//
	FILE *fin2;
	fin2 = fopen("C:\\Users\\Owner\\Documents\\Visual Studio 2013\\Projects\\Project\\Project\\intrinsic.txt", "r");
	double in[9];
	for (int i = 0; i<9; i++){
		fscanf(fin2, "%lf", &in[i]);
	}
	fclose(fin2);

	FILE *fin3;
	fin3 = fopen("C:\\Users\\User\\Documents\\Visual Studio 2013\\Projects\\Project\\Project\\distor.txt", "r");
	double dis[5];
	for (int i = 0; i<5; i++){
		fscanf(fin3, "%lf", &dis[i]);
	}
	fclose(fin3);

	cv::Mat intrinsic_Matrix(3, 3, CV_64FC1, in);
	cv::Mat distortion_coeffs(5, 1, CV_64FC1, dis);


	Sleep(3000);
	cout << "start!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

	// Enter main loop
	cv::namedWindow("Body Image");
	cv::Point2d fix = { 0, 0 };
	while (true)
	{
		// 4a. Get last frame
		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
			// 4c. Copy to OpenCV image
			if (pColorFrame->CopyConvertedFrameDataToArray(uBufferSize, mColorImg.data, ColorImageFormat_Bgra) != S_OK)
			{
				cerr << "Data copy error" << endl;
			}

			// 4e. release frame
			pColorFrame->Release();
		}
		cv::Mat mImg = mColorImg.clone();

		// 4b. Get body data
		IBodyFrame* pBodyFrame = nullptr;
		if (pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK)
		{
			// 4b. get Body data
			if (pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBodyData) == S_OK)
			{
				// 4c. for each body
				for (int i = 0; i < iBodyCount; ++i)
				{
					IBody* pBody = aBodyData[i];

					// check if is tracked
					BOOLEAN bTracked = false;
					if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
					{
						// get joint position
						Joint aJoints[JointType::JointType_Count];
						if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK)
						{
							/*DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_Neck], aJoints[JointType_Head], pCoordinateMapper);*/

							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], pCoordinateMapper);
							ColorSpacePoint p;
							pCoordinateMapper->MapCameraPointToColorSpace(aJoints[JointType_SpineShoulder].Position, &p);
							double dedx, dedy;
							double error = My_projection(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], intrinsic_Matrix, distortion_coeffs, fix, cv::Point2d{ p.X, p.Y }, &dedx, &dedy);
							if (error > 2) {
								fix.x = fix.x - 0.000001*error*dedx;
								fix.y = fix.y - 0.000001*error*dedy;
							}
							else {
								std::cerr << fix;
								cv::waitKey(0);
								return 1;
							}
							DrawLine(mImg, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], pCoordinateMapper);
							My_projection(mImg, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], intrinsic_Matrix, distortion_coeffs, fix);
							//DrawLine(mImg, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], pCoordinateMapper);
							//DrawLine(mImg, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], pCoordinateMapper);
							//DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], pCoordinateMapper);
							//DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], pCoordinateMapper);
							My_projection(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], intrinsic_Matrix, distortion_coeffs, fix);
							DrawLine(mImg, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], pCoordinateMapper);
							My_projection(mImg, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], intrinsic_Matrix, distortion_coeffs, fix);
							/*DrawLine(mImg, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_WristRight], aJoints[JointType_HandRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], pCoordinateMapper);*/

						}
					}
				}
			}
			else
			{
				cerr << "Can't read body data" << endl;
			}
			pBodyFrame->Release();
		}
		cv::imshow("Body Image", mImg);
		cv::waitKey(15);

		// 4c. check keyboard input
		/*if (cv::waitKey(30) == VK_ESCAPE){
		break;
		}*/
	}

	// 3. delete body data array
	delete[] aBodyData;

	// 3. release frame reader
	cout << "Release body frame reader" << endl;
	pBodyFrameReader->Release();
	pBodyFrameReader = nullptr;

	// 2. release color frame reader
	cout << "Release color frame reader" << endl;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;

	// 1c. Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// 1d. Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
