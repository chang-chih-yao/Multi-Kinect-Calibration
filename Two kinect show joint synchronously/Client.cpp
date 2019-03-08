
// THIS IS CLIENT

#include "stdafx.h"
#define _WINSOCKAPI_
#include <windows.h>
#define _WINSOCK_DEPRECATED_NO_WARNINGS

#include <WinSock2.h>
#include <iostream>
#include <Kinect.h>
#include <string>
#include <WS2tcpip.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#define calibration_frame 10000
#define IP "192.168.60.47"
#define port 1234
#define data_buffer 1024

using namespace std;

// output operator for CameraSpacePoint
ostream& operator<<(ostream& rOS, const CameraSpacePoint& rPos)
{
	rOS << "(" << rPos.X << "/" << rPos.Y << "/" << rPos.Z << ")";
	return rOS;
}

// output operator for Vector4
ostream& operator<<(ostream& rOS, const Vector4& rVec)
{
	rOS << "(" << rVec.x << "/" << rVec.y << "/" << rVec.z << "/" << rVec.w << ")";
	return rOS;
}

int main(int argc, char *argv[])
{
	string confirm;
	char message[200];

	int r;
	WSAData wsaData;
	WORD DLLVersion;
	DLLVersion = MAKEWORD(2, 2);
	r = WSAStartup(DLLVersion, &wsaData);

	SOCKADDR_IN addr;

	int addlen = sizeof(addr);

	addr.sin_addr.s_addr = inet_addr(IP);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);


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
	cout << "Try to get body source" << endl;
	IBodyFrameSource* pFrameSource = nullptr;
	if (pSensor->get_BodyFrameSource(&pFrameSource) != S_OK)
	{
		cerr << "Can't get body frame source" << endl;
		return -1;
	}

	// 2b. Get the number of body
	INT32 iBodyCount = 0;
	if (pFrameSource->get_BodyCount(&iBodyCount) != S_OK)
	{
		cerr << "Can't get body count" << endl;
		return -1;
	}
	cout << " > Can trace " << iBodyCount << " bodies" << endl;
	IBody** aBody = new IBody*[iBodyCount];
	for (int i = 0; i < iBodyCount; ++i)
		aBody[i] = nullptr;

	// 3a. get frame reader
	cout << "Try to get body frame reader" << endl;
	IBodyFrameReader* pFrameReader = nullptr;
	if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
	{
		cerr << "Can't get body frame reader" << endl;
		return -1;
	}

	// 2b. release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;


	Sleep(4000);
	cout << "start!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;

	// Enter main loop
	int iStep = 0;
	while (iStep < calibration_frame)
	{
		// 4a. Get last frame
		IBodyFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4b. get Body data
			if (pFrame->GetAndRefreshBodyData(iBodyCount, aBody) == S_OK)
			{
				int iTrackedBodyCount = 0;
				// 4c. for each body
				for (int i = 0; i < iBodyCount; ++i)
				{
					IBody* pBody = aBody[i];

					// check if is tracked
					BOOLEAN bTracked = false;
					if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
					{
						++iTrackedBodyCount;
						cout << "User " << i << " is under tracking" << endl;

						// get joint position
						Joint aJoints[JointType::JointType_Count];
						if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK)
						{
							const Joint& Hand_R = aJoints[JointType::JointType_ElbowRight];
							const Joint& Th_R = aJoints[JointType::JointType_ShoulderRight];
							const Joint& Hand_L = aJoints[JointType::JointType_SpineShoulder];
							const Joint& Th_L = aJoints[JointType::JointType_ShoulderLeft];
							const Joint& spine = aJoints[JointType::JointType_ElbowLeft];

							++iStep;
							/*cout << "Hand_R is tracked at " << Hand_R.Position << endl;
							cout << "Th_R is tracked at " << Th_R.Position << endl;
							cout << "Hand_L is tracked at " << Hand_L.Position << endl;
							cout << "Th_L is tracked at " << Th_L.Position << endl;
							cout << "spine is tracked at " << spine.Position << endl;*/

							cout << "Step : " << iStep << endl;

							char data[data_buffer] = "";
							int sum = 0;
							int ret1 = _snprintf(data, data_buffer, "%f", Hand_R.Position.X);
							sum += ret1;
							int ret2 = _snprintf(data + sum, data_buffer - sum, "|%f", Hand_R.Position.Y);
							sum += ret2;
							int ret3 = _snprintf(data + sum, data_buffer - sum, "|%f", Hand_R.Position.Z);
							sum += ret3;
							int ret4 = _snprintf(data + sum, data_buffer - sum, "|%f", Th_R.Position.X);
							sum += ret4;
							int ret5 = _snprintf(data + sum, data_buffer - sum, "|%f", Th_R.Position.Y);
							sum += ret5;
							int ret6 = _snprintf(data + sum, data_buffer - sum, "|%f", Th_R.Position.Z);
							sum += ret6;
							int ret7 = _snprintf(data + sum, data_buffer - sum, "|%f", Hand_L.Position.X);
							sum += ret7;
							int ret8 = _snprintf(data + sum, data_buffer - sum, "|%f", Hand_L.Position.Y);
							sum += ret8;
							int ret9 = _snprintf(data + sum, data_buffer - sum, "|%f", Hand_L.Position.Z);
							sum += ret9;
							int ret10 = _snprintf(data + sum, data_buffer - sum, "|%f", Th_L.Position.X);
							sum += ret10;
							int ret11 = _snprintf(data + sum, data_buffer - sum, "|%f", Th_L.Position.Y);
							sum += ret11;
							int ret12 = _snprintf(data + sum, data_buffer - sum, "|%f", Th_L.Position.Z);
							sum += ret12;
							int ret13 = _snprintf(data + sum, data_buffer - sum, "|%f", spine.Position.X);
							sum += ret13;
							int ret14 = _snprintf(data + sum, data_buffer - sum, "|%f", spine.Position.Y);
							sum += ret14;
							int ret15 = _snprintf(data + sum, data_buffer - sum, "|%f", spine.Position.Z);

							cout << data << endl;

							SOCKET sConnect;
							sConnect = socket(AF_INET, SOCK_STREAM, NULL);
							connect(sConnect, (SOCKADDR*)&addr, sizeof(addr));

							/*
							ZeroMemory(message, 200);
							r = recv(sConnect, message, sizeof(message), 0);
							cout << message << endl;
							*/

							send(sConnect, data, (int)strlen(data), 0);
							closesocket(sConnect);

							//getchar();
							//getchar();
						}
						else {
							cerr << "Get joints fail" << endl;
						}
					}
				}

				if (iTrackedBodyCount > 0)
					cout << "Total " << iTrackedBodyCount << " bodies in this time\n" << endl;
			}
			else
			{
				cerr << "Can't read body data" << endl;
			}

			// 4e. release frame
			pFrame->Release();
		}
	}

	// delete body data array
	delete[] aBody;

	// 3b. release frame reader
	cout << "Release frame reader" << endl;
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 1c. Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// 1d. Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;


	system("pause");

	return 0;
}
