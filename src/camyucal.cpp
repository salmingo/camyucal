/*
 Name        : camyucal.cpp
 Author      : Xiaomeng Lu
 Version     : 0.1
 Copyright   : Your copyright notice
 Description : do calibration for GWAC-camyu camera
 Note:
 - 更改相机IP、Mask、Gateway
 - 标定偏置电压
 - 保存更改项
 - 软件重启相机
 */

#include <iostream>
#include <string>
#include <stdlib.h>
#include "CameraGYCCD.h"

using namespace std;

int main(int argc, char **argv) {
	if (argc < 2) {
		cout << "Usage:" << endl;
		cout << "\tcamyucal <camera ip> <expected offset>" << endl;
	}
	else {
		string camip = argv[1];
		uint16_t offset(1000);
		int result;
		if (argc >= 3) offset = uint16_t(atoi(argv[2]));

		CameraGYCCD camera(camip.c_str());
		CameraGYCCD::InfoPtr nfcam = camera.GetInformation();

		if (!camera.Connect()) {
			cout << "failed to connect camera: " << nfcam->errmsg << endl;
			return -1;
		}
		cout << "****** prepare to tune bias voltage ******" << endl;
		result = camera.SetADCOffset(offset, stdout);
		if (result == 1) {
			cout << "SUCCESS: no tuning need" << endl;
		}
		else if (result == 2) {
			cout << "FAIL: " << nfcam->errmsg << endl;
		}
		else if (result == 3) {
			cout << "WARN: camera is not ready, could not start tuning" << endl;
		}
		else {
			cout << "SUCCESS: tuning succeed" << endl;
			cout << "software-reboot camera automatically" << endl;
			camera.Reboot();
		}
		camera.Disconnect();
		cout << "****** complete ******" << endl;
		cout << endl;
	}
	return 0;
}
