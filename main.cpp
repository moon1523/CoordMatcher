#include "Functions.hh"
#include "CharucoSync.hh"

#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>

using namespace std;

int main(int argc, char** argv) {
	k4a_color_resolution_t color_resolution;
	double scalingFactor;
	if ( argc > 1 && string(argv[1]) == "-sync") {
		cout << "sync" << endl;
		color_resolution = K4A_COLOR_RESOLUTION_3072P;
		scalingFactor = 0.3;
	}
	else {
		color_resolution = K4A_COLOR_RESOLUTION_720P;
		scalingFactor = 1.0;
	}
		
    // Azure Kinect
    //
    // Start camera
	k4a_device_t device = nullptr;
	VERIFY(k4a_device_open(K4A_DEVICE_DEFAULT, &device), "Open K4A Device failed!");

	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	deviceConfig.color_resolution = color_resolution;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_15;
	deviceConfig.subordinate_delay_off_master_usec = 0;
	deviceConfig.synchronized_images_only = true;
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	// Get calibration information
	k4a_calibration_t sensorCalibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration), "Get depth camera calibration failed!");

    k4a_capture_t sensorCapture = NULL;
	k4a_image_t color_image = NULL;
	k4a_image_t depth_image = NULL;
	k4a_image_t point_image = NULL;
	k4a_image_t colorlike_depth_image = NULL;
	k4a_transformation_t transformation = NULL;
	transformation = k4a_transformation_create(&sensorCalibration);

    int image_width  = sensorCalibration.color_camera_calibration.resolution_width;
	int image_height = sensorCalibration.color_camera_calibration.resolution_height;

    // ChArUco board
    string detParm("detector_params.yml");
	string camParm("kinect3072.yml");
    CharucoSync sync((int)BoardType::TEST, 4);
    sync.SetParameters(camParm, detParm);
    sync.SetScalingFactor(scalingFactor);

    int frameNo(0);
    while(true)
    {
        // 1. Capture image
		k4a_device_get_capture(device, &sensorCapture, 1000);
        color_image = k4a_capture_get_color_image(sensorCapture);
		depth_image = k4a_capture_get_depth_image(sensorCapture);
		k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, image_width, image_height, image_width * 3 * (int)sizeof(uint16_t), &point_image);
		k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, image_width, image_height, image_width * (int)sizeof(uint16_t), &colorlike_depth_image);
		k4a_transformation_depth_image_to_color_camera(transformation, depth_image, colorlike_depth_image);
		k4a_transformation_depth_image_to_point_cloud(transformation, colorlike_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_image);

        cv::Mat color_mat = color_to_opencv(color_image);
        cv::Vec3d rvec, tvec;
        sync.EstimatePose(color_mat);
        sync.Render();

        char key = (char)cv::waitKey(1);
        if (key == 'g') {
			cout << frameNo << endl;
			WritePointCloud(point_image, color_image, to_string(frameNo));
			WriteTrasnformedPointCloud(point_image, color_image, to_string(frameNo) + "_tf");
		}
        else if (key == 'c')
			sync.ClearData();
		else if (key == 't')
			sync.TickSwitch();
        else if (key == 'q')
            break;
        else if (key == 'a')
        {
			sync.ShowAvgValue(color_mat);
			char key2 = waitKey(0);
			if(key2=='q') break;
        }
        else if (key == 'e') {
        	cout << frameNo << endl;
        	WriteTrasnformedPointCloudToRefCoord(point_image, color_image, to_string(frameNo) + "_chk");
        }


        k4a_capture_release(sensorCapture);
        k4a_image_release(color_image);
		k4a_image_release(depth_image);
		k4a_image_release(colorlike_depth_image);
		k4a_image_release(point_image);
        frameNo++;
	}
//	sync.WriteTransformationData("world_coord");
    k4a_transformation_destroy(transformation);
	k4a_device_close(device);
    cout << "k4a device was succesfully closed" << endl;
	
	return EXIT_SUCCESS;
}









