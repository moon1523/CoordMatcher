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
	
	if ( argc > 1 ) {
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
    CharucoSync sync((int)BoardType::TEST, 2);
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
        else if (key == 'w') {
        	sync.WriteTransformationData("CoordData");
        }
        else if (key == 'e') {

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


// #include <Eigen/Dense>
// #include <iostream>
// #include <memory>

// #include "open3d/Open3D.h"

// using namespace open3d;

// std::tuple<std::shared_ptr<geometry::PointCloud>,
//            std::shared_ptr<geometry::PointCloud>,
//            std::shared_ptr<pipelines::registration::Feature>>
// PreprocessPointCloud(const char *file_name, const float voxel_size) {
//     auto pcd = open3d::io::CreatePointCloudFromFile(file_name);
//     auto pcd_down = pcd->VoxelDownSample(voxel_size);
//     pcd_down->EstimateNormals(
//             open3d::geometry::KDTreeSearchParamHybrid(2 * voxel_size, 30));
//     auto pcd_fpfh = pipelines::registration::ComputeFPFHFeature(
//             *pcd_down,
//             open3d::geometry::KDTreeSearchParamHybrid(5 * voxel_size, 100));
//     return std::make_tuple(pcd, pcd_down, pcd_fpfh);
// }

// void VisualizeRegistration(const open3d::geometry::PointCloud &source,
//                            const open3d::geometry::PointCloud &target,
//                            const Eigen::Matrix4d &Transformation) {
//     std::shared_ptr<geometry::PointCloud> source_transformed_ptr(
//             new geometry::PointCloud);
//     std::shared_ptr<geometry::PointCloud> target_ptr(new geometry::PointCloud);
//     *source_transformed_ptr = source;
//     *target_ptr = target;
//     source_transformed_ptr->Transform(Transformation);
//     visualization::DrawGeometries({source_transformed_ptr, target_ptr},
//                                   "Registration result");
// }

// void PrintHelp() {
//     using namespace open3d;

//     PrintOpen3DVersion();
//     // clang-format off
//     utility::LogInfo("Usage:");
//     utility::LogInfo("    > RegistrationFGR source_pcd target_pcd"
//                      "[--voxel_size=0.05] [--distance_multiplier=1.5]"
//                      "[--max_iterations=64] [--max_tuples=1000]"
//                      );
//     // clang-format on
// }


// int FGR(int argc, char** argv) {
//     using namespace open3d;

//     utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

//     if (argc < 3 ||
//         utility::ProgramOptionExistsAny(argc, argv, {"-h", "--help"})) {
//         PrintHelp();
//         return 1;
//     }

//     float voxel_size = utility::GetProgramOptionAsDouble(argc, argv, "--voxel_size", 0.05);
//     float distance_multiplier = utility::GetProgramOptionAsDouble(argc, argv, "--distance_multiplier", 1.5);
//     float distance_threshold = voxel_size * distance_multiplier;
//     int max_iterations = utility::GetProgramOptionAsInt(argc, argv, "--max_iterations", 64);
//     int max_tuples = utility::GetProgramOptionAsInt(argc, argv, "--max_tuples", 1000);

//     // Prepare input
//     std::shared_ptr<geometry::PointCloud> source, source_down, target,
//             target_down;
//     std::shared_ptr<pipelines::registration::Feature> source_fpfh, target_fpfh;
//     std::tie(source, source_down, source_fpfh) = PreprocessPointCloud(argv[1], voxel_size);
//     std::tie(target, target_down, target_fpfh) = PreprocessPointCloud(argv[2], voxel_size);

//     pipelines::registration::RegistrationResult registration_result =
//             pipelines::registration::
//                     FastGlobalRegistrationBasedOnFeatureMatching(
//                             *source_down, *target_down, *source_fpfh,
//                             *target_fpfh,
//                             pipelines::registration::
//                                     FastGlobalRegistrationOption(
//                                             /* decrease_mu =  */ 1.4, true,
//                                             true, distance_threshold,
//                                             max_iterations,
//                                             /* tuple_scale =  */ 0.95,
//                                             max_tuples));

//     // pipelines::registration::RegistrationResult registration_result;
//     VisualizeRegistration(*source, *target,
//                           registration_result.transformation_);

//     return 0;
// }
