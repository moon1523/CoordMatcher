#ifndef CHARUCOSYNC_HH_
#define CHARUCOSYNC_HH_

#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/eigen.hpp> // this header should be invoked after <Eigen/Sparse>
#include <opencv2/calib3d.hpp>

#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <set>

using namespace std;
using namespace cv;
using namespace Eigen;

enum class BoardType
{
	TEST,
	HYUMC,
};

typedef pair<int,int> MD;

class CharucoSync
{
public:
	CharucoSync(int type, int boardNo);
    virtual ~CharucoSync();
    bool SetParameters(string camParam, string detParam);
    void EstimatePose(const Mat &color);
    void Render();
    void ShowAvgValue(const Mat &color);
    void WriteTransformationData(string file);

    void ClearData();
    void TickSwitch() { getPose = !getPose; }
    void SetScalingFactor(float s);

    void calculate_transformation_matrices_for_detected_boardID(int viewNo);
    void calculate_empty_transformation_matrices_from_averaged_transfrmation_matrices();
    void averaging_stacked_transformation_matrices();

    void weighted_averaging_stacked_transformation_matrices();

private:
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> params;
    Mat camMatrix;
    Mat distCoeffs;
    bool getPose;
    Mat display, display_resize;
    float sf;

    //avg values
    bool isStack;
    int boardType;

    // vector
    int boardNo, markerNo;
	vector<cv::Ptr<cv::aruco::CharucoBoard>> boardVec;
	vector<cv::Ptr<cv::aruco::Dictionary>> dictionaryVec;

	vector<vector<int>> markerIds;
	vector<vector<vector<cv::Point2f>>> markerCorners;
	vector<vector<int>> charucoIds;
	vector<vector<cv::Point2f>> charucoCorners;
	vector<cv::Vec3d> rvec;
	vector<cv::Vec3d> tvec;

	vector<vector<Vector4d>> init10_qVec;
	vector<Vector4d> 		 avg10_qVec;
	vector<vector<Vector4d>> cum_qVec;
	vector<Vec3d> 			 sum_tVec;
	vector<vector<double>> cum_wVec;
	vector<double>         sum_wVec;

	// To reference coordinate
	vector<int> board_frameNo;
	vector<int> detected_boardID_oneView;
	vector<vector<int>> detected_boardID_viewVec;

	map<MD, vector<Eigen::Affine3d>> cum_m2dMap;
	map<MD, Eigen::Affine3d> avg_m2dMap;
	map<MD, vector<double>> cum_m2dwMap;
	map<MD, double> avg_m2dwMap;

	vector<MD> mdVec, emptyVec;



//	map<int, int> dm;
//	map<MD, Eigen::Affine3d> m2dMap;
//	vector<Eigen::Affine3d>  r2d;
//	vector<Eigen::Affine3d> R2D;







};

#endif
