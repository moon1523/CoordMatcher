#ifndef CHARUCOSYNC_HH_
#define CHARUCOSYNC_HH_

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Sparse>
#include <Eigen/Geometry>

#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
using namespace Eigen;

enum class BoardType
{
	TEST,
	HYUMC,
};

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
    vector<Eigen::Affine3d> CoordToReferenceCoord();
    void WriteCoordToReferenceCoord(vector<Eigen::Affine3d> c2r);

private:
    cv::Ptr<cv::aruco::CharucoBoard> board;
    cv::Ptr<cv::aruco::DetectorParameters> params;
    Mat camMatrix;
    Mat distCoeffs;
    bool getPose;
    Mat display, display_resize;
    float sf;

    //avg values
    int frameNo;
    bool isStack;
    int boardType;

    // vector
    int boardNo, viewNo;
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

	vector<vector<Eigen::Affine3d>> c2rVec;





};

#endif
