#include "CharucoSync.hh"

Rect cropRect;
bool clicked;
Point2i P1, P2;
float sfInv;
void onMouseCropImage(int event, int x, int y, int f, void *param);

Eigen::Vector4d quaternionAverage(std::vector<Eigen::Vector4d> quaternions);

CharucoSync::CharucoSync(int type, int _boardNo)
    :boardType(type), getPose(false), sf(1), frameNo(0), isStack(false), boardNo(_boardNo), viewNo(0)
{
	for (size_t i=0; i<boardNo; i++) {
    	dictionaryVec.push_back(cv::aruco::generateCustomDictionary(17, 4, i));
    }

	switch((BoardType)boardType)
	{
	case BoardType::TEST:
		for (size_t i=0; i<boardNo; i++) {
			boardVec.push_back(cv::aruco::CharucoBoard::create(5, 7, 0.0290f, 0.0230f, dictionaryVec[i]));
		}
		break;
	case BoardType::HYUMC:
		for (size_t i=0; i<boardNo; i++) {
			boardVec.push_back(cv::aruco::CharucoBoard::create(5, 7, 0.0290f, 0.0230f, dictionaryVec[i]));
		}
		break;
	}

    params = cv::aruco::DetectorParameters::create();

    markerIds.resize(boardNo);
	markerCorners.resize(boardNo);
	charucoIds.resize(boardNo);
	charucoCorners.resize(boardNo);
	rvec.resize(boardNo);
	tvec.resize(boardNo);
	init10_qVec.resize(boardNo);
	avg10_qVec.resize(boardNo);
	cum_qVec.resize(boardNo);
	sum_tVec.resize(boardNo);
}

void CharucoSync::ClearData()
{
	for (int i=0; i<boardNo; i++) {
		markerIds[i].clear();
		markerCorners[i].clear();
		charucoIds[i].clear();
		charucoCorners[i].clear();
		init10_qVec[i].clear();
		cum_qVec[i].clear();
	}
	rvec.clear();
	tvec.clear();
	avg10_qVec.clear();
	sum_tVec.clear();
}

CharucoSync::~CharucoSync()
{
}

bool CharucoSync::SetParameters(string camParm, string detParam)
{
    cv::FileStorage fs(camParm, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    FileStorage fs2(detParam, FileStorage::READ);
    if (!fs2.isOpened())
        return false;
    fs2["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs2["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs2["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs2["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs2["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs2["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs2["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs2["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs2["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs2["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs2["cornerRefinementMethod"] >> params->cornerRefinementMethod;
    fs2["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs2["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs2["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs2["markerBorderBits"] >> params->markerBorderBits;
    fs2["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs2["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs2["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs2["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs2["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

void CharucoSync::EstimatePose(const Mat &color)
{
    color.copyTo(display);

    Mat cropImg;
    if (cropRect.width > 0 && cropRect.height > 0)
        cropImg = color(cropRect).clone();
    else
        color.copyTo(cropImg);

    for(size_t i=0; i< boardNo; i++)
    {
    	cv::aruco::detectMarkers(cropImg, boardVec[i]->dictionary, markerCorners[i], markerIds[i], params);
    	if (markerIds[i].size() > 0)
    	{
    		std::for_each(markerCorners[i].begin(), markerCorners[i].end(), [](vector<Point2f> &vec)
			{
				for (Point2f &point : vec)
					point += Point2f(cropRect.tl());
			});
    		cv::aruco::drawDetectedMarkers(display, markerCorners[i], markerIds[i]);
    		cv::aruco::interpolateCornersCharuco(markerCorners[i], markerIds[i], color, boardVec[i], charucoCorners[i], charucoIds[i], camMatrix, distCoeffs);

    		// if at least one charuco corner detected
    		if (charucoIds[i].size() > 0)
    		{
    			// cv::aruco::drawDetectedCornersCharuco(display, charucoCorners[i], charucoIds[i]);
    			bool valid = cv::aruco::estimatePoseCharucoBoard(charucoCorners[i], charucoIds[i], boardVec[i], camMatrix, distCoeffs, rvec[i], tvec[i]);

    			// if charuco pose is valid
    			if (valid) {
    				cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec[i], tvec[i], 0.1f);
    			}

    			if (getPose)
    			{
    				double angle = norm(rvec[i]);
    				Vector3d axis(rvec[i](0) / angle, rvec[i](1) / angle, rvec[i](2) / angle);
    				Quaterniond q(AngleAxisd(angle, axis));
    				q.normalize();

    				// Simple filtering: delete outlier coordinate
    				if (isnan(q.x())) {
    					cout << "Filtering the frame for "+ to_string(i) +"'th coordinate was not correctly estimated" << endl;
    				}
    				else if (frameNo < 10)
    				{
    					init10_qVec[i].push_back(Vector4d(q.x(), q.y(), q.z(), q.w()));
    					if (i == boardNo - 1) frameNo++;
    				}
    				else if (frameNo == 10)
    				{
    					avg10_qVec[i] = quaternionAverage(init10_qVec[i]);
    					if (i == boardNo - 1) frameNo++;
    				}
    				else
    				{
    					Quaterniond quat0 = Quaterniond(avg10_qVec[i].w(), avg10_qVec[i].x(), avg10_qVec[i].y(), avg10_qVec[i].z());
                    	Quaterniond quat1 = Quaterniond(q.w(), q.x(), q.y(), q.z());

                    	Vector3d axisX0 = quat0.matrix() * Vector3d::UnitX();
    					Vector3d axisY0 = quat0.matrix() * Vector3d::UnitY();
    					Vector3d axisZ0 = quat0.matrix() * Vector3d::UnitZ();
    					Vector3d axisX1 = quat1.matrix() * Vector3d::UnitX();
                        Vector3d axisY1 = quat1.matrix() * Vector3d::UnitY();
                        Vector3d axisZ1 = quat1.matrix() * Vector3d::UnitZ();

                        double dotX = axisX0.dot(axisX1);
    					double dotY = axisY0.dot(axisY1);
    					double dotZ = axisZ0.dot(axisZ1);

                        if (dotX > 0 && dotY > 0 && dotZ > 0) {
                        	if (i == boardNo - 1) frameNo++;
    						isStack = true;
                        }
    				}

    				if (isStack)
					{
    					cum_qVec[i].push_back(Vector4d(q.x(), q.y(), q.z(), q.w()));
    					sum_tVec[i] += tvec[i];
						isStack = false;
					} // if (isStack)
    			} // if (getPose)
    		} // if (markerIds[i].size() > 0)
    	} // if (charucoIds[i].size() > 0)

    	if ( i != boardNo -1 ) continue;
    	bool sizeChk(false);
    	for (int i=0;i<boardNo; i++) {
    		if (cum_qVec[i].size() > 100)	sizeChk = true;
    		else 							sizeChk = false;
    	}
    	if (sizeChk)
		{
    		frameNo = 0;
			c2rVec.push_back(CoordToReferenceCoord());
			ClearData();
			getPose = false;
			cout << "=== INFORMATION ==============================================================" << endl;
			cout << "Coordinates data acquisition for " << ++viewNo << "'th was completed," << endl;
			cout << "Take coordinates from different viewpoint (viewpoint data: "<< c2rVec.size() << ")" << endl;
			cout << "Press 'e' key to averaging coordinate data" << endl;
			cout << "============================================================== INFORMATION ===" << endl;
		} // if (sizeChk)
    } // for(size_t i=0; i< boardNo; i++)
}

void CharucoSync::Render()
{
    setMouseCallback("Synchronization", onMouseCropImage);
    if (clicked)
    	cv::rectangle(display, P1, P2, CV_RGB(255, 255, 0), 3);
    else if (cropRect.width > 0){
        imshow("crop img.", display(cropRect));
        cv::rectangle(display, cropRect, CV_RGB(255, 255, 0), 3);
    }

    resize(display, display, Size(display.cols * sf, display.rows * sf));

    for (int i=0; i<boardNo; i++) {
    	putText(display, "number of data for " + to_string(i) + "'th coordinate " + to_string(cum_qVec[i].size()),
    			Point(10, 30*(i+1)), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255.f,0.f,0.f), 1.5);
    }

    if(getPose) 
        putText(display, "obtaining pose data..", Point(display.cols - 250, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255.f,0.f,0.f), 1);
    imshow("Synchronization", display);
}

#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
void CharucoSync::ShowAvgValue(const Mat &color)
{
	for (int i=0; i<boardNo; i++) {
		Vec3d rvec;
		AngleAxisd avg(Quaterniond((quaternionAverage(cum_qVec[i]))));
		eigen2cv(avg.axis(), rvec);
		rvec *= avg.angle();
		Vec3d tvec_avg = sum_tVec[i] / (double)cum_qVec[0].size();
		color.copyTo(display);
		cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec, tvec_avg, 0.1f);
		resize(display, display, Size(display.cols * sf, display.rows * sf));
	}
    imshow("Synchronization", display);
}

void CharucoSync::WriteTransformationData(string fileName){
	cout << "'" << fileName << "' was generated." << endl;
    ofstream ofs(fileName);
    ofs << "Reference_Coordinate(tvec,cm)" << endl;
//    for (int i=0; i<boardNo;i ++)
//    {
//    	Vector4d q = quaternionAverage(cum_qVec[i]);
//    	Vec3d t = sum_tVec[i] * 100 / (double)cum_qVec[i].size();
//    	ofs<<"q "<< q(0) <<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<endl;
//    	ofs<<"t "<<t(0)*100<<" "<<t(1)*100<<" "<<t(2)*100<<endl;
//    }
	Vector4d q = quaternionAverage(cum_qVec[0]);
	Vec3d t = sum_tVec[0] / (double)cum_qVec[0].size() * 100; // mm -> cm
	ofs<<"q "<< q(0) <<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<endl;
	ofs<<"t "<<t(0) <<" "<<t(1) <<" "<<t(2) <<endl;
	time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());

	ofs << endl;
	vector<Eigen::Affine3d> c2r = CoordToReferenceCoord();
	ofs << "Transformation_matrix_to_Ref_Coord" << endl;
//	for (auto itr: c2r) {
//		ofs << itr <<
//	}



	ofs<<ctime(&now)<<endl;
	ofs<<endl;
    ofs.close();


}

void CharucoSync::SetScalingFactor(float s)
{
    sf = s;
    sfInv = 1 / s;
}

vector<Eigen::Affine3d> CharucoSync::CoordToReferenceCoord()
{
	vector<Eigen::Affine3d> aVec, c2r;
	for (int i=0;i<boardNo;i++) {
		Eigen::Affine3d a;
		Vec3d tt = sum_tVec[i] / (double)cum_qVec[i].size() * 100;
		Vector3d t(tt.val[0], tt.val[1], tt.val[2]);
		a.translation() = t;
		a.linear() = Quaterniond(quaternionAverage(cum_qVec[i])).matrix();
		aVec.push_back(a);
	}

	for (int i=1;i<boardNo;i++) {
		c2r.push_back(aVec[i-1] * aVec[i].inverse());
	}

	cout << c2r[0].matrix() << endl << endl;
	cout << c2r[0].translation() << endl;
	cout << c2r[0].rotation() << endl;
	cout << endl;
	Quaterniond q(c2r[0].rotation());
	cout << q.matrix() << endl << endl;
	cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;

	Vector4d qv(q.x(), q.y(), q.z(), q.w());
	cout << qv.transpose() << endl;

	return c2r;
}

void CharucoSync::WriteCoordToReferenceCoord(vector<Eigen::Affine3d> c2r)
{
	for (size_t i=0; i<c2r.size(); i++)
	{



	}




}


Eigen::Vector4d quaternionAverage(std::vector<Eigen::Vector4d> quaternions)
{
    if (quaternions.size() == 0)
    {
        std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
        return Eigen::Vector4d::Zero();
    }

    // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    for (int q = 0; q < quaternions.size(); ++q)
        A += quaternions[q] * quaternions[q].transpose();

    // normalise with the number of quaternions
    A /= quaternions.size();

    // Compute the SVD of this 4x4 matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

    Eigen::VectorXd singularValues = svd.singularValues();
    Eigen::MatrixXd U = svd.matrixU();

    // find the eigen vector corresponding to the largest eigen value
    int largestEigenValueIndex;
    float largestEigenValue;
    bool first = true;

    for (int i = 0; i < singularValues.rows(); ++i)
    {
        if (first)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
            first = false;
        }
        else if (singularValues(i) > largestEigenValue)
        {
            largestEigenValue = singularValues(i);
            largestEigenValueIndex = i;
        }
    }

    Eigen::Vector4d average;
    average(0) = -U(0, largestEigenValueIndex);
    average(1) = -U(1, largestEigenValueIndex);
    average(2) = -U(2, largestEigenValueIndex);
    average(3) = -U(3, largestEigenValueIndex);

    return average;
}






void onMouseCropImage(int event, int x, int y, int f, void *param)
{
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
        clicked = true;
        P1.x = x * sfInv;
        P1.y = y * sfInv;
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        break;
    case cv::EVENT_LBUTTONUP:
        P2.x = x * sfInv;
        P2.y = y * sfInv;
        clicked = false;
        break;
    case cv::EVENT_MOUSEMOVE:
        if (clicked)
        {
            P2.x = x * sfInv;
            P2.y = y * sfInv;
        }
        break;
    case cv::EVENT_RBUTTONUP:
        clicked = false;
        P1.x = 0;
        P1.y = 0;
        P2.x = 0;
        P2.y = 0;
        break;
    default:
        break;
    }

    if (clicked)
    {
        if (P1.x > P2.x)
        {
            cropRect.x = P2.x;
            cropRect.width = P1.x - P2.x;
        }
        else
        {
            cropRect.x = P1.x;
            cropRect.width = P2.x - P1.x;
        }

        if (P1.y > P2.y)
        {
            cropRect.y = P2.y;
            cropRect.height = P1.y = P2.y;
        }
        else
        {
            cropRect.y = P1.y;
            cropRect.height = P2.y - P1.y;
        }
    }
}



