#include "CharucoSync.hh"

Rect cropRect;
bool clicked;
Point2i P1, P2;
float sfInv;
void onMouseCropImage(int event, int x, int y, int f, void *param);

Eigen::Vector4d averaging_quaternions(std::vector<Eigen::Vector4d> quaternions);
Eigen::Vector4d weighted_averaging_quaternions(
  const std::vector<Eigen::Vector4d>& quaternions,
  const std::vector<double>& weights);
double averaging_weights(std::vector<double> &weights);

CharucoSync::CharucoSync(int type, int _boardNo)
    :boardType(type), getPose(false), sf(1), isStack(false), boardNo(_boardNo), markerNo(17)
{
	Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_4X4_1000));
	cv::aruco::Dictionary _dictionary = *dictionary;

	for (int i=0; i<boardNo; i++) {
		Ptr<cv::aruco::Dictionary> customDictionary = makePtr<cv::aruco::Dictionary>(
		_dictionary.bytesList.rowRange(i*17, (i+1)*17).clone(),
		_dictionary.markerSize,
		_dictionary.maxCorrectionBits);
		dictionaryVec.push_back(customDictionary);
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
	cum_wVec.resize(boardNo);
	sum_wVec.resize(boardNo);

	board_frameNo.resize(boardNo);
	for (auto &itr:board_frameNo) {
		itr = 0;
	}


	// Test
//	vector<Vector4d> quaternions;
//	quaternions.push_back(Vector4d(-0.893526, 0.00679962, -0.0208888, -0.448473));
//	quaternions.push_back(Vector4d(0.0033933, -0.893018, 0.449876, -0.0109403));
//	vector<double> weights;
//	weights.push_back(0.5);
//	weights.push_back(0.5);
//	Vector4d avgQuat = averaging_quaternions(quaternions);
//	Vector4d wavgQuat = weighted_averaging_quaternions(quaternions, weights);
//
//	cout << "avg :" <<  avgQuat.transpose() << endl;
//	cout << "wavg:" <<  wavgQuat.transpose() << endl;
//	exit(1);
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
		cum_wVec[i].clear();

		rvec[i] = Vec3d(0,0,0);
		tvec[i] = Vec3d(0,0,0);
		avg10_qVec[i] = Vector4d::Zero();
		sum_tVec[i] = Vec3d(0,0,0);
		sum_wVec[i] = 0;
		detected_boardID_oneView.clear();
		board_frameNo[i] = 0;
	}
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

    bool isTFcal(false);
    for(int i=0; i< boardNo; i++)
    {
    	cv::aruco::detectMarkers(cropImg, boardVec[i]->dictionary, markerCorners[i], markerIds[i], params);

    	if (markerIds[i].size() <= 0)
    		continue;

		std::for_each(markerCorners[i].begin(), markerCorners[i].end(),
				[](vector<Point2f> &vec) {
				for (Point2f &point : vec)
					point += Point2f(cropRect.tl()); // Set top left
				}
		);
    	cv::aruco::drawDetectedMarkers(display, markerCorners[i], markerIds[i]);
		cv::aruco::interpolateCornersCharuco(markerCorners[i], markerIds[i], color, boardVec[i], charucoCorners[i], charucoIds[i], camMatrix, distCoeffs);

		// if at least one charuco corner detected
		if (charucoIds[i].size() > 0)
		{
			// cv::aruco::drawDetectedCornersCharuco(display, charucoCorners[i], charucoIds[i]);
			bool isDraw = cv::aruco::estimatePoseCharucoBoard(charucoCorners[i], charucoIds[i], boardVec[i], camMatrix, distCoeffs, rvec[i], tvec[i]);
			if (isDraw) {
				cv::aruco::drawAxis(display, camMatrix, distCoeffs, rvec[i], tvec[i], 0.1f);
			}

			if (getPose)
			{
				double angle = norm(rvec[i]);
				Vector3d axis(rvec[i](0) / angle, rvec[i](1) / angle, rvec[i](2) / angle);
				Quaterniond q(AngleAxisd(angle, axis));
				q.normalize();

				// Simple filtering: delete outlier pose
				if ( isnan(q.x()) || isnan(q.y()) || isnan(q.z()) || isnan(q.w()) ) {
					cout << "Filtering the frame for "+ to_string(i) +"'th coordinate was not correctly estimated" << endl;
				}
				else if (board_frameNo[i] < 10)
				{
					init10_qVec[i].push_back(Vector4d(q.x(), q.y(), q.z(), q.w()));
					board_frameNo[i]++;
				}
				else if (board_frameNo[i] == 10)
				{
					avg10_qVec[i] = averaging_quaternions(init10_qVec[i]);
					board_frameNo[i]++;
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
						//&& acos(axisZ1.dot(-Vector3d::UnitZ())) * 180/M_PI < 60
						board_frameNo[i]++;
						isStack = true;
					}
				} // filtering end

				if (isStack)
				{
					cum_qVec[i].push_back(Vector4d(q.x(), q.y(), q.z(), q.w()));
					sum_tVec[i] += tvec[i];
					double w1 = markerIds[i].size() / (double)markerNo;
					Vector3d axisZ = q.matrix() * Vector3d::UnitZ();
					double w2 = axisZ.dot(-Vector3d::UnitZ());
					double w = w1 * w2;
					cum_wVec[i].push_back(w);
					sum_wVec[i] += w;
					isStack = false;
				} // if (isStack) end
			} // if (getPose) end
    	} // if (charucoIds[i].size() > 0) end


		// if the number of cumulative pose data of i'th board was larger than 10,
    	// the status of i'th board was set to "detected".
		if (cum_qVec[i].size() == 5) {
			detected_boardID_oneView.push_back(i);
			// cout << "Detected coordinate ID: " << i << endl;
		}

		// if the number of cumulative pose data of all i'th detected board was larger than 50,
		// the switch for calculating transformation matrix was on.
		for (auto i:detected_boardID_oneView) {
			if (cum_qVec[i].size() < 10) {
				isTFcal = false;
				break;
			}
			else {
				isTFcal = true;
			}
		}
    } // for(size_t i=0; i< boardNo; i++) end
    if (!isTFcal) return;

    if (detected_boardID_oneView.size() < 2) {
		cout << "you should shoot with at least two boards as vertical as possible." << endl;
		ClearData();
		getPose = false;
		return;
	}

	sort(detected_boardID_oneView.begin(), detected_boardID_oneView.end());
	detected_boardID_viewVec.push_back(detected_boardID_oneView);
	int viewID = detected_boardID_viewVec.size()-1;

	cout << "=== View " << viewID << " =======================================================" << endl;
	for (size_t i=0; i<detected_boardID_viewVec.size(); i++) {
		cout << "  Detected boardID for " << i << "'th viewpoint: ";
		for (size_t j=0; j<detected_boardID_viewVec[i].size(); j++) {
			cout << detected_boardID_viewVec[i][j] << " ";
		}
		cout << endl;
	}
	WriteTransformationData("view" +to_string(viewID) + "_CoordData");
	calculate_transformation_matrices_for_detected_boardID(viewID);

	ClearData();
	getPose = false;
}



void CharucoSync::calculate_transformation_matrices_for_detected_boardID(int viewID)
{
	map<int,Eigen::Affine3d> aMap;
	map<int, double> wMap;

	// Set the 6D pose (T_i) of the board viewed from the camera coordinate system
	for (auto i: detected_boardID_viewVec[viewID]) {
		Eigen::Affine3d a;
		Vec3d avgt = sum_tVec[i] / (double)cum_qVec[i].size() * 100;
		Vector3d t(avgt.val[0], avgt.val[1], avgt.val[2]);
		a.translation() = t;
//		a.linear() = Quaterniond(averaging_quaternions(cum_qVec[i])).normalized().matrix();
		a.linear() = Quaterniond(weighted_averaging_quaternions(cum_qVec[i], cum_wVec[i])).normalized().matrix();
		aMap[i] = a;
		wMap[i] = sum_wVec[i] / (double)cum_wVec[i].size();
	}

	// Calculate the transformation matrix T_i->j = T_i.inverse() * T_j
	for (size_t i=0; i<detected_boardID_viewVec[viewID].size(); i++) {
		int m = detected_boardID_viewVec[viewID][i];
		for (size_t j=i+1; j<detected_boardID_viewVec[viewID].size(); j++) {
			int d = detected_boardID_viewVec[viewID][j];
			MD md = make_pair(m,d);
			MD dm = make_pair(d,m);
			cum_m2dMap[md].push_back(aMap[m].inverse() * aMap[d]);
			cum_m2dMap[dm].push_back(aMap[d].inverse() * aMap[m]);
			cum_m2dwMap[md].push_back(wMap[m] * wMap[d]);
			cum_m2dwMap[dm].push_back(wMap[d] * wMap[m]);
		}
	}

	emptyVec.clear();
	mdVec.clear();
	// Check the empty matrix T_i->j
	for (int i=0;i<boardNo;i++) {
		for (int j=i+1; j<boardNo; j++) {
			MD md = make_pair(i,j);
			if (!cum_m2dMap.count(md)) {
				cout << "  " << i << "->" << j << " matrix is empty (md)" << endl;
				for (auto itr: cum_m2dMap) {
					if(itr.first.first == md.first) {
						// ex) T_0->2 = T_0->1 * T_1->2
						// T_0->2: empty matrix
						// T_0->1: staring matrix
						emptyVec.push_back(md); // empty matrix
						mdVec.push_back(itr.first); // starting matrix
					}
				}
			}
		}
	}

	calculate_empty_transformation_matrices_from_averaged_transfrmation_matrices();
}

void CharucoSync::calculate_empty_transformation_matrices_from_averaged_transfrmation_matrices()
{
	weighted_averaging_stacked_transformation_matrices();
//	averaging_stacked_transformation_matrices();
	cout << "*** Calculate the empty matrix from starting matrix ***" << endl;
	int count(0);
	int prev_m(-1), prev_d(-1);
	for (auto md: mdVec) {
		set<int> dSet;
		int m = md.first;
		int d = md.second;
		cout << "   " << m << "->" << d << " first" << endl;
		Eigen::Affine3d mat = avg_m2dMap[md];
		double wgt = avg_m2dwMap[md];

		dSet.insert(m);
		dSet.insert(d);

		if (prev_m >= 0 && prev_m == m) {
			dSet.insert(prev_d);
		}
		bool isBreak(false);
		set<int> s;
		while(true)
		{
			int connNum(1);
			for (auto itr: avg_m2dMap) {
				if (itr.first.first == d)
				{
					set<int>::iterator it = dSet.find(itr.first.second);

					if (it == dSet.end()) {
						dSet.insert(itr.first.second);
						mat = mat * itr.second;
						wgt = wgt * avg_m2dwMap[itr.first];
						d = itr.first.second;
						s.clear();
						connNum++;
						cout << "   " << itr.first.first << "->" << itr.first.second << endl;

						if (itr.first.second == emptyVec[count].second) {
							MD empty_md = emptyVec[count];
							MD empty_dm = make_pair(emptyVec[count].second, emptyVec[count].first);
							if (connNum > 2) {
								cout << "  Found matrix " << empty_md.first << "->" << empty_md.second
									 << ", but it will be filtered (Connection #: " + to_string(connNum) + ")" << endl;
								continue;
							}
							cum_m2dMap[empty_md].push_back(mat);
							cum_m2dMap[empty_dm].push_back(mat.inverse());
							cum_m2dwMap[empty_md].push_back(wgt*1/(double)connNum);
							cum_m2dwMap[empty_dm].push_back(wgt*1/(double)connNum);

							cout << "  Found matrix " << empty_md.first << "->" << empty_md.second
									<< "  (Connection #: " + to_string(connNum) + ")" << endl;
							isBreak = true;
							break;
						}
					}
					else {
						set<int>::iterator s_it = s.find(itr.first.second);
						if (s_it == s.end()) {
							s.insert(itr.first.second);
						}
						else {
							cout << "  Need more data" << endl;
							isBreak = true;
							break;
						}
						continue;
					}
				} // if (itr.first.first == d)
			} // for (auto itr: avg_m2dMap)
			if (isBreak) break;
		} // while(true)
		prev_m = md.first;
		prev_d = md.second;
		count++;
		cout << "  ------------" << endl;
	}

	cout << "*** Check the transformation matrices for all coordinates ***" << endl;
	bool isPrint(true);
	for (int i=0;i<boardNo;i++) {
		for (int j=i+1; j<boardNo; j++) {
			MD md = make_pair(i,j);
			if (!cum_m2dMap.count(md)) {
				isPrint = false;
				cout << "  " << i << "->" << j << " matrix is empty (md)" << endl;
			}
		}
	}
	if (!isPrint) return;
	// Re-averaging stacked transformation matrices including calculated empty matrices.
	// averaging_stacked_transformation_matrices();
	weighted_averaging_stacked_transformation_matrices();
	cout << "===================================================================" << endl;
	cout <<	"Congratulations!! All transformation matrices were set in 'm2d.dat'" << endl;
	cout << "Press 'q' to exit the program" << endl;
	cout << "===================================================================" << endl;
	ofstream ofs("m2d.dat");
	for (auto itr: avg_m2dMap) {
		Quaterniond q = Quaterniond(itr.second.rotation());
		Vector3d t = itr.second.translation();
		ofs << itr.first.first << " " << itr.first.second << endl;
		ofs << "q " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
		ofs << "t " << t.x() << " " << t.y() << " " << t.z() << endl;
	} ofs.close();
}

void CharucoSync::averaging_stacked_transformation_matrices()
{
	avg_m2dMap.clear();
	cout << "*** Averaging stacked transformation matrices ***" << endl;
	for (auto itr: cum_m2dMap) {
		MD md = make_pair(itr.first.first, itr.first.second);
		if (itr.second.empty()) continue;
		vector<Vector4d> qVec;
		vector<Vector3d> tVec;
		Eigen::Affine3d avgMat;
		Vector3d sum_trans = Vector3d::Zero();
		for (auto m: itr.second) {
			Quaterniond q(m.linear());
			Vector4d quat(q.x(),q.y(),q.z(),q.w());
			qVec.push_back(quat);
			sum_trans += Vector3d(m.translation());
		}
		Vector4d avgQuat = averaging_quaternions(qVec);
		Quaterniond avgQ(avgQuat);
		Vector3d avgTrans = sum_trans / (double)itr.second.size();

		avgMat.linear() = avgQ.normalized().matrix();
		avgMat.translation() = avgTrans;
		avg_m2dMap[md] = avgMat;
	}
}

void CharucoSync::weighted_averaging_stacked_transformation_matrices()
{
	avg_m2dMap.clear();
	avg_m2dwMap.clear();
	for (auto itr: cum_m2dMap) {
		MD md = itr.first;
		if (itr.second.empty()) continue;
		vector<Vector4d> qVec;
		vector<Vector3d> tVec;
		Eigen::Affine3d avgMat;
		Vector3d sum_trans = Vector3d::Zero();
		for (auto m: itr.second) {
			Quaterniond q(m.linear());
			Vector4d quat(q.x(),q.y(),q.z(),q.w());
			qVec.push_back(quat);
			sum_trans += Vector3d(m.translation());
		}
//		Vector4d avgQuat = averaging_quaternions(qVec);
		Vector4d avgQuat = weighted_averaging_quaternions(qVec, cum_m2dwMap[md]);
		Quaterniond avgQ(avgQuat);
		Vector3d avgTrans = sum_trans / (double)itr.second.size();

		avgMat.linear() = avgQ.normalized().matrix();
		avgMat.translation() = avgTrans;
		avg_m2dMap[md] = avgMat;
		avg_m2dwMap[md] = averaging_weights(cum_m2dwMap[md]);
	}
}




double averaging_weights(std::vector<double> &weights)
{
	double sum_weight(0);
	for (const auto &w:weights) {
		sum_weight += w;
	}
	return sum_weight/(double)weights.size();
}

Eigen::Vector4d averaging_quaternions(std::vector<Eigen::Vector4d> quaternions)
{
    if (quaternions.empty())
        return Eigen::Vector4d(0,0,0,1);

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

Eigen::Vector4d weighted_averaging_quaternions(
  const std::vector<Eigen::Vector4d>& quaternions,
  const std::vector<double>& weights)
{
	Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, quaternions.size());
	Eigen::Vector3d vec;
	for (size_t i = 0; i < quaternions.size(); ++i)
	{
		// Weigh the quaternions according to their associated weight
		Eigen::Vector4d quat = quaternions[i] * weights[i];
		// Append the weighted Quaternion to a matrix Q.
		Q(0,i) = quat.x();
		Q(1,i) = quat.y();
		Q(2,i) = quat.z();
		Q(3,i) = quat.w();
	}

	// Creat a solver for finding the eigenvectors and eigenvalues
	Eigen::EigenSolver<Eigen::MatrixXd> es(Q * Q.transpose());

	// Find index of maximum (real) Eigenvalue.
	auto eigenvalues = es.eigenvalues();
	size_t max_idx = 0;
	double max_value = eigenvalues[max_idx].real();
	for (size_t i = 1; i < 4; ++i)
	{
		double real = eigenvalues[i].real();
		if (real > max_value)
		{
		  max_value = real;
		  max_idx = i;
		}
	}

	// Get corresponding Eigenvector, normalize it and return it as the average quat
	auto eigenvector = es.eigenvectors().col(max_idx).normalized();

	Eigen::Vector4d mean_orientation(
		eigenvector[0].real(),
		eigenvector[1].real(),
		eigenvector[2].real(),
		eigenvector[3].real());

  return mean_orientation;
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
    			Point(10, 30*(i+1)), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255.f,0.f,0.f), 1.5);
    }

    if(getPose)
        putText(display, "obtaining pose data..", Point(10, display.rows - 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255.f,0.f,0.f), 1.5);
    imshow("Synchronization", display);
}


void CharucoSync::ShowAvgValue(const Mat &color)
{
	for (int i=0; i<boardNo; i++) {
		Vec3d rvec;
		AngleAxisd avg(Quaterniond((averaging_quaternions(cum_qVec[i]))));
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
//	cout << "'" << fileName << "' was generated." << endl;
    ofstream ofs(fileName);
    for (int i=0; i<boardNo;i ++)
    {
//    	Vector4d q = averaging_quaternions(cum_qVec[i]);
    	Vector4d q = weighted_averaging_quaternions(cum_qVec[i], cum_wVec[i]);
    	Vec3d t;
    	if (cum_qVec[i].size() == 0)
    		t = Vec3d(0,0,0);
    	else
    		t = sum_tVec[i] * 100 / (double)cum_qVec[i].size();
    	Vector3d tt(t.val[0], t.val[1], t.val[2]);

    	ofs << "q " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) <<endl;
    	ofs << "t " << t(0) << " " << t(1) << " " << t(2) << endl;
    }
	time_t now = chrono::system_clock::to_time_t(chrono::system_clock::now());

	ofs << endl;
	ofs << ctime(&now) << endl;
    ofs.close();
}

void CharucoSync::SetScalingFactor(float s)
{
    sf = s;
    sfInv = 1 / s;
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

