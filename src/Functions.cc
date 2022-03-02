#include "Functions.hh"

k4a_device_configuration_t get_default_config()
{
	k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	camera_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
	camera_config.subordinate_delay_off_master_usec = 0;
	camera_config.synchronized_images_only = true;

	return camera_config;
}

cv::Mat color_to_opencv(const k4a_image_t im)
{
	cv::Mat cv_image_with_alpha(k4a_image_get_height_pixels(im), k4a_image_get_width_pixels(im), CV_8UC4, (void*)k4a_image_get_buffer(im));
    cv::Mat cv_image_no_alpha;
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
    return cv_image_no_alpha;
}

cv::Mat depth_to_opencv(const k4a_image_t im)
{
    return cv::Mat(k4a_image_get_height_pixels(im),
    			   k4a_image_get_width_pixels(im),
				   CV_16U,
        (void*)k4a_image_get_buffer(im),
        static_cast<size_t>(k4a_image_get_stride_bytes(im)));
}

k4a_image_t Convert_Color_MJPG_To_BGRA(k4a_image_t color_image)
{
	k4a_image_t uncompressed_color_image = NULL;
	// Convert color frame from mjpeg to bgra
	k4a_image_format_t format;
	format = k4a_image_get_format(color_image);
	if (format != K4A_IMAGE_FORMAT_COLOR_MJPG) {
		printf("Color format not supported. Please use MJPEG\n"); exit(1);
	}

	int color_width, color_height;
	color_width = k4a_image_get_width_pixels(color_image);
	color_height = k4a_image_get_height_pixels(color_image);

	if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
												 color_width,
												 color_height,
												 color_width * 4 * (int)sizeof(uint8_t),
												 &uncompressed_color_image))
	{
		printf("Failed to create image buffer\n"); exit(1);
	}

	tjhandle tjHandle;
	tjHandle = tjInitDecompress();
	if (tjDecompress2(tjHandle,
					  k4a_image_get_buffer(color_image),
					  static_cast<unsigned long>(k4a_image_get_size(color_image)),
					  k4a_image_get_buffer(uncompressed_color_image),
					  color_width,
					  0, // pitch
					  color_height,
					  TJPF_BGRA,
					  TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) != 0)
	{
		printf("Failed to decompress color frame\n");
		if (tjDestroy(tjHandle))
		{
			printf("Failed to destroy turboJPEG handle\n");
		}
		exit(1);
	}
	if (tjDestroy(tjHandle))
	{
		printf("Failed to destroy turboJPEG handle\n");
	}
	return uncompressed_color_image;
}

void WritePointCloud(const k4a_image_t point_image, 
                     const k4a_image_t color_image,
                     string fileName)
{
	cout << "WritePointCloud()" << endl;
    vector<RowVector3d> xyz;
    vector<RowVector3i> rgb;

    int width  = k4a_image_get_width_pixels(point_image);
    int height = k4a_image_get_height_pixels(color_image);

    int16_t *point_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_image);
    uint8_t *color_image_data = k4a_image_get_buffer(color_image);

    for (int i=0; i<width*height; i++) {
        
        double Z = point_image_data[3 * i + 2];
        if (Z == 0) continue;
        double X = point_image_data[3 * i + 0];
        double Y = point_image_data[3 * i + 1];

        int b = color_image_data[4 * i + 0];
        int g = color_image_data[4 * i + 1];
        int r = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (b == 0 && g == 0 && r == 0 && alpha == 0)
            continue;

        xyz.push_back(RowVector3d(X,Y,Z));
        rgb.push_back(RowVector3i(r,g,b));
    }

    ofstream ofs(fileName + ".ply");
    ofs << "ply" << endl;
    ofs << "format ascii 1.0" << endl;
    ofs << "element vertex "  << xyz.size() << endl;
    ofs << "property float x" << endl;
    ofs << "property float y" << endl;
    ofs << "property float z" << endl;
    ofs << "property uchar red" << endl;
    ofs << "property uchar green" << endl;
    ofs << "property uchar blue" << endl;
    ofs << "end_header" << endl;

    for (size_t i=0;i<xyz.size();i++) {
        ofs << xyz[i] << " " << rgb[i] << endl;
    }
    ofs.close();
}

void WriteTrasnformedPointCloud(const k4a_image_t point_image, 
                     			const k4a_image_t color_image,
                     			string fileName)
{
	cout << "WriteTransformedPointCloud()" << endl;
	ifstream ifs("ori_CoordData");
	if(!ifs.is_open()) {
		cerr << "ori_CoordData was not opened" << endl;
		exit(1);
	}

	string dump;
	double x,y,z,w;

	ifs >> dump >> x >> y >> z >> w;
	Quaterniond quat(w,x,y,z);
	ifs >> dump >> x >> y >> z;
	Vector3d trans(x*10,y*10,z*10); // cm => mm

	Affine3d a;
	a.translation() = trans;
	a.linear() = quat.normalized().matrix();

	vector<RowVector3d> xyz;
	vector<RowVector3i> rgb;

	int width  = k4a_image_get_width_pixels(point_image);
	int height = k4a_image_get_height_pixels(color_image);

	int16_t *point_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	for (int i=0; i<width*height; i++) {
		if (point_image_data[3*i+2] == 0) continue;
		Vector3d pointVec(point_image_data[3*i+0], point_image_data[3*i+1], point_image_data[3*i+2]);
		pointVec = a.inverse() * pointVec;

		double X = pointVec(0);
		double Y = pointVec(1);
		double Z = pointVec(2);

		int b = color_image_data[4 * i + 0];
		int g = color_image_data[4 * i + 1];
		int r = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];

		if (b == 0 && g == 0 && r == 0 && alpha == 0)
			continue;

		xyz.push_back(RowVector3d(X,Y,Z));
		rgb.push_back(RowVector3i(r,g,b));
	}

	ofstream ofs(fileName + ".ply");
	ofs << "ply" << endl;
	ofs << "format ascii 1.0" << endl;
	ofs << "element vertex "  << xyz.size() << endl;
	ofs << "property float x" << endl;
	ofs << "property float y" << endl;
	ofs << "property float z" << endl;
	ofs << "property uchar red" << endl;
	ofs << "property uchar green" << endl;
	ofs << "property uchar blue" << endl;
	ofs << "end_header" << endl;

	for (size_t i=0;i<xyz.size();i++) {
		ofs << xyz[i] << " " << rgb[i] << endl;
	}
	ofs.close();


}

void WriteTrasnformedPointCloudToRefCoord(const k4a_image_t point_image,
                     					  const k4a_image_t color_image,
										  string fileName)
{
	cout << "WriteTrasnformedPointCloudToRefCoord()" << endl;

	string dump;
	double x,y,z,w;

	ifstream ifs("m2dQT.dat");
	if(!ifs.is_open()) { cerr << "ifs" << endl;	exit(1); }
	vector<Eigen::Affine3d> T_prime;

	T_prime.push_back(Affine3d::Identity());
	for (int i=0; i<3; i++) {
		int m, d;
		ifs >> m >> d;
		ifs >> dump >> x >> y >> z >> w;
		Quaterniond quat(w,x,y,z);
		ifs >> dump >> x >> y >> z;
		Vector3d trans(x*10,y*10,z*10); // cm => mm

		Eigen::Affine3d a;
		a.linear() = quat.normalized().matrix();
		a.translation() = trans;
		T_prime.push_back(a);
	}
	ifs.close();

	ifstream ifs2("view0_CoordData");
	if(!ifs2.is_open()) { cerr << "ifs2" << endl; exit(1); }
	vector<Eigen::Affine3d> T;

	for (int i=0; i<4; i++) {
		ifs2 >> dump >> x >> y >> z >> w;
		Quaterniond quat(w,x,y,z);
		ifs2 >> dump >> x >> y >> z;
		Vector3d trans(x*10,y*10,z*10); // cm => mm

		Eigen::Affine3d a;
		a.linear() = quat.normalized().matrix();
		a.translation() = trans;
		T.push_back(a);
	}
	ifs2.close();

	vector<Eigen::Affine3d> Tf;
	for (int i=0;i<4;i++) {
		Tf.push_back(T_prime[i] * T[i].inverse()); // T_0.inverse() = T_0->1 * T_1.inverse()
	}

	vector<RowVector3d> xyz;
	vector<RowVector3i> rgb;

	int width  = k4a_image_get_width_pixels(point_image);
	int height = k4a_image_get_height_pixels(color_image);

	int16_t *point_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_image);
	uint8_t *color_image_data = k4a_image_get_buffer(color_image);

	for (int i=0; i<width*height; i++) {
		if (point_image_data[3*i+2] == 0) continue;
		Vector3d pointVec(point_image_data[3*i+0], point_image_data[3*i+1], point_image_data[3*i+2]);
		pointVec = Tf[1] * pointVec;

		double X = pointVec(0);
		double Y = pointVec(1);
		double Z = pointVec(2);

		int b = color_image_data[4 * i + 0];
		int g = color_image_data[4 * i + 1];
		int r = color_image_data[4 * i + 2];
		uint8_t alpha = color_image_data[4 * i + 3];

		if (b == 0 && g == 0 && r == 0 && alpha == 0)
			continue;

		xyz.push_back(RowVector3d(X,Y,Z));
		rgb.push_back(RowVector3i(r,g,b));
	}

	ofstream ofs(fileName + ".ply");
	ofs << "ply" << endl;
	ofs << "format ascii 1.0" << endl;
	ofs << "element vertex "  << xyz.size() << endl;
	ofs << "property float x" << endl;
	ofs << "property float y" << endl;
	ofs << "property float z" << endl;
	ofs << "property uchar red" << endl;
	ofs << "property uchar green" << endl;
	ofs << "property uchar blue" << endl;
	ofs << "end_header" << endl;

	for (size_t i=0;i<xyz.size();i++) {
		ofs << xyz[i] << " " << rgb[i] << endl;
	}
	ofs.close();


}


