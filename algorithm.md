# PCL�����ƿ⣩& Eigen�����Դ�����)

PCL�����ƴ�����Eigen���������㣩��ROS����Ϣ���ݣ�
## 2222

```CPP
�����˲�??���뾶�˲���ֱͨ�˲����뾶��Ⱥ��ȥ����
??���Ʒָ�??��ŷʽ���ࡢƽ����ϣ�RANSAC����
??������׼??��ICP�㷨�����ڽ�ģ�������Ŀ����ƶ��롣
??����任??��ʹ��Eigen���о���任��
??��̬��ʾ??��ʹ��ŷ���ǣ�roll, pitch, yaw����ʾ��ж������̬��
??���μ���??��ͨ�����ġ������нǵȼ��㳵������
std::mutex mtx;
std::condition_variable cv;
bool udp_data_ready = false; 
uint8_t g_type = 0;
double g_angle = 0.0;
int model_car = 1; //��ʱ��ͨѶ�����������ͺ�
int model = 1; //��ʱ��ͨѶ�����������ͺ�
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher dis_and_att_pub;
int catch_count = 3;
 
///** */
//ll
Eigen::Matrix4d getRotationMatrixZ_Eigen(double angle_clockwise) {
    double radians = angle_clockwise * M_PI / 180.0;

    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<2,2>(0,0) = Eigen::Rotation2Dd(radians).toRotationMatrix();
    return mat;
}
bool sig_strat=0;
double angle =0.0;
double rz_2d_array[4][4]; 
Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
void get2DArray(const Eigen::Matrix4d& mat, double out[4][4]) {
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            out[i][j] = mat(i, j);
        }
    }
}
void udpMsgCallback(const UDP_ros_pkg::fromudp::ConstPtr& msg)
{
	model=msg->type;
	angle=msg->angle;
	Rz=getRotationMatrixZ_Eigen(angle);
	get2DArray(Rz,rz_2d_array);
}
void change_sig(const std_msgs::Bool::ConstPtr& msg)
{
	sig_strat=msg->data;
}
//ll





float PI = 3.1415926;
pcl::PointCloud<pcl::PointXYZ>::Ptr m_template_point_cloud_trans;
pcl::PointCloud<pcl::PointXYZ>::Ptr template_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr model1_template_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr model2_template_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr model3_template_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// ƽ�淽�̽ṹ��
struct PlaneEquation {
	float a, b, c, d;  // Ax+By+Cz+D=0
};

// ���㵥���㵽ƽ��ľ���
double computePointToPlaneDistance(const pcl::PointXYZ& point, const pcl::ModelCoefficients::Ptr& plane) 
{
	const double a = plane->values[0];
	const double b = plane->values[1];
	const double c = plane->values[2];
	const double d = plane->values[3];
	return std::fabs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(a * a + b * b + c * c);
}


//��ȡƽ������ƵĽ���
void extractIntersectionPlaneAndPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input, const PlaneEquation& plane,
	float threshold_distance, pcl::PointCloud<pcl::PointXYZ>::Ptr& output)
{
	const float a = plane.a;
	const float b = plane.b;
	const float c = plane.c;
	const float d = plane.d;
	float threhod = threshold_distance;
	//std::vector<double> distances;
	//distances.resize(input->size());
	float disPointToPlane;
	for (size_t i = 0; i < input->size(); ++i) {
		disPointToPlane = (fabs(a * input->points[i].x + b * input->points[i].y + c * input->points[i].z + d) / std::sqrt(a * a + b * b + c * c));
		if (disPointToPlane < threhod)
		{
			output->points.push_back(input->points[i]);
		}
	}
}

void ICPMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, int& maxIterNum, Eigen::Matrix4d& transform)

{
	/* ���� ICP ����  */
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

	/*���� ���� Դ����*/
	gicp.setInputSource(source);

	/*���� ���� Ŀ�����*/
	gicp.setInputTarget(target);
	/*��4�������������� */
		// �������Ķ�Ӧ��ϵ����Ϊ ���� ��λm�� ����Ӧ������ڴ˾��뽫�����ԣ� 
	gicp.setMaxCorrespondenceDistance(0.5);
	// ��������������(Ĭ�� 1)
	gicp.setMaximumIterations(maxIterNum);
	// ���� ���α任����֮��Ĳ�ֵ (Ĭ�� 2)
	gicp.setTransformationEpsilon(1e-8);
	// ���� �������(Ĭ�� 3)
	gicp.setEuclideanFitnessEpsilon(0.01);
	//icp.setUseReciprocalCorrespondences(true);//����Ϊtrue,��ʹ���໥��Ӧ��ϵ

	/* ���� ������׼�任 Դ���� �� �� ���� */
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	/*ִ����׼  �洢�任���Դ���Ƶ� Final ��  */
	gicp.align(*final);


	/*���� ���յı任����*/
	Eigen::Matrix4f transformation;
	//��ֵ��ת����
	transformation = gicp.getFinalTransformation();
	transform = transformation.cast<double>();

	transform = Eigen::Matrix<double, 4, 4>::Identity();
	transform(0, 0) = double(transformation(0, 0));
	transform(0, 1) = double(transformation(0, 1));
	transform(0, 2) = double(transformation(0, 2));
	transform(0, 3) = double(transformation(0, 3));
	transform(1, 0) = double(transformation(1, 0));
	transform(1, 1) = double(transformation(1, 1));
	transform(1, 2) = double(transformation(1, 2));
	transform(1, 3) = double(transformation(1, 3));
	transform(2, 0) = double(transformation(2, 0));
	transform(2, 1) = double(transformation(2, 1));
	transform(2, 2) = double(transformation(2, 2));
	transform(2, 3) = double(transformation(2, 3));
	transform(3, 0) = double(transformation(3, 0));
	transform(3, 1) = double(transformation(3, 1));
	transform(3, 2) = double(transformation(3, 2));
	transform(3, 3) = double(transformation(3, 3));

	//print4x4Matrix(transform, Dis, roll, pitch, yaw);

}

void DisplayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_1_ptr,
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_2_ptr) {
	pcl::visualization::PCLVisualizer viewer("3D viewer");
	viewer.setBackgroundColor(0.3, 0.3, 0.3);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_1(point_cloud_1_ptr, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_2(point_cloud_2_ptr, 0, 255, 0);
	viewer.addPointCloud<pcl::PointXYZ>(point_cloud_1_ptr, single_color_1, "point cloud 1");
	viewer.addPointCloud<pcl::PointXYZ>(point_cloud_2_ptr, single_color_2, "point cloud 2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "point cloud 1");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "point cloud 2");
	viewer.addCoordinateSystem(5.0);
	viewer.initCameraParameters();

	while (!viewer.wasStopped()) {
		viewer.spin();
	}
}
//**************************���ڴ�4x4��α任��������ȡλ�ƺ�ŷ���ǣ�roll, pitch, yaw�����ǻ�����ѧ��3D�Ӿ��еĺ��Ĺ��ߡ�
void print4x4Matrix(const Eigen::Matrix4d& matrix, Eigen::Vector3d& Dis, double& roll, double& pitch, double& yaw)
{
	//double roll, pitch, yaw;

	// Assuming the angles are in radians.
	double sy = sqrt(matrix(0, 0) * matrix(0, 0) + matrix(1, 0) * matrix(1, 0));
	if (matrix(2, 0) > 0.998)
	{ // singularity at north pole
		pitch = atan2(-matrix(1, 2), matrix(1, 1));
		yaw = atan2(-matrix(2, 0), sy);
		roll = 0;

	}
	else if (matrix(2, 0) < -0.998)
	{ // singularity at south pole
		pitch = atan2(-matrix(1, 2), matrix(1, 1));
		yaw = atan2(-matrix(2, 0), sy);
		roll = 0;

	}
	else
	{
		pitch = atan2(matrix(2, 1), matrix(2, 2));
		yaw = atan2(-matrix(2, 0), sy);
		roll = atan2(matrix(1, 0), matrix(0, 0));
	}

	// Convert angles from radians to degrees
	roll = roll * 180.0 / PI;
	pitch = pitch * 180.0 / PI;
	yaw = yaw * 180.0 / PI;
	Dis[0] = matrix(0, 3);
	Dis[1] = matrix(1, 3);
	Dis[2] = matrix(2, 3);

}

// ������X�����ת����
Matrix3d rotationMatrixX(double angle)
{
	double c = cos(angle);
	double s = sin(angle);

	Matrix3d rotation_matrix;
	rotation_matrix <<
		1, 0, 0,
		0, c, -s,
		0, s, c;

	return rotation_matrix;
}

// ������Y�����ת����
Matrix3d rotationMatrixY(double angle)
{
	double c = cos(angle);
	double s = sin(angle);

	Matrix3d rotation_matrix;
	rotation_matrix <<
		c, 0, s,
		0, 1, 0,
		-s, 0, c;
	return rotation_matrix;
}

// ������Z�����ת����
Matrix3d rotationMatrixZ(double angle)
{
	double c = cos(angle);
	double s = sin(angle);
	//cout << "c s" << c << s << endl;
	Matrix3d rotation_matrix;
	rotation_matrix <<
		c, -s, 0,
		s, c, 0,
		0, 0, 1;
	return rotation_matrix;

}

Matrix3d theta2m(double angle1, double angle2, double angle3)
{
	// ŷ���ǣ������ƣ�
	//cout << angle1 << angle2 << angle3 << endl;
	double anglex = angle1 * PI / 180.0;
	double angley = angle2 * PI / 180.0;
	double anglez = angle3 * PI / 180.0;

	// ������ת����
	Matrix3d rotationX = rotationMatrixX(anglex);
	Matrix3d rotationY = rotationMatrixY(angley);
	Matrix3d rotationZ = rotationMatrixZ(anglez);

	Matrix3d rotate_matrix;
	return  rotate_matrix = rotationZ * rotationY * rotationX;
}

Matrix4d m2M(double angle1, double angle2, double angle3, double x, double y, double z)
{
	Matrix4d transformationMatrix = Matrix4d::Identity();
	Matrix3d rotationMatrix = theta2m(angle1, angle2, angle3);
	transformationMatrix.block<3, 3>(0, 0) = rotationMatrix;
	Vector3d translationVector;
	translationVector(0) = x;
	translationVector(1) = y;
	translationVector(2) = z;
	transformationMatrix.block<3, 1>(0, 3) = translationVector;
	return transformationMatrix;
}

void findcorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target, pcl::PointCloud<pcl::PointXYZ>::Ptr& P, pcl::PointCloud<pcl::PointXYZ>::Ptr& Q)
{
	P->clear();
	Q->clear();
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(target);
	//#pragma omp parallel for num_threads(200)
	for (int i = 0; i < source->points.size(); i++)
	{
		vector<int> indx;
		vector<float> dists;
		kdtree.nearestKSearch(source->points[i], 1, indx, dists);
		if (dists.size() > 0)
		{
			if (dists[0] < 1)
			{
				P->push_back(source->points[i]);
				Q->push_back(target->points[indx[0]]);
			}
		}
	}
}

Eigen::Vector3f Distance(pcl::PointCloud<pcl::PointXYZ>::Ptr& target)
{
	Eigen::Vector4f centroid_s, centroid_t;					// ����
	// ������꣬��c0,c1,c2,1��
	pcl::compute3DCentroid(*target, centroid_t);
	Eigen::Vector3f Distance;
	Distance(0) = centroid_t(0);
	Distance(1) = centroid_t(1);
	Distance(2) = centroid_t(2);
	return Distance;
}


void pc_normalize(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target)
{
	//// ��ȡ���Ƶ�����
	//int l = pc.rows();

	// ������Ƶ�����
	Eigen::Vector4f centroid_s, centroid_t;					// ����
	pcl::compute3DCentroid(*source, centroid_s);	// ������꣬��c0,c1,c2,1��
	pcl::compute3DCentroid(*target, centroid_t);
	pcl::PointCloud<pcl::PointXYZ>::Ptr normalized_pc(new pcl::PointCloud<pcl::PointXYZ>);

	double Distancex = centroid_t(0);
	double Distancey = centroid_t(1);
	double Distancez = centroid_t(2);
	//cout << " Distancex, Distancey, Distancez:" << Distancex<<","<<Distancey << ","<<Distancez << endl;

	for (int i = 0; i < source->points.size(); i++)
	{
		source->points[i].x += Distancex;
		source->points[i].y += Distancey;
		source->points[i].z += Distancez;

	}

}

double computeUpdate(double param1, double param2, double param3) {
	double sum_squared_diff = 0.0;
	sum_squared_diff = param1 * param1 + param2 * param2 + param3 * param3;
	double rmse = std::sqrt(sum_squared_diff / 3);
	cout << "rmse: " << rmse << endl;
	return rmse;
}

Eigen::Matrix<double, 4, 4> findRT(pcl::PointCloud<pcl::PointXYZ>::Ptr& P, pcl::PointCloud<pcl::PointXYZ>::Ptr& Q)
{
	int n = P->points.size();
	Eigen::Matrix<double, 3, Eigen::Dynamic> matrixP(3, n), matrixQ(3, n);
	for (int i = 0; i < n; i++)
	{
		matrixP(0, i) = P->points[i].x;
		matrixP(1, i) = P->points[i].y;
		matrixP(2, i) = P->points[i].z;
		matrixQ(0, i) = Q->points[i].x;
		matrixQ(1, i) = Q->points[i].y;
		matrixQ(2, i) = Q->points[i].z;
	}
	Eigen::Matrix4d RT = Eigen::umeyama(matrixP, matrixQ);
	//print4x4Matrix(RT, Dis);
	//cout << " (Distancex, Distancey, Distancez)" << Dis;
	return RT;
}


void DisplayPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& point_cloud_1_ptr) {
    pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer.setBackgroundColor(0.3, 0.3, 0.3);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color_1(point_cloud_1_ptr, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(point_cloud_1_ptr, single_color_1, "point cloud 1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "point cloud 1");
    viewer.addCoordinateSystem(5.0);
    viewer.initCameraParameters();

    while (!viewer.wasStopped()) 
	{
        viewer.spin();
    }
}

void CreateCarTemplate()
{
	//����ͼ��ʶ���ͣ����ݳ��Ϳɵõ������ĳ���ߣ���λ���ף�
	float model1_length_car_body = 3.5;
	float model1_width_car_body = 2.0;
	float model1_height_car_body = 0.5;

	//���ݳ����ߴ紴��ģ��,�ò��ֿ��������
	pcl::PointCloud<pcl::PointXYZ>& cloud = *model1_template_point_cloud;
	float point_distance = 0.05;

	int template_point_cloud_width = model1_length_car_body / point_distance;  // ÿ��200����
	int template_point_cloud_height = model1_width_car_body / point_distance; // ��100��
	model1_template_point_cloud->is_dense = false;
	model1_template_point_cloud->width = template_point_cloud_width;
	model1_template_point_cloud->height = template_point_cloud_height;

	for (int i = 0; i < 12; i++)
	{
		for (int x = 0; x < template_point_cloud_width; ++x)
		{
			cloud.points.push_back(pcl::PointXYZ{ float(-1.75 + float(x) * point_distance), -1.0 , -float(0.0 + float(i) * point_distance) });
			cloud.points.push_back(pcl::PointXYZ{ float(-1.75 + float(x) * point_distance),  1.0 , -float(0.0 + float(i) * point_distance) });
		}

		for (int y = 0; y < template_point_cloud_height; ++y)
		{
			cloud.points.push_back(pcl::PointXYZ{ -1.75, float(-1.0 + float(y) * point_distance), -float(0.0 + float(i) * point_distance) });
			cloud.points.push_back(pcl::PointXYZ{ 1.75, float(-1.0 + float(y) * point_distance), -float(0.0 + float(i) * point_distance) });
		}
	}
	//��ӵײ�ƽ�����
	for (int x = 0; x < template_point_cloud_width; ++x)
	{
		for (int y = 0; y < template_point_cloud_height; ++y)
		{
			cloud.points.push_back(pcl::PointXYZ{ float(-1.75 + float(x) * point_distance), float(-1.0 + float(y) * point_distance), -float(0.5) });
		}
	}
}

//2025.7.5�����Ӻ���
void ExtractPlanePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_1_ptr, float x_low, float x_up, float y_low, float y_up, float z_low, float z_up, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_2_ptr)
{
	
	// X�᷶Χ����
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_x_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_y_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PassThrough<pcl::PointXYZ> pass_x;
	pass_x.setInputCloud(point_cloud_1_ptr);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(x_low, x_up); // ���ó�����X�᷶Χ
	pass_x.setNegative(false);
	pass_x.filter(*point_cloud_x_ptr);

	// Y�᷶Χ����
	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud(point_cloud_x_ptr);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(y_low, y_up); // ���ó�����Y�᷶Χ
	pass_y.filter(*point_cloud_y_ptr);

	// Z�᷶Χ����
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud(point_cloud_y_ptr);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(z_low, z_up); // ���ó�����Z�᷶Χ
	pass_z.filter(*point_cloud_2_ptr);
}

float car_center_x_by_img = 5.47;
float car_center_y_by_img = -0.2;
float car_center_z_by_img = -0.3;
int flag = 0; //1��ʾ�н�����أ�0��ʾ��ȡ����ʧ�ܣ�-1��ʾ��2��-15�׷�Χ����ȡ�������ʧ��



void total_process(pcl::PointCloud<pcl::PointXYZ>::Ptr origin_point_cloud)
{
	//pcl::PointXYZ origin(0, 0, 0);
	pcl::PointXYZ origin = origin_point_cloud->points.back();
	std::cout << "minpoint(���ĵ�)��"<< origin << std::endl;
	car_center_x_by_img = origin.x;
 	car_center_y_by_img = origin.y;
	car_center_z_by_img = origin.z;

	Vector3d Dis;
	Dis[0] = 0;
	Dis[1] = 0;
	Dis[2] = 0;
	double rollCurrent = 0.0;
	double pitchCurrent = 0.0;
	double yawCurrent = 0.0;



	//����
	double m_length_car_body = 0.0;
	double m_width_car_body = 0.0;
	double m_height_car_body = 0.0;

	//2025.7.5������
	double model1_length_car_body = 3.5;
	double model1_width_car_body = 2.0;
	double model1_height_car_body = 0.5;

	double model2_length_car_body = 3.5;
	double model2_width_car_body = 2.0;
	double model2_height_car_body = 0.5;

	double model3_length_car_body = 3.5;
	double model3_width_car_body = 2.0;
	double model3_height_car_body = 0.5;

	if (model_car == 1)
	{
		m_length_car_body = model1_length_car_body;
		m_width_car_body = model1_width_car_body;
		m_height_car_body = model1_height_car_body;
	}
	else if (model_car == 2)
	{
		m_length_car_body = model2_length_car_body;
		m_width_car_body = model2_width_car_body;
		m_height_car_body = model2_height_car_body;
	}
	else if(model_car == 3)
	{
		m_length_car_body = model3_length_car_body;
		m_width_car_body = model3_width_car_body;
		m_height_car_body = model3_height_car_body;
	}
////2025.7.5�����ӽ���

	// ������ж�����ڻ��������뷶Χ���а뾶ŷʽ�����˲�
	pcl::PointXYZ origin1(0, 0, 0);
	***********************pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	**��ԭʼ������֯����״�ṹ********************kdtree.setInputCloud(origin_point_cloud);

	// �뾶������ȡ������ֵ�ڵ�
	std::vector<int> indices;
	std::vector<float> distances;
    **********************indices: ����������洢���ڰ뾶�ڵĵ�����������е�����
*********************distances: ����������洢��Щ�㵽���ĵ��ŷ�Ͼ����ƽ��
	**************************kdtree.radiusSearch(origin1, float(car_center_x_by_img+6.0), indices, distances);

	// ��ȡ�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_origin_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	***************************pcl::copyPointCloud(*origin_point_cloud, indices, *filtered_origin_point_cloud);

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix = m2M(0.0, 17.0, 0.0, 0.0, 0.0, 0.0); //��һ֡���ݽǶ�
	Eigen::Matrix4d transformation_matrix_inv = Eigen::Matrix4d::Identity();
	transformation_matrix_inv = m2M(0.0, -17.0, 0.0, 0.0, 0.0, 0.0); //��һ֡���ݽǶ�
	�����״�ԭ��Ϊ���ģ�ͼ�����ڳ����ĵ��Ƶ��е�x����+6Ϊ�뾶���˵�����ĵ��ƣ�Ȼ���Ƴ������б任����y��ת17�ȣ���KdTreeFLANN �� PCL��Point Cloud Library�������ڿ��������������һ���࣬�ر������ڴ���������ݡ�������ǻ��� FLANN��Fast Library for Approximate Nearest Neighbors����ʵ�ֵ� kd-tree ���ݽṹ��**************************pcl::transformPointCloud(*filtered_origin_point_cloud, *filtered_origin_point_cloud, transformation_matrix);

	if (filtered_origin_point_cloud->size() > 100)
	{
		// 2. ���þ�����ֵ����ԭ��(0,0,0)Ϊ�ο��㣩
		float distance_threshold = 2.0; // ��λ����
		// 3. ����������˺�ĵ�������
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// 4. �������ƽ��о������
		for (const auto& point : filtered_origin_point_cloud->points)
		{
			float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
			if (dist > distance_threshold)
			{
				filtered_cloud->push_back(point);
			}
		}

		if (filtered_cloud->size() > 100)
		{
			//������ж���ϵ����ŷʽ�����˲�

			pcl::PointXYZ origin_car(car_center_x_by_img, car_center_y_by_img, car_center_z_by_img);
			
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_car;
			kdtree_car.setInputCloud(filtered_cloud);

			// �뾶������ȡ������ֵ�ڵ�
			kdtree_car.radiusSearch(origin_car, float(6.0), indices, distances);
    ***********************�ٴν��а뾶����
			// ��ȡ�������
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			if (indices.size() > 100)
			{
				pcl::copyPointCloud(*filtered_cloud, indices, *filtered_point_cloud);

***********************���������������µ���
				//���˲���ĵ���г�ͷ����
			    //�뾶��Ⱥ��ȥ��
				pcl::PointCloud<pcl::PointXYZ>::Ptr fliter_outlier_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::RadiusOutlierRemoval<pcl::PointXYZ> fliter_outlier_point;
				fliter_outlier_point.setInputCloud(filtered_point_cloud);
				fliter_outlier_point.setRadiusSearch(0.5);     // �����뾶0.1��λ
				fliter_outlier_point.setMinNeighborsInRadius(50); // �뾶������5����
				fliter_outlier_point.filter(*fliter_outlier_point_cloud);
**********************************�뾶��Ⱥ��ȥ����������
				if (fliter_outlier_point_cloud->size() > 0)
				{
					//��zֵ��ߵĵ㣬����ߵ㴦����ƽ����Ƭ���õ�������
					pcl::PointXYZ min_pt, max_pt;
					pcl::getMinMax3D(*fliter_outlier_point_cloud, min_pt, max_pt);  // ֱ�ӻ�ȡXYZ��ֵ��
********************���������X��Y��Z�������ϵ���Сֵ�����ֵ��
					//������ֵ�����ֱͨ�˲����õ���ͷ����
					// ����ֱͨ�˲�������
					pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_head(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PassThrough<pcl::PointXYZ> pass_head;
				******************ֱͨ�˲���������ָ���ֶΣ�������Z�ᣩ����ֵ��Χ�����˵��ơ�
					// �����˲�������
					pass_head.setInputCloud(fliter_outlier_point_cloud);
					pass_head.setFilterFieldName("z");  // ���ù����ֶ�ΪZ��
					pass_head.setFilterLimits(max_pt.z - 0.5, max_pt.z); // ����Zֵ��0.5-2.0�׷�Χ�ڵĵ�
					pass_head.setNegative(false); // false������Χ�ڣ�true������Χ��
					pass_head.filter(*filtered_cloud_head);

					Eigen::Vector4d centroid_head;
					pcl::compute3DCentroid(*filtered_cloud_head, centroid_head);
                    *********************������Ƶ�3D���ģ��������ģ������� = ��(������)/����
					centroid_head = transformation_matrix_inv * centroid_head;

                    ************����������ӱ任�������ϵת����ԭʼ����ϵ
					// ����ֱͨ�˲�������
					pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PassThrough<pcl::PointXYZ> pass0;
					//centroid_head = transformation_matrix_inv * centroid_head;
					// �����˲�������
					pass0.setInputCloud(fliter_outlier_point_cloud);
					pass0.setFilterFieldName("z");  // ���ù����ֶ�ΪZ��
					pass0.setFilterLimits(max_pt.z - 2.8, max_pt.z-1.0); // ����Zֵ��0.5-2.0�׷�Χ�ڵĵ�
					pass0.setNegative(false); // false������Χ�ڣ�true������Χ��
					pass0.filter(*filtered_cloud_temp);
**************************************�ٴ�ʹ��ֱͨ�˲����������Ϊ����ȡ�����ϲ��ĵ���
					if (filtered_cloud_temp->size() > 50)
					{
						//�Ƚ���ƽ����ң�ȷ�����ķ���
					// ���㷨����
					// 2. ���㷨����
                    *********************���������� (Normal Estimation)
						pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
						pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
						pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
						ne.setInputCloud(filtered_cloud_temp);
						ne.setSearchMethod(tree);
						ne.setKSearch(200);
						ne.compute(*normals);//normals �洢ÿ����ķ�����(nx, ny, nz
**************************************************��XYZ�����뷨�������ƺϲ�
						// 3. �������������ĵ���
						pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
						pcl::concatenateFields(*filtered_cloud_temp, *normals, *cloud_with_normals);
*************??����??��������������������
??�������??��
ConditionAnd���߼�AND�������
FieldComparison���ֶαȽ���
"normal_z"��Ŀ���ֶΣ�������Z������
pcl::ComparisonOps::GT�����ڲ�����
0.8����ֵ����
pcl::ComparisonOps::LT��С�ڲ�����
1.0����ֵ����
??��������??��
����������Z������(0.8, 1.0)��Χ�ڵĵ�
��Ӧ��������Z��н�С��36.87�㣨cos??(0.8)��36.87�㣩
Ŀ������ȡ����ˮƽ�ĳ�������

						// 4. ���÷�����������������ȡZ�᷽��0.8-1.0��ƽ�棩
						pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointNormal>());
						range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
							new pcl::FieldComparison<pcl::PointNormal>("normal_z", pcl::ComparisonOps::GT, 0.8)));
						range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
							new pcl::FieldComparison<pcl::PointNormal>("normal_z", pcl::ComparisonOps::LT, 1.0)));

						// 5. ִ�������˲�
						pcl::ConditionalRemoval<pcl::PointNormal> filter;
						filter.setCondition(range_cond);
						filter.setInputCloud(cloud_with_normals);

						pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>);
						filter.filter(*filtered);
						pcl::PointCloud<pcl::PointXYZ>::Ptr extract_point_cloud_car(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::copyPointCloud(*filtered, *extract_point_cloud_car);
**********************??����??����PointNormal����ת��PointXYZ
??����??��
filtered��������ƣ�����������
extract_point_cloud_car��������ƣ������꣩

***************************2. ŷʽ����
						pcl::search::KdTree<pcl::PointXYZ>::Ptr tree0(new pcl::search::KdTree<pcl::PointXYZ>);
						tree0->setInputCloud(extract_point_cloud_car);

						std::vector<pcl::PointIndices> cluster_indices;
						pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
						ec.setClusterTolerance(0.2); // 2cm�������
						ec.setMinClusterSize(500);    // ��С����
						ec.setMaxClusterSize(50000);  // ������
						ec.setSearchMethod(tree0);
						ec.setInputCloud(extract_point_cloud_car);
						ec.extract(cluster_indices);

						// 3. ����ÿ�������͹�����
						int max_area_label = 0;
						int max_area = 0.0;
						std::vector<int> label;
						std::vector<int> area;
						std::vector<double> height;
						Eigen::Vector4d centroid_car;
						for (size_t i = 0; i < cluster_indices.size(); ++i)
						{
                            ****************************�����ж���䣬����ɸѡ���������ľ���أ�cluster����
                            1������ͷ�����뵱ǰ����������Z�᷽��ĸ߶Ȳ����0.8�ס�
                            2����ͷ�����뵱ǰ����������XYƽ���ϵ�ˮƽ�������1.0�ס�
							pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
							for (const auto& idx : cluster_indices[i].indices)
							{
								cluster->push_back((*extract_point_cloud_car)[idx]);
							}
							Eigen::Vector4d centroid_car_temp;
							pcl::compute3DCentroid(*cluster, centroid_car_temp);**********************?������ƴص���ά�������ģ����ģ�??
							if (centroid_head[2] - centroid_car_temp[2] > 0.8 && sqrt((centroid_head[0] - centroid_car_temp[0]) * (centroid_head[0] - centroid_car_temp[0]) + (centroid_head[1] - centroid_car_temp[1]) * (centroid_head[1] - centroid_car_temp[1])) > 1.0)
							{
								label.push_back(int(i));
								area.push_back(cluster->size());
								height.push_back(centroid_car_temp[2]);
							}
						}
***********************************?**���ڶ����ѡ������
??ѡ�����??��ѡ��߶���ߵĴ�

						if (label.size() == 1)
						{
							max_area = area[0];
							max_area_label = label[0];
						}
						else if (label.size() > 1)
						{
							double max_val = height[0];
							max_area_label = label[0];
							for (int k = 1; k < height.size(); k++)
							{
								if (height[k] > max_val)
								{
									max_val = height[k];
									max_area_label = k;
								}
							}
						}
						else
						{
							flag = 0;
						}


						//���㳵�������ĵ�
						pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_car(new pcl::PointCloud<pcl::PointXYZ>);
						for (const auto& idx : cluster_indices[max_area_label].indices)
						{
							cluster_car->push_back((*extract_point_cloud_car)[idx]);
						}
						pcl::compute3DCentroid(*cluster_car, centroid_car);//���������Ĳ�ת������ϵ
						centroid_car = transformation_matrix_inv * centroid_car;
						//��ȡ����ƽ���ϲ��ĵ���
						//�������ƽ��
						// 2. RANSACƽ�����
						pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
						pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
						pcl::SACSegmentation<pcl::PointXYZ> seg;
						seg.setOptimizeCoefficients(true);
						seg.setModelType(pcl::SACMODEL_PLANE);
						seg.setMethodType(pcl::SAC_RANSAC);
						seg.setMaxIterations(1000);
						seg.setDistanceThreshold(0.05);  // ƽ�������ֵ1cm
						seg.setInputCloud(cluster_car);
						seg.segment(*inliers, *coefficients);****
coefficients��ƽ�淽��ϵ��(A,B,C,D)����ӦAx+By+Cz+D=0
inliers��ƽ���ڵ�����**********
                        ********************
�ӳ���������ȡ��ϳ���ƽ����ƣ��������棩
						// 3. ��ȡƽ�����
						pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_car(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::ExtractIndices<pcl::PointXYZ> extract;
						extract.setIndices(inliers);
						extract.setNegative(false);
						extract.setInputCloud(cluster_car);
						extract.filter(*plane_cloud_car);

						// 4. ����ƽ���ϲ�����
						pcl::PointCloud<pcl::PointXYZ>::Ptr upper_cloud(new pcl::PointCloud<pcl::PointXYZ>);
						float A = coefficients->values[0];
						float B = coefficients->values[1];
						float C = coefficients->values[2];
						float D = coefficients->values[3];

						for (const auto& point : *filtered_cloud_temp)
						{
							float distance = A * point.x + B * point.y + C * point.z + D;
							//float distance_point_to_car_center = sqrt( (centroid_car[0] - point.x)*(centroid_car[0] - point.x) + (centroid_car[1] - point.y) * (centroid_car[1] - point.y) + (centroid_car[2] - point.z) * (centroid_car[2] - point.z));
							if (distance > -0.4 && distance < 0.65)
							{  // ������ֵ1cm
								upper_cloud->push_back(point);
							}
						}
??����??��������ϵ�ƽ�淽�̣���ȡ�����ϲ��������
??ԭ��??������ÿ���㵽ƽ��ľ��룬������[-0.4m, 0.65m]��Χ�ڵĵ�
??�������??�������Χ�����˳����ĵ��͸߶ȣ����ص����أ�
						// ���㳵ͷ���ĵ��ڳ���ƽ���ͶӰ
						float t = (A * centroid_head[0] + B * centroid_head[1] + C * centroid_head[2] + D) /
							(A * A + B * B + C * C);

						// ����ͶӰ������    ���㳵ͷ�ڳ���ƽ���ͶӰ
						pcl::PointXYZ projected;
						projected.x = centroid_head[0] - A * t;
						projected.y = centroid_head[1] - B * t;
						projected.z = centroid_head[2] - C * t;
						pcl::transformPointCloud(*upper_cloud, *upper_cloud, transformation_matrix_inv);
						//���㳵ͷ���Ƶ�����(Xc_car_head, Yc_car_head, Zc_car_head)
						double xc;
						double yc;
						double zc;
						xc = centroid_car[0];
						yc = centroid_car[1];
						zc = centroid_car[2];

						// ���ó�ͷ���ĵ�����ͳ������ĵ����꣬���㳵��z�����ת�Ƕ�
						double dx = double(projected.x) - xc;
						double dy = double(projected.y) - yc;
						double dz = double(projected.z) - zc;

						// ��������ģ��
						double magnitude = sqrt(dx * dx + dy * dy);
						if (magnitude < 1e-6) {  // ���������
							std::cerr << "Error: Zero length vector" << std::endl;
							return ;
						}

						// ������x��(1,0,0)�ĵ��
						double dot_product = dx;  // 1*dx + 0*dy + 0*dz

						// ����н�����ֵ��ת��Ϊ�Ƕ�
						double cos_theta = dot_product / magnitude;
						double angle = acos(cos_theta) * 180.0 / PI;
						std::cout << "angle: "
							<< angle << std::endl;
??����??��ͨ����ͷͶӰ���복�����ĵ��������㳵������
??ԭ��??��
���㳵ͷͶӰ���복�����ĵ����� 
d
 =(dx,dy)
�����������X��(1,0)�ļнǣ�

����Ƕ�ֵ��0��~180�㣩
						
						CreateCarTemplate();
						if (model_car == 1)
						{
							template_point_cloud = model1_template_point_cloud;
						}
						else if (model_car == 2)
						{
							template_point_cloud = model2_template_point_cloud;
						}
						else
						{
							template_point_cloud = model3_template_point_cloud;
						}
						////���е���ƥ��
						pcl::PointCloud<pcl::PointXYZ> temp_Source;
						temp_Source = *template_point_cloud;
						m_template_point_cloud_trans = temp_Source.makeShared();

						Eigen::Matrix4d initial_transformation_matrix = Eigen::Matrix4d::Identity();

						initial_transformation_matrix = m2M(0.0, 0.0, angle, xc, yc, zc + 0.5); //��һ֡���ݽǶ�
						pcl::transformPointCloud(*template_point_cloud, *m_template_point_cloud_trans, initial_transformation_matrix);
						// ���ӻ�
						//pcl::visualization::PCLVisualizer viewer_upper_cloud("upper_cloud");
						//viewer_upper_cloud.addPointCloud<pcl::PointXYZ>(upper_cloud, "original", 0);
						//viewer_upper_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "original");
						//viewer_upper_cloud.spin();

						int iterNum = 100;
						Matrix4d transformation;
						ICPMatch(m_template_point_cloud_trans, upper_cloud, iterNum, transformation);
                        *******************������׼�ĺ����㷨������������㣨ICP���㷨�����ڼ�����������֮������ſռ�任��ϵ��
						transformation_matrix = transformation * initial_transformation_matrix;

						
						print4x4Matrix(transformation_matrix, Dis, rollCurrent, pitchCurrent, yawCurrent);
						std::cout<<"---***result:***---"<< "\nDis:\n" << Dis<<"--------------------------------debug--------------------"<<std::endl;
						flag = 1;
						
						//pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_trans;
						//point_cloud_trans = template_point_cloud;
						//pcl::transformPointCloud(*template_point_cloud, *point_cloud_trans, transformation_matrix);
						pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_trans;
						point_cloud_trans = template_point_cloud;
						pcl::transformPointCloud(*template_point_cloud, *point_cloud_trans, transformation_matrix);
						//DisplayPointCloud(point_cloud_trans, upper_cloud);

						//���㳵�������2015.7.5
						Eigen::Matrix4d inverse_transformation_matrix = transformation_matrix.inverse();
						point_cloud_trans = upper_cloud;
						pcl::transformPointCloud(*upper_cloud, *point_cloud_trans, inverse_transformation_matrix);
						
						//��ȡ�����ĸ���ĵ��ƣ�������ƽ����ϣ�Ȼ�������Ϻ�ƽ��֮��ľ���
						//����ֱͨ�˲���ȡ��1�棬��2�棬��3�棬��4��
						// ��������ֱͨ�˲���(X/Y/Z��)
						pcl::PointCloud<pcl::PointXYZ>::Ptr left_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr up_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr right_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr down_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

						ExtractPlanePointCloud(point_cloud_trans, float((-0.5) * m_length_car_body - 0.4), float((-0.5) * m_length_car_body + 0.15), float((-0.5) * m_width_car_body + 0.2), float(0.5 * m_width_car_body - 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), left_plane_point_cloud);
						ExtractPlanePointCloud(point_cloud_trans, float((-0.5) * m_length_car_body + 0.3), float((0.5) * m_length_car_body - 0.3), float((0.5) * m_width_car_body - 0.2), float(0.5 * m_width_car_body + 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), up_plane_point_cloud);
						ExtractPlanePointCloud(point_cloud_trans, float((0.5) * m_length_car_body - 0.4), float((0.5) * m_length_car_body + 0.4), float((-0.5) * m_width_car_body + 0.2), float(0.5 * m_width_car_body - 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), right_plane_point_cloud);
						ExtractPlanePointCloud(point_cloud_trans, float((-0.5) * m_length_car_body + 0.4), float((0.5) * m_length_car_body - 0.4), float((-0.5) * m_width_car_body - 0.2), float(-0.5 * m_width_car_body + 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), down_plane_point_cloud);
						

						//���ƽ��
						// 2. RANSACƽ�����
						pcl::ModelCoefficients::Ptr coefficients_plane_left(new pcl::ModelCoefficients);
						pcl::ModelCoefficients::Ptr coefficients_plane_up(new pcl::ModelCoefficients);
						pcl::ModelCoefficients::Ptr coefficients_plane_right(new pcl::ModelCoefficients);
						pcl::ModelCoefficients::Ptr coefficients_plane_down(new pcl::ModelCoefficients);

						pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
						pcl::SACSegmentation<pcl::PointXYZ> seg_plane;
						seg_plane.setOptimizeCoefficients(true);
						seg_plane.setModelType(pcl::SACMODEL_PLANE);
						seg_plane.setMethodType(pcl::SAC_RANSAC);
						seg_plane.setMaxIterations(1000);
						seg_plane.setDistanceThreshold(0.05);  // ƽ�������ֵ1cm

						seg_plane.setInputCloud(left_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_left);

						seg_plane.setInputCloud(up_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_up);

						seg_plane.setInputCloud(right_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_right);

						seg_plane.setInputCloud(down_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_down);

						//������1���ϵ㵽��3��ƽ��ľ��룬����

						m_length_car_body = fabs(coefficients_plane_right->values[3] - coefficients_plane_left->values[3]);
						m_width_car_body = fabs(coefficients_plane_down->values[3] - coefficients_plane_up->values[3]);
						std::cout << "m_length_car_body:" << m_length_car_body << std::endl;
						std::cout << "m_width_car_body:" << m_width_car_body << std::endl;

��ʼ�任�����ݳ�������λ�úͳ�ʼ�Ƕȣ�������ģ����Ʊ任������λ�á�
ICP��׼��ʹ��ICP�㷨��ģ��������ʵ�ʵ�������ȡ�ĳ����ϲ����ƽ�����׼���õ�����ȷ�ı任����
�������ձ任������ʼ�任��ICP�õ��ı任��ϣ��õ���ģ�嵽ʵ�ʵ��Ƶ������任����
�任���ƣ���ʵ�ʳ������ƣ�upper_cloud��ͨ�����ձ任�������任���任��ģ������ϵ�¡�
��ȡ�������ĸ�������ƣ����ϡ��ҡ��£���
��ÿ������ĵ��ƽ���ƽ����ϡ�
ͨ������ƽ��֮��ľ�����㳵���ĳ��Ϳ�
						// ���ӻ�
						//pcl::visualization::PCLVisualizer viewer4("ExtractSlices2");
						//viewer4.addPointCloud<pcl::PointXYZ>(left_plane_point_cloud, "original", 0);
						//viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "original");

						//viewer4.addPointCloud<pcl::PointXYZ>(up_plane_point_cloud, "original1", 0);
						//viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "original1");

						//viewer4.addPointCloud<pcl::PointXYZ>(right_plane_point_cloud, "original2", 0);
						//viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "original2");

						//viewer4.addPointCloud<pcl::PointXYZ>(down_plane_point_cloud, "original3", 0);
						//viewer4.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 0, "original3");
						//viewer4.spin();
					}
					else
					{
						flag = -1;
					}					
				}
				else
				{
					flag = -1;
				}				
			}
			else
			{
				flag = -1;
			}			
		}
		else
		{
			flag = -1;
		}
		
	}
	else
	{
		flag = -1;
	}

	//DisplayPointCloud(point_cloud_trans, upper_cloud);

	std::cout<<"---***result:***---"<< "\nDis:\n" << Dis << "\nrollCurrent:" << rollCurrent 
	<< "\npitchCurrent:"<< pitchCurrent<< "\nyawCurrent:" << yawCurrent << std::endl;
	Vector3d Dis_a;
	Dis_a[0] = 0.94493859  * Dis[0] + 0.01980631 * Dis[1] + 0.32664782 * Dis[2] + 1.066995347;
	Dis_a[1] = -0.03302529 * Dis[0] + 0.99884216 * Dis[1] + 0.0349713  * Dis[2] + 1.34183012;
	Dis_a[2] = -0.32557715 * Dis[0] -0.04383366   * Dis[1] + 0.944499 * Dis[2] + 3.25196206;
	
	Vector3d Dis_b;
	Dis_b[0] = rz_2d_array[0][0]  * Dis_a[0] + rz_2d_array[0][1] * Dis_a[1] + rz_2d_array[0][2] * Dis_a[2];
	Dis_b[1] = rz_2d_array[1][0] * Dis_a[0] + rz_2d_array[1][1] * Dis_a[1] + rz_2d_array[1][2] * Dis_a[2] ;
	Dis_b[2] = rz_2d_array[2][0] * Dis_a[0] +rz_2d_array[2][1]  * Dis_a[1] + rz_2d_array[2][2] * Dis_a[2] ;
	// dis_pub.publish(dis_msg);

	// ==== ��������̬�任�Ĺؼ��޸Ĳ��� =====
	// ���������4x4�任����T_matrix��
	Eigen::Matrix4d T_matrix;
	T_matrix << 
	0.94493859  ,0.01980631  ,0.32664782 ,1.066995347,
	-0.03302529 , 0.99884216 , 0.0349713  , 1.34183012,
	-0.32557715, -0.04383366 , 0.94449923 , 3.25196206,
	0.       ,   0.     ,     0.      ,    1.    ;
	T_matrix =  Rz* T_matrix;
	// ��ȡԭʼICP�任�������ת���֣�3x3��
	Eigen::Matrix3d R_icp = transformation_matrix.topLeftCorner<3,3>();
	// ��ȡ�����任�������ת����
	Eigen::Matrix3d R_given = T_matrix.topLeftCorner<3,3>();

	// ���㸴����ת����: R_new = R_given * R_icp
	Eigen::Matrix3d R_new = R_given * R_icp;

	// ����һ��4x4�ı任�������ڷֽ�ŷ����
	Eigen::Matrix4d T_new = Eigen::Matrix4d::Identity();
	T_new.topLeftCorner<3,3>() = R_new;

	// �ֽ⸴����ת����õ��µ�ŷ����
	double new_roll, new_pitch, new_yaw;
	Eigen::Vector3d temp_dis; // �������ǲ�����ƽ�ƣ����Դ���һ����ʱ����
	print4x4Matrix(T_new, temp_dis, new_roll, new_pitch, new_yaw);
	// ===== ��̬�任�޸Ĳ��ֽ��� =====

	// geometry_msgs::Vector3 att_msg;
	// att_msg.x = new_roll;  // ʹ�ñ任���roll
	// att_msg.y = new_pitch; // ʹ�ñ任���pitch
	// att_msg.z = new_yaw;   // ʹ�ñ任���yaw
	// angles_pub.publish(att_msg);

	std::cout<<"---result_new:----"<< "\nDis_b:\n" << Dis_b << "\nrollCurrent:" << new_roll 
	<< "\npitchCurrent:"<< new_pitch<< "\nyawCurrent:" << new_yaw << std::endl;


	UDP_ros_pkg::Dis combined_msg;
	if(Dis[0]==0)
	{
	// ���λ������
		std::cout<<"get 0 data\n";
		combined_msg.is_ready=false;
		combined_msg.displacement.x =0;
		combined_msg.displacement.y = 0;
		combined_msg.displacement.z = 0;

		// �����̬����
		combined_msg.attitude.x = 0;
		combined_msg.attitude.y = 0;
		combined_msg.attitude.z = 0;
		combined_msg.height=0;
		combined_msg.width=0;
		combined_msg.length=0;
		// �����ϲ������Ϣ
		if (catch_count!= 0)
			{
				dis_and_att_pub.publish(combined_msg);
			}
	}
	else    
	{
		combined_msg.is_ready=true;
		combined_msg.displacement.x = Dis_b[0];
		combined_msg.displacement.y = Dis_b[1];
		combined_msg.displacement.z = Dis_b[2];

		// �����̬����
		combined_msg.attitude.x = new_roll;
		combined_msg.attitude.y = new_pitch;
		combined_msg.attitude.z = new_yaw;
		combined_msg.height=m_height_car_body;
		combined_msg.width=m_width_car_body;
		combined_msg.length=m_length_car_body;
		printf("msg->data.height:%f\n",m_height_car_body);
        printf("msg->data.height:%f\n",m_width_car_body);
        printf("msg->data.height:%f\n",m_length_car_body);
		// �����ϲ������Ϣ
		if (catch_count>0)
			{
				dis_and_att_pub.publish(combined_msg);
			}
	}

	ROS_INFO("end ---------------------" );
	model=1;
	catch_count= catch_count-1;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
 if(sig_strat==1)
  {
	if(1)//���ۻ� no 
	{
		pcl::fromROSMsg(*input, *cloud);
		total_process(cloud);
		ROS_INFO("------start ----------------------- \n Point cloud received: %zu points", cloud->points.size());
		
	}
  }
}
int main(int argc ,char **argv) 
{

	ros::Subscriber sub = nh.subscribe("/merged_lidar_points", 2, cloud_cb);

	ros::Subscriber udp_sub = nh.subscribe("/from_udp", 3, udpMsgCallback);
	ros::Subscriber sig_sub=nh.subscribe("/sig_topic",10,change_sig);
	dis_and_att_pub = nh.advertise<UDP_ros_pkg::Dis>("/dis_and_att", 2);

	ros::Rate loop_rate(10); // 10Hz
	while (ros::ok())
	{
	ros::spinOnce();
	loop_rate.sleep();
	}

	return 0;
}
```