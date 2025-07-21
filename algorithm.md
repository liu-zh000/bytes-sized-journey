# PCL（点云库）& Eigen（线性代数库)

PCL（点云处理）、Eigen（矩阵运算）、ROS（消息传递）
## 2222

```CPP
点云滤波??：半径滤波、直通滤波、半径离群点去除。
??点云分割??：欧式聚类、平面拟合（RANSAC）。
??点云配准??：ICP算法，用于将模板点云与目标点云对齐。
??坐标变换??：使用Eigen进行矩阵变换。
??姿态表示??：使用欧拉角（roll, pitch, yaw）表示自卸车的姿态。
??几何计算??：通过质心、向量夹角等计算车斗方向。
std::mutex mtx;
std::condition_variable cv;
bool udp_data_ready = false; 
uint8_t g_type = 0;
double g_angle = 0.0;
int model_car = 1; //到时候通讯传过来车的型号
int model = 1; //到时候通讯传过来车的型号
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
// 平面方程结构体
struct PlaneEquation {
	float a, b, c, d;  // Ax+By+Cz+D=0
};

// 计算单个点到平面的距离
double computePointToPlaneDistance(const pcl::PointXYZ& point, const pcl::ModelCoefficients::Ptr& plane) 
{
	const double a = plane->values[0];
	const double b = plane->values[1];
	const double c = plane->values[2];
	const double d = plane->values[3];
	return std::fabs(a * point.x + b * point.y + c * point.z + d) / std::sqrt(a * a + b * b + c * c);
}


//提取平面与点云的交集
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
	/* 声明 ICP 对象  */
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

	/*输入 设置 源点云*/
	gicp.setInputSource(source);

	/*输入 设置 目标点云*/
	gicp.setInputTarget(target);
	/*有4个参数可以设置 */
		// 设置最大的对应关系距离为 多少 单位m， （对应距离大于此距离将被忽略） 
	gicp.setMaxCorrespondenceDistance(0.5);
	// 设置最大迭代次数(默认 1)
	gicp.setMaximumIterations(maxIterNum);
	// 设置 两次变换矩阵之间的差值 (默认 2)
	gicp.setTransformationEpsilon(1e-8);
	// 设置 均方误差(默认 3)
	gicp.setEuclideanFitnessEpsilon(0.01);
	//icp.setUseReciprocalCorrespondences(true);//设置为true,则使用相互对应关系

	/* 声明 经过配准变换 源点云 后 的 点云 */
	pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
	/*执行配准  存储变换后的源点云到 Final 上  */
	gicp.align(*final);


	/*声明 最终的变换矩阵*/
	Eigen::Matrix4f transformation;
	//赋值旋转矩阵
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
//**************************用于从4x4齐次变换矩阵中提取位移和欧拉角（roll, pitch, yaw），是机器人学和3D视觉中的核心工具。
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

// 计算绕X轴的旋转矩阵
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

// 计算绕Y轴的旋转矩阵
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

// 计算绕Z轴的旋转矩阵
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
	// 欧拉角（弧度制）
	//cout << angle1 << angle2 << angle3 << endl;
	double anglex = angle1 * PI / 180.0;
	double angley = angle2 * PI / 180.0;
	double anglez = angle3 * PI / 180.0;

	// 计算旋转矩阵
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
	Eigen::Vector4f centroid_s, centroid_t;					// 质心
	// 齐次坐标，（c0,c1,c2,1）
	pcl::compute3DCentroid(*target, centroid_t);
	Eigen::Vector3f Distance;
	Distance(0) = centroid_t(0);
	Distance(1) = centroid_t(1);
	Distance(2) = centroid_t(2);
	return Distance;
}


void pc_normalize(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target)
{
	//// 获取点云的行数
	//int l = pc.rows();

	// 计算点云的质心
	Eigen::Vector4f centroid_s, centroid_t;					// 质心
	pcl::compute3DCentroid(*source, centroid_s);	// 齐次坐标，（c0,c1,c2,1）
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
	//利用图像识别车型，根据车型可得到车斗的长宽高（单位：米）
	float model1_length_car_body = 3.5;
	float model1_width_car_body = 2.0;
	float model1_height_car_body = 0.5;

	//根据车斗尺寸创建模板,该部分可离线完成
	pcl::PointCloud<pcl::PointXYZ>& cloud = *model1_template_point_cloud;
	float point_distance = 0.05;

	int template_point_cloud_width = model1_length_car_body / point_distance;  // 每行200个点
	int template_point_cloud_height = model1_width_car_body / point_distance; // 共100行
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
	//添加底部平面点云
	for (int x = 0; x < template_point_cloud_width; ++x)
	{
		for (int y = 0; y < template_point_cloud_height; ++y)
		{
			cloud.points.push_back(pcl::PointXYZ{ float(-1.75 + float(x) * point_distance), float(-1.0 + float(y) * point_distance), -float(0.5) });
		}
	}
}

//2025.7.5新增加函数
void ExtractPlanePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_1_ptr, float x_low, float x_up, float y_low, float y_up, float z_low, float z_up, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_2_ptr)
{
	
	// X轴范围过滤
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_x_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_y_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::PassThrough<pcl::PointXYZ> pass_x;
	pass_x.setInputCloud(point_cloud_1_ptr);
	pass_x.setFilterFieldName("x");
	pass_x.setFilterLimits(x_low, x_up); // 设置长方体X轴范围
	pass_x.setNegative(false);
	pass_x.filter(*point_cloud_x_ptr);

	// Y轴范围过滤
	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud(point_cloud_x_ptr);
	pass_y.setFilterFieldName("y");
	pass_y.setFilterLimits(y_low, y_up); // 设置长方体Y轴范围
	pass_y.filter(*point_cloud_y_ptr);

	// Z轴范围过滤
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud(point_cloud_y_ptr);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(z_low, z_up); // 设置长方体Z轴范围
	pass_z.filter(*point_cloud_2_ptr);
}

float car_center_x_by_img = 5.47;
float car_center_y_by_img = -0.2;
float car_center_z_by_img = -0.3;
int flag = 0; //1表示有结果返回，0表示提取车斗失败，-1表示在2米-15米范围内提取车体点云失败



void total_process(pcl::PointCloud<pcl::PointXYZ>::Ptr origin_point_cloud)
{
	//pcl::PointXYZ origin(0, 0, 0);
	pcl::PointXYZ origin = origin_point_cloud->points.back();
	std::cout << "minpoint(中心点)："<< origin << std::endl;
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



	//新增
	double m_length_car_body = 0.0;
	double m_width_car_body = 0.0;
	double m_height_car_body = 0.0;

	//2025.7.5新增加
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
////2025.7.5新增加结束

	// 利用自卸车到挖机的最大距离范围进行半径欧式距离滤波
	pcl::PointXYZ origin1(0, 0, 0);
	***********************pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	**将原始点云组织成树状结构********************kdtree.setInputCloud(origin_point_cloud);

	// 半径搜索获取距离阈值内点
	std::vector<int> indices;
	std::vector<float> distances;
    **********************indices: 输出参数，存储落在半径内的点在输入点云中的索引
*********************distances: 输出参数，存储这些点到中心点的欧氏距离的平方
	**************************kdtree.radiusSearch(origin1, float(car_center_x_by_img+6.0), indices, distances);

	// 提取结果点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_origin_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	***************************pcl::copyPointCloud(*origin_point_cloud, indices, *filtered_origin_point_cloud);

	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
	transformation_matrix = m2M(0.0, 17.0, 0.0, 0.0, 0.0, 0.0); //第一帧数据角度
	Eigen::Matrix4d transformation_matrix_inv = Eigen::Matrix4d::Identity();
	transformation_matrix_inv = m2M(0.0, -17.0, 0.0, 0.0, 0.0, 0.0); //第一帧数据角度
	先以雷达原点为中心，图像中挖出来的点云的中点x坐标+6为半径过滤掉多余的点云，然后复制出来进行变换（绕y轴转17度），KdTreeFLANN 是 PCL（Point Cloud Library）中用于快速最近邻搜索的一个类，特别适用于处理点云数据。其核心是基于 FLANN（Fast Library for Approximate Nearest Neighbors）库实现的 kd-tree 数据结构。**************************pcl::transformPointCloud(*filtered_origin_point_cloud, *filtered_origin_point_cloud, transformation_matrix);

	if (filtered_origin_point_cloud->size() > 100)
	{
		// 2. 设置距离阈值（以原点(0,0,0)为参考点）
		float distance_threshold = 2.0; // 单位：米
		// 3. 创建距离过滤后的点云容器
		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		// 4. 遍历点云进行距离过滤
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
			//利用自卸车上点进行欧式距离滤波

			pcl::PointXYZ origin_car(car_center_x_by_img, car_center_y_by_img, car_center_z_by_img);
			
			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_car;
			kdtree_car.setInputCloud(filtered_cloud);

			// 半径搜索获取距离阈值内点
			kdtree_car.radiusSearch(origin_car, float(6.0), indices, distances);
    ***********************再次进行半径搜索
			// 提取结果点云
			pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
			if (indices.size() > 100)
			{
				pcl::copyPointCloud(*filtered_cloud, indices, *filtered_point_cloud);

***********************按照索引拷贝到新点云
				//对滤波后的点进行车头查找
			    //半径离群点去除
				pcl::PointCloud<pcl::PointXYZ>::Ptr fliter_outlier_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::RadiusOutlierRemoval<pcl::PointXYZ> fliter_outlier_point;
				fliter_outlier_point.setInputCloud(filtered_point_cloud);
				fliter_outlier_point.setRadiusSearch(0.5);     // 搜索半径0.1单位
				fliter_outlier_point.setMinNeighborsInRadius(50); // 半径内至少5个点
				fliter_outlier_point.filter(*fliter_outlier_point_cloud);
**********************************半径离群点去除（噪声）
				if (fliter_outlier_point_cloud->size() > 0)
				{
					//找z值最高的点，从最高点处进行平面切片，得到车点云
					pcl::PointXYZ min_pt, max_pt;
					pcl::getMinMax3D(*fliter_outlier_point_cloud, min_pt, max_pt);  // 直接获取XYZ最值点
********************计算点云在X、Y、Z三个轴上的最小值和最大值。
					//利用最值点进行直通滤波，得到车头坐标
					// 创建直通滤波器对象
					pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_head(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PassThrough<pcl::PointXYZ> pass_head;
				******************直通滤波器，根据指定字段（这里是Z轴）的数值范围来过滤点云。
					// 设置滤波器参数
					pass_head.setInputCloud(fliter_outlier_point_cloud);
					pass_head.setFilterFieldName("z");  // 设置过滤字段为Z轴
					pass_head.setFilterLimits(max_pt.z - 0.5, max_pt.z); // 保留Z值在0.5-2.0米范围内的点
					pass_head.setNegative(false); // false保留范围内，true保留范围外
					pass_head.filter(*filtered_cloud_head);

					Eigen::Vector4d centroid_head;
					pcl::compute3DCentroid(*filtered_cloud_head, centroid_head);
                    *********************计算点云的3D质心（几何中心）。质心 = Σ(点坐标)/点数
					centroid_head = transformation_matrix_inv * centroid_head;

                    ************将质心坐标从变换后的坐标系转换回原始坐标系
					// 创建直通滤波器对象
					pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PassThrough<pcl::PointXYZ> pass0;
					//centroid_head = transformation_matrix_inv * centroid_head;
					// 设置滤波器参数
					pass0.setInputCloud(fliter_outlier_point_cloud);
					pass0.setFilterFieldName("z");  // 设置过滤字段为Z轴
					pass0.setFilterLimits(max_pt.z - 2.8, max_pt.z-1.0); // 保留Z值在0.5-2.0米范围内的点
					pass0.setNegative(false); // false保留范围内，true保留范围外
					pass0.filter(*filtered_cloud_temp);
**************************************再次使用直通滤波器，这次是为了提取车斗上部的点云
					if (filtered_cloud_temp->size() > 50)
					{
						//先进行平面查找，确定车的方向
					// 计算法向量
					// 2. 计算法向量
                    *********************法向量估计 (Normal Estimation)
						pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
						pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
						pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
						ne.setInputCloud(filtered_cloud_temp);
						ne.setSearchMethod(tree);
						ne.setKSearch(200);
						ne.compute(*normals);//normals 存储每个点的法向量(nx, ny, nz
**************************************************将XYZ点云与法向量点云合并
						// 3. 创建带法向量的点云
						pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
						pcl::concatenateFields(*filtered_cloud_temp, *normals, *cloud_with_normals);
*************??功能??：创建法向量过滤条件
??参数详解??：
ConditionAnd：逻辑AND组合条件
FieldComparison：字段比较器
"normal_z"：目标字段（法向量Z分量）
pcl::ComparisonOps::GT：大于操作符
0.8：阈值下限
pcl::ComparisonOps::LT：小于操作符
1.0：阈值上限
??物理意义??：
保留法向量Z分量在(0.8, 1.0)范围内的点
对应法向量与Z轴夹角小于36.87°（cos??(0.8)≈36.87°）
目的是提取近似水平的车斗底面

						// 4. 设置法向量过滤条件（提取Z轴方向0.8-1.0的平面）
						pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointNormal>());
						range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
							new pcl::FieldComparison<pcl::PointNormal>("normal_z", pcl::ComparisonOps::GT, 0.8)));
						range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
							new pcl::FieldComparison<pcl::PointNormal>("normal_z", pcl::ComparisonOps::LT, 1.0)));

						// 5. 执行条件滤波
						pcl::ConditionalRemoval<pcl::PointNormal> filter;
						filter.setCondition(range_cond);
						filter.setInputCloud(cloud_with_normals);

						pcl::PointCloud<pcl::PointNormal>::Ptr filtered(new pcl::PointCloud<pcl::PointNormal>);
						filter.filter(*filtered);
						pcl::PointCloud<pcl::PointXYZ>::Ptr extract_point_cloud_car(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::copyPointCloud(*filtered, *extract_point_cloud_car);
**********************??功能??：将PointNormal点云转回PointXYZ
??参数??：
filtered：输入点云（带法向量）
extract_point_cloud_car：输出点云（仅坐标）

***************************2. 欧式聚类
						pcl::search::KdTree<pcl::PointXYZ>::Ptr tree0(new pcl::search::KdTree<pcl::PointXYZ>);
						tree0->setInputCloud(extract_point_cloud_car);

						std::vector<pcl::PointIndices> cluster_indices;
						pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
						ec.setClusterTolerance(0.2); // 2cm聚类距离
						ec.setMinClusterSize(500);    // 最小点数
						ec.setMaxClusterSize(50000);  // 最大点数
						ec.setSearchMethod(tree0);
						ec.setInputCloud(extract_point_cloud_car);
						ec.extract(cluster_indices);

						// 3. 计算每个聚类的凸包面积
						int max_area_label = 0;
						int max_area = 0.0;
						std::vector<int> label;
						std::vector<int> area;
						std::vector<double> height;
						Eigen::Vector4d centroid_car;
						for (size_t i = 0; i < cluster_indices.size(); ++i)
						{
                            ****************************条件判断语句，用于筛选符合条件的聚类簇（cluster）。
                            1、：车头质心与当前聚类质心在Z轴方向的高度差大于0.8米。
                            2、车头质心与当前聚类质心在XY平面上的水平距离大于1.0米。
							pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
							for (const auto& idx : cluster_indices[i].indices)
							{
								cluster->push_back((*extract_point_cloud_car)[idx]);
							}
							Eigen::Vector4d centroid_car_temp;
							pcl::compute3DCentroid(*cluster, centroid_car_temp);**********************?计算点云簇的三维几何中心（质心）??
							if (centroid_head[2] - centroid_car_temp[2] > 0.8 && sqrt((centroid_head[0] - centroid_car_temp[0]) * (centroid_head[0] - centroid_car_temp[0]) + (centroid_head[1] - centroid_car_temp[1]) * (centroid_head[1] - centroid_car_temp[1])) > 1.0)
							{
								label.push_back(int(i));
								area.push_back(cluster->size());
								height.push_back(centroid_car_temp[2]);
							}
						}
***********************************?**存在多个候选车斗簇
??选择策略??：选择高度最高的簇

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


						//计算车斗的中心点
						pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_car(new pcl::PointCloud<pcl::PointXYZ>);
						for (const auto& idx : cluster_indices[max_area_label].indices)
						{
							cluster_car->push_back((*extract_point_cloud_car)[idx]);
						}
						pcl::compute3DCentroid(*cluster_car, centroid_car);//车斗簇质心并转换坐标系
						centroid_car = transformation_matrix_inv * centroid_car;
						//提取车斗平面上部的点云
						//首先拟合平面
						// 2. RANSAC平面拟合
						pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
						pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
						pcl::SACSegmentation<pcl::PointXYZ> seg;
						seg.setOptimizeCoefficients(true);
						seg.setModelType(pcl::SACMODEL_PLANE);
						seg.setMethodType(pcl::SAC_RANSAC);
						seg.setMaxIterations(1000);
						seg.setDistanceThreshold(0.05);  // 平面拟合阈值1cm
						seg.setInputCloud(cluster_car);
						seg.segment(*inliers, *coefficients);****
coefficients：平面方程系数(A,B,C,D)，对应Ax+By+Cz+D=0
inliers：平面内点索引**********
                        ********************
从车斗簇中提取拟合出的平面点云（车斗底面）
						// 3. 提取平面点云
						pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_car(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::ExtractIndices<pcl::PointXYZ> extract;
						extract.setIndices(inliers);
						extract.setNegative(false);
						extract.setInputCloud(cluster_car);
						extract.filter(*plane_cloud_car);

						// 4. 计算平面上部点云
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
							{  // 距离阈值1cm
								upper_cloud->push_back(point);
							}
						}
??功能??：基于拟合的平面方程，提取车斗上部区域点云
??原理??：计算每个点到平面的距离，保留在[-0.4m, 0.65m]范围内的点
??设计依据??：这个范围覆盖了车斗的典型高度（空载到满载）
						// 计算车头中心点在车斗平面的投影
						float t = (A * centroid_head[0] + B * centroid_head[1] + C * centroid_head[2] + D) /
							(A * A + B * B + C * C);

						// 计算投影点坐标    计算车头在车斗平面的投影
						pcl::PointXYZ projected;
						projected.x = centroid_head[0] - A * t;
						projected.y = centroid_head[1] - B * t;
						projected.z = centroid_head[2] - C * t;
						pcl::transformPointCloud(*upper_cloud, *upper_cloud, transformation_matrix_inv);
						//计算车头点云的质心(Xc_car_head, Yc_car_head, Zc_car_head)
						double xc;
						double yc;
						double zc;
						xc = centroid_car[0];
						yc = centroid_car[1];
						zc = centroid_car[2];

						// 利用车头中心点坐标和车斗中心点坐标，计算车绕z轴的旋转角度
						double dx = double(projected.x) - xc;
						double dy = double(projected.y) - yc;
						double dz = double(projected.z) - zc;

						// 计算向量模长
						double magnitude = sqrt(dx * dx + dy * dy);
						if (magnitude < 1e-6) {  // 零向量检查
							std::cerr << "Error: Zero length vector" << std::endl;
							return ;
						}

						// 计算与x轴(1,0,0)的点积
						double dot_product = dx;  // 1*dx + 0*dy + 0*dz

						// 计算夹角余弦值并转换为角度
						double cos_theta = dot_product / magnitude;
						double angle = acos(cos_theta) * 180.0 / PI;
						std::cout << "angle: "
							<< angle << std::endl;
??功能??：通过车头投影点与车斗质心的向量计算车斗朝向
??原理??：
计算车头投影点与车斗质心的向量 
d
 =(dx,dy)
计算该向量与X轴(1,0)的夹角：

输出角度值（0°~180°）
						
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
						////进行点云匹配
						pcl::PointCloud<pcl::PointXYZ> temp_Source;
						temp_Source = *template_point_cloud;
						m_template_point_cloud_trans = temp_Source.makeShared();

						Eigen::Matrix4d initial_transformation_matrix = Eigen::Matrix4d::Identity();

						initial_transformation_matrix = m2M(0.0, 0.0, angle, xc, yc, zc + 0.5); //第一帧数据角度
						pcl::transformPointCloud(*template_point_cloud, *m_template_point_cloud_trans, initial_transformation_matrix);
						// 可视化
						//pcl::visualization::PCLVisualizer viewer_upper_cloud("upper_cloud");
						//viewer_upper_cloud.addPointCloud<pcl::PointXYZ>(upper_cloud, "original", 0);
						//viewer_upper_cloud.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "original");
						//viewer_upper_cloud.spin();

						int iterNum = 100;
						Matrix4d transformation;
						ICPMatch(m_template_point_cloud_trans, upper_cloud, iterNum, transformation);
                        *******************点云配准的核心算法——迭代最近点（ICP）算法，用于计算两个点云之间的最优空间变换关系。
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

						//计算车斗长宽高2015.7.5
						Eigen::Matrix4d inverse_transformation_matrix = transformation_matrix.inverse();
						point_cloud_trans = upper_cloud;
						pcl::transformPointCloud(*upper_cloud, *point_cloud_trans, inverse_transformation_matrix);
						
						//提取车斗四个面的点云，并进行平面拟合，然后计算拟合后平面之间的距离
						//利用直通滤波提取左1面，上2面，右3面，下4面
						// 创建三个直通滤波器(X/Y/Z轴)
						pcl::PointCloud<pcl::PointXYZ>::Ptr left_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr up_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr right_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr down_plane_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

						ExtractPlanePointCloud(point_cloud_trans, float((-0.5) * m_length_car_body - 0.4), float((-0.5) * m_length_car_body + 0.15), float((-0.5) * m_width_car_body + 0.2), float(0.5 * m_width_car_body - 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), left_plane_point_cloud);
						ExtractPlanePointCloud(point_cloud_trans, float((-0.5) * m_length_car_body + 0.3), float((0.5) * m_length_car_body - 0.3), float((0.5) * m_width_car_body - 0.2), float(0.5 * m_width_car_body + 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), up_plane_point_cloud);
						ExtractPlanePointCloud(point_cloud_trans, float((0.5) * m_length_car_body - 0.4), float((0.5) * m_length_car_body + 0.4), float((-0.5) * m_width_car_body + 0.2), float(0.5 * m_width_car_body - 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), right_plane_point_cloud);
						ExtractPlanePointCloud(point_cloud_trans, float((-0.5) * m_length_car_body + 0.4), float((0.5) * m_length_car_body - 0.4), float((-0.5) * m_width_car_body - 0.2), float(-0.5 * m_width_car_body + 0.2), float(-1.0 * m_height_car_body + 0.3), float(0.2), down_plane_point_cloud);
						

						//拟合平面
						// 2. RANSAC平面拟合
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
						seg_plane.setDistanceThreshold(0.05);  // 平面拟合阈值1cm

						seg_plane.setInputCloud(left_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_left);

						seg_plane.setInputCloud(up_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_up);

						seg_plane.setInputCloud(right_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_right);

						seg_plane.setInputCloud(down_plane_point_cloud);
						seg_plane.segment(*inliers_plane, *coefficients_plane_down);

						//计算左1面上点到右3面平面的距离，计算

						m_length_car_body = fabs(coefficients_plane_right->values[3] - coefficients_plane_left->values[3]);
						m_width_car_body = fabs(coefficients_plane_down->values[3] - coefficients_plane_up->values[3]);
						std::cout << "m_length_car_body:" << m_length_car_body << std::endl;
						std::cout << "m_width_car_body:" << m_width_car_body << std::endl;

初始变换：根据车斗中心位置和初始角度，将车斗模板点云变换到估计位置。
ICP配准：使用ICP算法将模板点云与从实际点云中提取的车斗上部点云进行配准，得到更精确的变换矩阵。
计算最终变换：将初始变换和ICP得到的变换结合，得到从模板到实际点云的完整变换矩阵。
变换点云：将实际车斗点云（upper_cloud）通过最终变换矩阵的逆变换，变换到模板坐标系下。
提取车斗的四个侧面点云（左、上、右、下）。
对每个侧面的点云进行平面拟合。
通过相邻平面之间的距离计算车斗的长和宽。
						// 可视化
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

	// ==== 以下是姿态变换的关键修改部分 =====
	// 定义给定的4x4变换矩阵（T_matrix）
	Eigen::Matrix4d T_matrix;
	T_matrix << 
	0.94493859  ,0.01980631  ,0.32664782 ,1.066995347,
	-0.03302529 , 0.99884216 , 0.0349713  , 1.34183012,
	-0.32557715, -0.04383366 , 0.94449923 , 3.25196206,
	0.       ,   0.     ,     0.      ,    1.    ;
	T_matrix =  Rz* T_matrix;
	// 提取原始ICP变换矩阵的旋转部分（3x3）
	Eigen::Matrix3d R_icp = transformation_matrix.topLeftCorner<3,3>();
	// 提取给定变换矩阵的旋转部分
	Eigen::Matrix3d R_given = T_matrix.topLeftCorner<3,3>();

	// 计算复合旋转矩阵: R_new = R_given * R_icp
	Eigen::Matrix3d R_new = R_given * R_icp;

	// 构造一个4x4的变换矩阵用于分解欧拉角
	Eigen::Matrix4d T_new = Eigen::Matrix4d::Identity();
	T_new.topLeftCorner<3,3>() = R_new;

	// 分解复合旋转矩阵得到新的欧拉角
	double new_roll, new_pitch, new_yaw;
	Eigen::Vector3d temp_dis; // 这里我们不关心平移，所以传入一个临时变量
	print4x4Matrix(T_new, temp_dis, new_roll, new_pitch, new_yaw);
	// ===== 姿态变换修改部分结束 =====

	// geometry_msgs::Vector3 att_msg;
	// att_msg.x = new_roll;  // 使用变换后的roll
	// att_msg.y = new_pitch; // 使用变换后的pitch
	// att_msg.z = new_yaw;   // 使用变换后的yaw
	// angles_pub.publish(att_msg);

	std::cout<<"---result_new:----"<< "\nDis_b:\n" << Dis_b << "\nrollCurrent:" << new_roll 
	<< "\npitchCurrent:"<< new_pitch<< "\nyawCurrent:" << new_yaw << std::endl;


	UDP_ros_pkg::Dis combined_msg;
	if(Dis[0]==0)
	{
	// 填充位移数据
		std::cout<<"get 0 data\n";
		combined_msg.is_ready=false;
		combined_msg.displacement.x =0;
		combined_msg.displacement.y = 0;
		combined_msg.displacement.z = 0;

		// 填充姿态数据
		combined_msg.attitude.x = 0;
		combined_msg.attitude.y = 0;
		combined_msg.attitude.z = 0;
		combined_msg.height=0;
		combined_msg.width=0;
		combined_msg.length=0;
		// 发布合并后的消息
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

		// 填充姿态数据
		combined_msg.attitude.x = new_roll;
		combined_msg.attitude.y = new_pitch;
		combined_msg.attitude.z = new_yaw;
		combined_msg.height=m_height_car_body;
		combined_msg.width=m_width_car_body;
		combined_msg.length=m_length_car_body;
		printf("msg->data.height:%f\n",m_height_car_body);
        printf("msg->data.height:%f\n",m_width_car_body);
        printf("msg->data.height:%f\n",m_length_car_body);
		// 发布合并后的消息
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
	if(1)//不累积 no 
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