#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vector>
#include <math.h>
#include <iostream>

using namespace sensor_msgs;

// definitions
#define scan_topic "/scan"			//"/RosAria/sim_lms2xx_1_laserscan"			//name of laser topic
#define cloud_topic "/cloud"		//name of pointcloud2 topic
#define ptu_topic "/cmd"			//name of ptu topic
#define height_cloud 150				//heith of pointcloud colocar só numeros pares
#define width_cloud 540
#define resolution 0.5				//laser resolution in degrees/points
#define stack_resolution 1		//resolution betwen two laserscans

//some global variables
std::vector<std::vector<double> > scan_matrix;
bool inicia_movimento = 0;
clock_t start = 0;

class My_Filter {
public:
	My_Filter();
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
	void preencher_pointcloud();
	void mover_ptu(double pan, double tilt);
	double standard_deviation();
private:
	ros::NodeHandle node_;
	laser_geometry::LaserProjection projector_;

	ros::Publisher point_cloud_publisher_;
	ros::Publisher ptu_publisher;
	ros::Subscriber scan_sub_;

	int i;
	float k;
	double max_intensity, min_intensity, mean_intensity, deviation;


};

My_Filter::My_Filter() {
	scan_sub_ = node_.subscribe<sensor_msgs::LaserScan>(
	scan_topic, 3, &My_Filter::scanCallback, this);
	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(
			cloud_topic, 100, false);
	ptu_publisher = node_.advertise<sensor_msgs::JointState>(ptu_topic, 100,
			false);
	i = 0;
	k = -75;
	max_intensity = 0;
	min_intensity = 1000;
	mean_intensity = 0;
	deviation =0;

}

void My_Filter::mover_ptu(double pan, double tilt) {
	sensor_msgs::JointState ptu;
	ptu.header.stamp = ros::Time::now();
	ptu.name.resize(2);
	ptu.position.resize(2);
	ptu.velocity.resize(2);
	ptu.name[0] = "ptu_pan";
	ptu.position[0] = pan;
	ptu.velocity[0] = 0.875;
	ptu.name[1] = "ptu_tilt";
	ptu.position[1] = tilt;
	ptu.velocity[1] = 0.8;

	ptu_publisher.publish(ptu);

}

void My_Filter::preencher_pointcloud() {
	int r = 0, g = 0, b = 0;
	PointCloud2Ptr points_msg = boost::make_shared<PointCloud2>();
	const sensor_msgs::PointCloud2 cloud_out;
	points_msg->header.stamp = ros::Time::now();
	points_msg->header.frame_id = "cloud_frame";
	points_msg->height = height_cloud; // if this is a "full 3D" pointcloud, height is 1; if this is Kinect-like pointcloud, height is the vertical resolution
	points_msg->width = width_cloud;
	points_msg->is_bigendian = false;
	points_msg->is_dense = false; // there may be invalid points

	sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
// this call also resizes the data structure according to the given width, height and fields
	pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

	sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

	for (size_t i = 0; i < scan_matrix.size() - 1;
			++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
		// TODO fill in x, y, z, r, g, b local variables
		//std::cout<<"x: "<<scan_matrix[i][1]<<"y: "<<scan_matrix[i][2]<<"z: "<<scan_matrix[i][3]<< std::endl;
		*iter_x = scan_matrix[i][0];
		*iter_y = scan_matrix[i][1];
		*iter_z = scan_matrix[i][2];
		*iter_r = (scan_matrix[i][3]-mean_intensity-4*deviation)/(8*deviation)*255;
		*iter_g = (scan_matrix[i][3]-mean_intensity-4*deviation)/(8*deviation)*255;	//scan_matrix[i][4] * 10;
		*iter_b = (scan_matrix[i][3]-mean_intensity-4*deviation)/(8*deviation)*255;
	}
	point_cloud_publisher_.publish(points_msg);
}

double My_Filter::standard_deviation(){
	double coiso = 0;
	for(int l = 0 ; l < scan_matrix.size() - 1; l++){
				 coiso = coiso + pow(scan_matrix[l][3]- mean_intensity , 2);
			}
	coiso = sqrt((coiso/(scan_matrix.size() - 2)));
	return coiso;



}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	std::cout << "movimento = "<< inicia_movimento << std::endl;
	//auto begin = chrono::chrono::high_resolution_clock::now();

	double theta, phi, r, x, y, z, intensidade, range;
	std::vector<double> linha;
	sensor_msgs::LaserScan r_laser = *scan;

	//My_Filter::mover_ptu(k * M_PI / 180, 0);
	//ros::Duration(0.3).sleep(); // sleep
	//k = i;

	if (i == height_cloud) {

		if (k == 75) {
			k = -75;
			//ros::Duration(1).sleep(); // sleep
			My_Filter::mover_ptu(k * M_PI / 180, 0);
			inicia_movimento = 0;
		}
		mean_intensity = mean_intensity/scan_matrix.size();	//calcula a media da intensidade
		//calcula o desvio padrão
		deviation = standard_deviation();
		My_Filter::preencher_pointcloud();
		scan_matrix.clear();
		ros::Duration(5.0).sleep(); // sleep


		mover_ptu((height_cloud) * M_PI / 180, 0.0);
		inicia_movimento = 1;

		std::cout << "rodouessa parada		deviation "<<deviation<<"  min_intensity "<<min_intensity << std::endl;

		mean_intensity = 0;
		deviation = 0;
		i = 0;

	} else {
		if (inicia_movimento == 1) {
			for (int j = 0; j < (r_laser.ranges.size() - 1); j++) {

				intensidade = r_laser.intensities[j];
				mean_intensity = mean_intensity + intensidade;
				// calcula as intensidades maximas e minimas
				if(intensidade > max_intensity){
					max_intensity = intensidade;
				}
				if(intensidade < min_intensity){
					min_intensity = intensidade;
				}


				range = r_laser.ranges[j];

				phi = k * (M_PI / 180) * stack_resolution;//angulo entre as leituras do lazer
				theta = (j - 270) * (M_PI / 180) * resolution;//angulo entre os pontos de uma mesma leitura
				r = r_laser.ranges[j] * std::cos(theta);

				y = r_laser.ranges[j] * std::sin(theta) + 0.045 * std::cos(phi);
				z = r * std::sin(phi) + 0.045 * std::sin(phi);
				x = r * std::cos(phi);

				linha.push_back(x);
				linha.push_back(y);
				linha.push_back(z);
				linha.push_back(intensidade);
				linha.push_back(range);
				//std::cout<<"x: "<<x<<" y: "<<y<<" z: "<<linha[2]<< std::endl;
				scan_matrix.push_back(linha);
				linha.clear();
			}

			std::cout << "scan " << k << " i: " << i << std::endl;
			i++;
			k++;

			//auto end = std::chrono::high_resolution_clock::now();
			//std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count()*1000 << "us" << std::endl;

		}
	}

}

int main(int argc, char** argv) {
	//std:: cout<<"deu até 1" << std::endl;

	scan_matrix.clear();
	//std:: cout<<"deu até 2" << std::endl;
	ros::init(argc, argv, "my_filter");

	inicia_movimento = 0;

	My_Filter filter;
	//std:: cout<<"deu até 3" << std::endl;

	// move o ptu para o inicio do movimento
	filter.mover_ptu((-75) * M_PI / 180, 0.0);
	ros::Duration(5.0).sleep(); // sleep

	//inicia o movimento do ptu
	filter.mover_ptu((height_cloud) * M_PI / 180, 0.0);
	inicia_movimento = 1;

	ros::spin();

	return 0;

}

