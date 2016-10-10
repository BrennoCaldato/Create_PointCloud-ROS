#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <vector>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/video.hpp"
#include <iostream>
#include <fstream>

using namespace sensor_msgs;

// definitions
#define scan_topic "/scan"			//"/RosAria/sim_lms2xx_1_laserscan"			//name of laser topic
#define cloud_topic "/cloud"		//name of pointcloud2 topic
#define ptu_topic "/cmd"			//name of ptu topic
#define height_cloud 140			//heith of pointcloud colocar só numeros pares
#define deslocamento_max 140		//deslocamento maximo do pan-tilt
#define width_cloud 540
#define resolution 0.5				//laser resolution in degrees/points
#define stack_resolution 0.5			//resolution betwen two laserscans
#define color_rgb 1					//1 if use a rgb camera or 0 if don't

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
	void obter_imagem();
	void depth_image();

private:
	ros::NodeHandle node_;
	laser_geometry::LaserProjection projector_;
	image_transport::ImageTransport it ;
	ros::Publisher point_cloud_publisher_;
	ros::Publisher ptu_publisher;
	ros::Subscriber scan_sub_;
	image_transport::Publisher pub;

	int i;
	float k;
	double inverte_rotacao;
	double max_intensity, min_intensity, mean_intensity, deviation;
	double max_range, min_range;
	double rgb[19600][3];
	float ranges_vec[height_cloud*width_cloud];
	cv::Mat dst;	//dst image

};

My_Filter::My_Filter() {
	scan_sub_ = node_.subscribe<sensor_msgs::LaserScan>(scan_topic, 3, &My_Filter::scanCallback, this);
	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2>(	cloud_topic, 100, false);
	ptu_publisher = node_.advertise<sensor_msgs::JointState>(ptu_topic, 100, false);
	pub = it.advertise("/depth_image", 1);

	i = 0;
	k = -deslocamento_max / 2;
	max_intensity = 0;
	max_range = 0;
	min_intensity = 1000;
	min_range = 30;
	mean_intensity = 0;
	deviation = 0;
	inverte_rotacao = 0;
	cv::namedWindow("depth", CV_WINDOW_FREERATIO);

}

// executa a movimentação do ptu
void My_Filter::mover_ptu(double pan, double tilt) {
	sensor_msgs::JointState ptu;
	ptu.header.stamp = ros::Time::now();
	ptu.name.resize(2);
	ptu.position.resize(2);
	ptu.velocity.resize(2);
	ptu.name[0] = "ptu_pan";
	ptu.position[0] = pan;
	ptu.velocity[0] = 0.875 * stack_resolution;
	ptu.name[1] = "ptu_tilt";
	ptu.position[1] = tilt;
	ptu.velocity[1] = 0.8;

	ptu_publisher.publish(ptu);

}

// preenche o point cloud com os dados tratados das leituras do laser
void My_Filter::preencher_pointcloud() {
	int r = 0, g = 0, b = 0;
	double intensity_norm;
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

	//set interators for pointcloud
	sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");

	int j = 0, k = 0;
	for (size_t i = 0; i < scan_matrix.size() - 1; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
		//normalize intensity values to 0-255
		intensity_norm = (scan_matrix[i][3] - mean_intensity - 4 * deviation)/ (8 * deviation) * 255;
		//std::cout <<"j "<<j<<" k "<<k<< "  nao entrou ainda  "<<std::endl;

		if(color_rgb == 1){
			if (j >= 200 && j <= 340){
				//std::cout <<"j "<<j<<" k "<<k<<std::endl;
				*iter_x = scan_matrix[i][0];
				*iter_y = scan_matrix[i][1];
				*iter_z = scan_matrix[i][2];
				*iter_r = dst.at<cv::Vec3b>(j - 200, k)[2];
				*iter_g = dst.at<cv::Vec3b>(j - 200, k)[1];
				*iter_b = dst.at<cv::Vec3b>(j - 200, k)[0];
			}
			else{

				*iter_x = scan_matrix[i][0];
				*iter_y = scan_matrix[i][1];
				*iter_z = scan_matrix[i][2];
				*iter_r = intensity_norm;
				*iter_g = intensity_norm;
				*iter_b = intensity_norm;
			}

			j++;

			if (j>= 540 ){
				j = 0;
				k++;
			}

		}
		else{

		*iter_x = scan_matrix[i][0];
		*iter_y = scan_matrix[i][1];
		*iter_z = scan_matrix[i][2];
		*iter_r = intensity_norm;
		*iter_g = intensity_norm;
		*iter_b = intensity_norm;
		}

	}
	point_cloud_publisher_.publish(points_msg);
}

// calcula o desvio parão dos valores de intensidade e normaliza os valores do range entre 0-255
double My_Filter::standard_deviation() {
	std::ofstream myfile;
	myfile.open ("example.txt");
	double coiso = 0;
	for (int l = 0; l < scan_matrix.size(); l++) {
		// calculo do desvio padão
		coiso = coiso + pow(scan_matrix[l][3] - mean_intensity, 2);

		//normaliza os valores do range
		ranges_vec[l] = ( (ranges_vec[l] - min_range) / (max_range - min_range)) * 255;
		myfile << (int)ranges_vec[l] <<"\n";
	}
	coiso = sqrt((coiso / (scan_matrix.size() - 2)));
	myfile.close();
	return coiso;

}


// obtém uma imagem da webcam
void My_Filter::obter_imagem() {
	if (color_rgb == 1) {
		cv::VideoCapture cap;
		cv::Size size(140, 140);
		double r, g, b;

		cv::namedWindow("output", CV_WINDOW_FREERATIO);
		// open the default camera, use something different from 0 otherwise;
		// Check VideoCapture documentation.
		if (!cap.open(1))
			std::cout << " não foi possivel abrir a camera";
		//for(;;)
		//{
		cv::Mat frame;
		cap >> frame;
		cap >> frame;
		cv::flip(frame,frame,1);
		//if( frame.empty() ) break; // end of video stream
		imshow("output", frame);
		//waitKey(0);

		//if( waitKey(1) == 27 ) break; // stop capturing by pressing ESC

		resize(frame, dst, size);	         //resize image
		imshow("output", dst);

//	for (int i = 0; i < dst.rows; i++) {
//		for (int j = 0; j < dst.cols; j++) {
//			rgb[i*dst.cols+j][0] = dst.at<cv::Vec3b>(i, j)[0];
//			g = dst.at<cv::Vec3b>(i, j)[1];
//			b = dst.at<cv::Vec3b>(i, j)[2];
//
//			std::cout << r << " " << g << " " << b << std::endl;
//
//		}
//	}
	}
}

// publica uma depth image com os valores do range
void My_Filter::depth_image(){

	cv::Mat src =  cv::Mat(height_cloud , width_cloud , CV_32FC1,(float*)ranges_vec);
	src.convertTo( src, CV_8UC1 );
	std::cout <<src;
	cv::imshow("depth", src);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), CV_8UC1, src).toImageMsg();

	pub.publish(msg);

}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	std::cout << "movimento = " << inicia_movimento << std::endl;
	//auto begin = chrono::chrono::high_resolution_clock::now();

	double theta, phi, r, x, y, z, intensidade, range;
	std::vector<double> linha;
	sensor_msgs::LaserScan r_laser = *scan;

	//My_Filter::mover_ptu(k * M_PI / 180, 0);
	//ros::Duration(0.3).sleep(); // sleep
	//k = i;

	if (i == height_cloud) {

		if (k == deslocamento_max / 2) {
			inicia_movimento = 0;
			//ros::Duration(1).sleep(); // sleep
			mean_intensity = mean_intensity / scan_matrix.size();//calcula a media da intensidade
					//calcula o desvio padrão
			deviation = standard_deviation();
			depth_image();
			obter_imagem();
			My_Filter::preencher_pointcloud();
			ros::Duration(1.0).sleep(); // sleep

			My_Filter::mover_ptu(-k * (M_PI / 180) * stack_resolution, 0);
			inicia_movimento = 1;
			inverte_rotacao = 1;

		}

		if (k == -deslocamento_max / 2) {
			inicia_movimento = 0;
			//ros::Duration(1).sleep(); // sleep
			mean_intensity = mean_intensity / scan_matrix.size(); //calcula a media da intensidade
					//calcula o desvio padrão
			deviation = standard_deviation();
			depth_image();
			obter_imagem();
			My_Filter::preencher_pointcloud();
			ros::Duration(1.0).sleep(); // sleep

			My_Filter::mover_ptu(-k * (M_PI / 180) * stack_resolution, 0);
			inicia_movimento = 1;
			inverte_rotacao = 0;
		}

		std::cout << "rodouessa parada		deviation " << deviation
				<< "  min_intensity " << min_intensity << std::endl;

		scan_matrix.clear();

		mean_intensity = 0;
		deviation = 0;
		i = 0;

	} else {
		if (inicia_movimento == 1) {
			for (int j = 0; j < (r_laser.ranges.size() - 1); j++) {

				intensidade = r_laser.intensities[j];
				mean_intensity = mean_intensity + intensidade;
				// calcula as intensidades maximas e minimas
				if (intensidade > max_intensity) {
					max_intensity = intensidade;
				}
				if (intensidade < min_intensity) {
					min_intensity = intensidade;
				}

				range = r_laser.ranges[j];
				// calcula os ranges maximos e minimos
				if (range > max_range) {
					max_range = range;
				}
				if (range < min_range) {
					min_range = range;
				}

				phi = k * (M_PI / 180) * stack_resolution; //angulo entre as leituras do lazer
				theta = (j - 270) * (M_PI / 180) * resolution; //angulo entre os pontos de uma mesma leitura
				r = r_laser.ranges[j] * std::cos(theta);

				y = r_laser.ranges[j] * std::sin(theta) + 0.045 * std::cos(phi);
				z = r * std::sin(phi) + 0.045 * std::sin(phi);
				x = r * std::cos(phi);

				linha.push_back(x);
				linha.push_back(y);
				linha.push_back(z);
				linha.push_back(intensidade);
				ranges_vec[i*width_cloud+j] = r_laser.ranges[j];
				//std::cout<<"x: "<<x<<" y: "<<y<<" z: "<<linha[2]<< std::endl;
				scan_matrix.push_back(linha);
				linha.clear();
			}

			std::cout << "scan " << k << " i: " << i << std::endl;
			i++;
			if (inverte_rotacao == 1)
				k--;
			else
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
	ros::Duration(1.0).sleep(); // sleep

	//inicia o movimento do ptu
	filter.mover_ptu((height_cloud) * (M_PI / 180) * stack_resolution, 0.0);
	inicia_movimento = 1;

	ros::spin();

	return 0;

}
