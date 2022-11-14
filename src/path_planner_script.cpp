#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "boat_interfaces/msg/vector_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <utility>
#include <stdlib.h> 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "path_planner/buffer_subscriber.hpp"
#include "path_planner/Timer.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <Eigen/LU>
#include <Eigen/QR>
#include <mutex>
#include <tuple>
#include "geometry_msgs/msg/vector3.hpp"
#include <fstream>
#include <random>
#include <time.h>
using std::placeholders::_1;
std::mutex mtx;

// <x, y>
void bresenham2d(std::pair<std::pair<double, double>, std::pair<double, double>> theData, std::vector<std::pair<int, int>> *theVector) {
	int y1, x1, y2, x2;
	int dx, dy, sx, sy;
	int e2;
	int error;

	x1 = int(theData.first.first);
	y1 = int(theData.first.second);
	x2 = int(theData.second.first);
	y2 = int(theData.second.second);

	dx = abs(x1 - x2);
	dy = -abs(y1 - y2);

	sx = x1 < x2 ? 1 : -1;
	sy = y1 < y2 ? 1 : -1;

	error = dx + dy;

	while (1) {
		theVector->push_back(std::pair<int, int>(x1, y1));
		if (x1 == x2 && y1 == y2)
			break;
		e2 = 2 * error;

		if (e2 >= dy) {
			if (x2 == x1) break;
			error = error + dy;
			x1 = x1 + sx;
		}
		if (e2 <= dx) {
			if (y2 == y1) break;
			error = error + dx;
			y1 = y1 + sy;
		}
	}
}

Eigen::MatrixXd calc_worldTbody(Eigen::MatrixXd& x){
	// x is 3 by 1
	Eigen::MatrixXd worldTbody(4, 4); //4 by n matrix
	worldTbody = Eigen::MatrixXd::Zero(4,4);
	worldTbody(0,0) = 1;
	worldTbody(1,1) = worldTbody(2,2) = worldTbody(3,3) = worldTbody(0,0);
	worldTbody(0,0) = cos(x(2,0));
	worldTbody(0,1) = -sin(x(2,0));
	worldTbody(1,0) = sin(x(2,0));
	worldTbody(1,1) = cos(x(2,0));
	worldTbody(0,3) = x(0,0);
	worldTbody(1,3) = x(1,0);
	return worldTbody;
}

Eigen::MatrixXd calc_T_inv(Eigen::MatrixXd& T){
    Eigen::MatrixXd T_inv(4, 4); //4 by 4 matrix
	T_inv = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd R = T.block(0, 0, 3, 3);
    Eigen::MatrixXd p = T.block(0, 3, 3, 1);
    Eigen::MatrixXd R_T = R.transpose();
    T_inv(3,3) = 1;
    T_inv.block(0,0,3,3) = R_T;
    T_inv.block(0,3,3,1) = -R_T*p;
    return T_inv;
}

class path_planner : public rclcpp::Node{
protected:
    // publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr path_planner_map_pub_;
	rclcpp::Publisher<boat_interfaces::msg::VectorArray>::SharedPtr map_path_pub_;

    MsgSubscriber<sensor_msgs::msg::Image>::UniquePtr occupency_grid_map_sub;
    rclcpp::TimerBase::SharedPtr step_timer_;  
    Timer check_duration_timer; 
    double dt_;
public:
	explicit path_planner(const rclcpp::NodeOptions & options): Node("path_planner_node", options) {
        this->path_planner_map_pub_ = this->create_publisher<sensor_msgs::msg::Image>("Jesus_output", 10);
		this->map_path_pub_ = this->create_publisher<boat_interfaces::msg::VectorArray>("map_path", 10);

        subscribe_from(this, occupency_grid_map_sub, "/slam_occupancy_grid_map");
        declare_parameter("dt", 0.1); // step function every 0.065 sec
	    get_parameter("dt", this->dt_);
        this->step_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(this->dt_), [this] {step();});
		publish_map_path();
	}

	void publish_map_path(){
		int num_angle_pts = 0;
		std::cout << "enter how many angle points: ";
		std::cin >> num_angle_pts;

		boat_interfaces::msg::VectorArray path;
		for(int i = 0; i < num_angle_pts; ++i){
			std::cout << "pt " << i << " enter x: " << std::endl;
			int x;
			std::cin >> x;
			std::cout << "pt " << i << " enter y: " << std::endl;
			int y;
			std::cin >> y;
    		geometry_msgs::msg::Vector3 theVec;
    		theVec.x = x;
    		theVec.y = y;
    		theVec.z = 0;
    		path.vec3list.push_back(theVec);
		}
		map_path_pub_->publish(path);
		this->publish_map_path();
	}

    void publish_map(cv::Mat& map) { // running repeatedly with the timer set frequency
		std::cout << "start publishing" << std::endl;

		//this->Trajectory_Map; // 8UC3
		sensor_msgs::msg::Image::UniquePtr map_msg(new sensor_msgs::msg::Image());
		auto stamp = now();
		map_msg->header.stamp = stamp;
		map_msg->height = map.rows;
		map_msg->width = map.cols;
		map_msg->encoding = "8UC3";
		map_msg->is_bigendian = false;
		map_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(map.step);
		map_msg->data.assign(map.datastart,map.dataend);
		path_planner_map_pub_->publish(std::move(map_msg));

		std::cout << "finish publishing" << std::endl;
    }

    void step(){
		this->check_duration_timer.reset();
		mtx.lock();
        
        if(this->occupency_grid_map_sub->has_msg()){ // has a new message buffered
            std::cout << "received the slam map" << std::endl;
            sensor_msgs::msg::Image occupency_map = *(this->occupency_grid_map_sub->take());

            // Jesus you do all sort's of computation here now is just printing each map's grid's value

		    int img_height = occupency_map.height;
		    int img_width = occupency_map.width;

            std::cout << "occupency_map_height: " << img_height << std::endl;
            std::cout << "occupency_map_width: " << img_width << std::endl;
            std::cout << "occupency_map encoding: " << occupency_map.encoding << std::endl; 

            cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(occupency_map, occupency_map.encoding);
            cv::Mat frame = img_ptr->image;
            uint8_t* original_pixel_ptr = (uint8_t*)frame.data;
            int cn = frame.channels();
            std::cout << "channel count: " << cn << std::endl;
			std::cout << "pushing image step: " << frame.step << std::endl;
            for(int i = 0; i < frame.rows; i++){
                for(int j = 0; j < frame.cols; j++){
                    int r = original_pixel_ptr[i*frame.cols*cn + j*cn + 0]; 
					int g = original_pixel_ptr[i*frame.cols*cn + j*cn + 1]; 
					int b = original_pixel_ptr[i*frame.cols*cn + j*cn + 2]; 
                    //std::cout << "r: " << r << " g: " << g << " b: " << b;
                }
                //std::cout << std::endl;
            }

			// J
			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[100*frame.cols*cn + (100+i)*cn + 0] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (100+i)*cn + 1] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (100+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[(100+i)*frame.cols*cn + 150*cn + 0] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + 150*cn + 1] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + 150*cn + 2] = 0; 
			}

			for(int i = 0; i < 50; ++i){
				original_pixel_ptr[200*frame.cols*cn + (100+i)*cn + 0] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (100+i)*cn + 1] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (100+i)*cn + 2] = 0; 
			}

			// E x: 250 - 350 y: 100 - 200
			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[100*frame.cols*cn + (250+i)*cn + 0] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (250+i)*cn + 1] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (250+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[150*frame.cols*cn + (250+i)*cn + 0] = 255; 
				original_pixel_ptr[150*frame.cols*cn + (250+i)*cn + 1] = 255; 
				original_pixel_ptr[150*frame.cols*cn + (250+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[200*frame.cols*cn + (250+i)*cn + 0] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (250+i)*cn + 1] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (250+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[(100+i)*frame.cols*cn + (250)*cn + 0] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (250)*cn + 1] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (250)*cn + 2] = 0; 
			}

			// S x: 400 - 500 y: 100 - 200
			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[100*frame.cols*cn + (400+i)*cn + 0] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (400+i)*cn + 1] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (400+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[150*frame.cols*cn + (400+i)*cn + 0] = 255; 
				original_pixel_ptr[150*frame.cols*cn + (400+i)*cn + 1] = 255; 
				original_pixel_ptr[150*frame.cols*cn + (400+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[200*frame.cols*cn + (400+i)*cn + 0] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (400+i)*cn + 1] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (400+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 50; ++i){
				original_pixel_ptr[(100+i)*frame.cols*cn + (400)*cn + 0] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (400)*cn + 1] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (400)*cn + 2] = 0; 
			}

			for(int i = 0; i < 50; ++i){
				original_pixel_ptr[(150+i)*frame.cols*cn + (500)*cn + 0] = 255; 
				original_pixel_ptr[(150+i)*frame.cols*cn + (500)*cn + 1] = 255; 
				original_pixel_ptr[(150+i)*frame.cols*cn + (500)*cn + 2] = 0; 
			}

			// U x: 550 - 650 y: 100 - 200
			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[200*frame.cols*cn + (550+i)*cn + 0] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (550+i)*cn + 1] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (550+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[(100+i)*frame.cols*cn + (550)*cn + 0] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (550)*cn + 1] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (550)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[(100+i)*frame.cols*cn + (650)*cn + 0] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (650)*cn + 1] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (650)*cn + 2] = 0; 
			}

			// S x: 700 - 800 y: 100 - 200
			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[100*frame.cols*cn + (700+i)*cn + 0] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (700+i)*cn + 1] = 255; 
				original_pixel_ptr[100*frame.cols*cn + (700+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[150*frame.cols*cn + (700+i)*cn + 0] = 255; 
				original_pixel_ptr[150*frame.cols*cn + (700+i)*cn + 1] = 255; 
				original_pixel_ptr[150*frame.cols*cn + (700+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 100; ++i){
				original_pixel_ptr[200*frame.cols*cn + (700+i)*cn + 0] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (700+i)*cn + 1] = 255; 
				original_pixel_ptr[200*frame.cols*cn + (700+i)*cn + 2] = 0; 
			}

			for(int i = 0; i < 50; ++i){
				original_pixel_ptr[(100+i)*frame.cols*cn + (700)*cn + 0] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (700)*cn + 1] = 255; 
				original_pixel_ptr[(100+i)*frame.cols*cn + (700)*cn + 2] = 0; 
			}

			for(int i = 0; i < 50; ++i){
				original_pixel_ptr[(150+i)*frame.cols*cn + (800)*cn + 0] = 255; 
				original_pixel_ptr[(150+i)*frame.cols*cn + (800)*cn + 1] = 255; 
				original_pixel_ptr[(150+i)*frame.cols*cn + (800)*cn + 2] = 0; 
			}

            // Jesus you stop here and publish back
            this->publish_map(frame);
        }

		mtx.unlock();
		std::cout << "step duration: " << this->check_duration_timer.elapsed() << std::endl;
		return;
	}
};

int main(int argc, char** argv){
    std::cout << "this is path planner node" << std::endl;

	/*
	boat_interfaces::msg::VectorArray path;
    geometry_msgs::msg::Vector3 theVec;
    theVec.x = 0;
    theVec.y = 1;
    theVec.z = 2;
    path.vec3list.push_back(theVec);
    std::cout << path.vec3list.size() << std::endl;
    for(int i = 0; i < path.vec3list.size(); ++i){
        std::cout << path.vec3list[i].x << " " << path.vec3list[i].y << " " << path.vec3list[i].z << std::endl;
    }
	*/

    rclcpp::init(argc, argv);
	rclcpp::NodeOptions options{};
	auto node = std::make_shared<path_planner>(options);
	rclcpp::spin(node);
	rclcpp::shutdown();
}