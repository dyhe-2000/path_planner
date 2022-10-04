#include <iostream>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
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

class path_planner : public rclcpp::Node{
protected:
    MsgSubscriber<sensor_msgs::msg::Image>::UniquePtr occupency_grid_map_sub;
    rclcpp::TimerBase::SharedPtr step_timer_;  
    Timer check_duration_timer; 
    double dt_;
public:
	explicit path_planner(const rclcpp::NodeOptions & options): Node("path_planner_node", options) {
        subscribe_from(this, occupency_grid_map_sub, "/slam_map");
        declare_parameter("dt", 0.065); // step function every 0.065 sec
	    get_parameter("dt", this->dt_);
        this->step_timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::duration<float>(this->dt_), [this] {step();});
	}

    void step(){
		this->check_duration_timer.reset();
		mtx.lock();
        
        if(this->occupency_grid_map_sub->has_msg()){ // has a new message buffered
            std::cout << "received the slam map" << std::endl;
            sensor_msgs::msg::Image occupency_map = *(this->occupency_grid_map_sub->take());

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
            for(int i = 0; i < frame.rows; i++){
                for(int j = 0; j < frame.cols; j++){
                    int num_in_grid = original_pixel_ptr[i*frame.cols*cn + j*cn + 0]; 
                    std::cout << num_in_grid << " ";
                }
                std::cout << std::endl;
            }
        }

		mtx.unlock();
		std::cout << "step duration: " << this->check_duration_timer.elapsed() << std::endl;
		return;
	}
};

int main(int argc, char** argv){
    std::cout << "this is path planner node" << std::endl;
    rclcpp::init(argc, argv);
	rclcpp::NodeOptions options{};
	auto node = std::make_shared<path_planner>(options);
	rclcpp::spin(node);
	rclcpp::shutdown();
}