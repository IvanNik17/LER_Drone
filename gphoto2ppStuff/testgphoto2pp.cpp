#include <gphoto2pp/camera_wrapper.hpp> 		// Header for CameraWrapper
#include <gphoto2pp/camera_file_wrapper.hpp>	// Header for CameraFileWrapper
#include <gphoto2pp/helper_camera_wrapper.hpp>	// Used for helper::capture(...) method


#include <gphoto2pp/camera_capture_type_wrapper.hpp>
#include <gphoto2pp/exceptions.hpp>

#include <gphoto2pp/window_widget.hpp>
#include <gphoto2pp/toggle_widget.hpp>


#include <iostream>

#include <stdio.h>
#include <stdlib.h>

#include <chrono>
#include <thread>

#include <future>




class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const { 
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};

template<typename T>
bool future_is_ready(std::future<T>& t){
    return t.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

/*
void capture_image( ){
		
		while (true){
			if (takeImage == true){
				gphoto2pp::CameraWrapper camera;
				// Creates empty instance of a cameraFile, which will be populated in our helper method
				gphoto2pp::CameraFileWrapper cameraFile;

				// Takes a picture with the camera and does all of the behind the scenes fetching
				gphoto2pp::helper::capture(camera, cameraFile);
			}

		}


}
*/

void capture_image( ){
	gphoto2pp::CameraWrapper camera;
	gphoto2pp::CameraFileWrapper cameraFile;
	gphoto2pp::helper::capture(camera, cameraFile);
}

int main()
{

	system("./killgphoto.sh");
	// Connects to the first camera found and initializes
	
	//gphoto2pp::CameraWrapper camera;
	// Prints out the summary of your camera's abilities
	//std::cout << camera.getSummary() << std::endl;


	Timer tmr;

	bool isThready = true;
	int counter = 0;
	std::future<void> checker;
	
	while (true) {


		
		if (tmr.elapsed() >1 ){
			// Creates empty instance of a cameraFile, which will be populated in our helper method
			//gphoto2pp::CameraFileWrapper cameraFile;
	
			// Takes a picture with the camera and does all of the behind the scenes fetching
			//gphoto2pp::helper::capture(camera, cameraFile);

			//std::thread t1(capture_image);

			//t1.join();
			if (isThready == true) {

				checker = std::async(std::launch::async, capture_image);  
				isThready = false;
			}
			

			
			    if (future_is_ready(checker)){
				isThready = true;
				tmr.reset();
			    }

			
		} 

		counter +=1;
		std::cout <<counter<< std::endl;
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	



	
	// Lastly saves the picture to your hard drive
	// Your camera might take files in different formats (bmp, raw)
	// so this extension might be wrong and you should rename your file appropriately
	//cameraFile.save("my-gphoto2pp-test.jpg");
	
	std::cout <<"Done!"<< std::endl;
	return 1;
}
