/**
 *    Desc:
 *
 *    (C) 2017, Author: EMRE OZBILGE
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, see <http://www.gnu.org/licenses/>
 */


#include <libplayerc++/playerc++.h>
#include <iostream>
#include <cstdio>
#include <cmath>
#include <iomanip>
#include <vector> //std::vector
#include <algorithm> // min_element, max_element
#include <numeric> // accumulate
#include <ctime> // time()
#include<unistd.h> // usleep()

using namespace PlayerCc;

/*
 * This device looks for fiducials (markers or beacons) in the laser scan, and 
 * determines their identity, range, bearing and orientation. 
 */
enum class State {
    OBSTACLE_AVOIDANCE,
    WANDER_CONTROLLER,
    FOLLOW_YELLOW_ROBOT
};

int main(int argc, char **argv)
{
    // we throw exceptions on creation if we fail
    try
    {
        // create robot client to communicate with robot
        PlayerClient robot("localhost");
        
        //change client data mode to receive always latest data
        robot.SetDataMode(PLAYER_DATAMODE_PULL);
        robot.SetReplaceRule(true);   
        
        // creating instance for robot position data
        Position2dProxy pp(&robot, 0);
        
        // creating proxy for laser device        
        RangerProxy lp(&robot,1);
        
        // define a BlobfinderProxy proxy to communicate with robot's camera
        BlobfinderProxy bfp(&robot, 0);            
        
        // make sure we get data from laser
        std::puts("Waiting for Laser sensors to be ready...\n");
        while (lp.GetRangeCount() == 0) {
            robot.Read();
        }    
        std::puts("Laser is ready!\n");
        
        // enabling robot motors
        pp.SetMotorEnable(true);
        
        // only print two decimals after the point
        std::cout << std::setprecision(2) << std::fixed;  
        
        // variables for changing speed of the robot
        double lv = 0, av = 0;
        
        // variables for laser readings
        const int SIZE_LP = lp.GetRangeCount();
        std::vector<double> temp_readings(SIZE_LP);
        double min_left, min_right, min_front;
        
        // variables for blobfinder
        // 0: right, 1: front, 2: left
        int direct_blob;
        
        int centre;
        /*number of pixels away from the image centre a blob
         *     can be, to be in front of the robot. This is
         *     essentially the margin of error.*/
        int margin = 10;
        
        playerc_blobfinder_blob_t blob;
        //find the largest blob
        int biggestBlobArea = 0;
        int biggestBlob = 0;  
        
        uint32_t color; 
        float range;
        uint32_t yellow_code = 16776960; // rgb code of yellow robot
        bool is_yellow_rob = false; // flag if yellow robot is detected
        
        // Finite State Machine
        State current_state = State::WANDER_CONTROLLER;
        double threshold = 0.7;      // Distance threshold for obstacle detection
      
        int max_turn = 90;
        int min_turn = -90;
      
           // variables for blob detection
    bool is_blob_detected = false; // flag for detecting other blobs
    double blob_threshold = 1.0; // distance threshold for detecting blobs

        
        // go into read-think-act loop
        for(;;)
        {
            // this blocks until new data comes; 10Hz by default
            robot.Read();    
            
            ////////////////////////////////////////////////////////////////////////////////////
            
            std::cout << "Number of detected blobs: " << bfp.GetCount() << std::endl;            
            if(bfp.GetCount() != 0) {
                
                for(int i = 0; i < bfp.GetCount(); i++)
                {
                    //get blob from proxy
                    playerc_blobfinder_blob_t currBlob = bfp[i];
                    
                    if(abs((int)currBlob.area) > biggestBlobArea)
                    {
                        biggestBlob = i;
                        biggestBlobArea = currBlob.area;
                    }
                }
                blob = bfp[biggestBlob]; 
                
                // find centre of image
                centre = bfp.GetWidth() / 2;
                //blob to the left of centre
                if(blob.x < centre - margin)
                    direct_blob = 2;
                //blob to the right of centre
                else if(blob.x > centre + margin)
                    direct_blob = 0;
                else
                    direct_blob = 1;
                
                range = blob.range;
                color = blob.color;
                
                // How far is the detected blob and its colour and directions
                std::cout << "Range: " << range << " Color: " << color << " Direction: " << direct_blob << std::endl;
                
                is_yellow_rob = false;
                if(color == yellow_code) {
                    std::cout << "Yellow robot found!" << std::endl;
                    is_yellow_rob = true;
                }  
            }
            else {
                is_yellow_rob = false;
            }
            
            ////////////////////////////////////////////////////////////////////////////////////
            
            // copy the laser readings to the C++ vector to use powerful processing
            for(int i = 0; i < SIZE_LP; ++i)
                temp_readings[i] = lp[i];
            
            // find the minimum readings for each 60 degree range
            min_right = *std::min_element(temp_readings.begin(), temp_readings.begin() + 60);
            min_front = *std::min_element(temp_readings.begin() + 60, temp_readings.begin() + 120);
            min_left = *std::min_element(temp_readings.begin() + 120, temp_readings.begin() + 180);
            
            // display the outputs from the laser processing
            std::cout << "Left object: " << min_left << " Front object: " << min_front << " Right object: " << min_right << std::endl;       
            
            ////////////////////////////////////////////////////////////////////////////////////
            /create finite state machine to find and follow the yellow robot/
            
            // Finite State Machine Logic
            switch (current_state) {
                case State::OBSTACLE_AVOIDANCE:
                    if (min_right < threshold && min_left > threshold) {
                       
                        lv = 0.1;
                        av = 0.5;
                    }
                    else if(min_right > threshold && min_left < threshold) {
                       
                        lv = 0.1;
                        av = -0.5;
                    }
                    else if(min_right < threshold && min_left < threshold) {
                       
                        lv = 0;
                        av = 0.15;
                    }
                    else {
                       
                        current_state = State::WANDER_CONTROLLER;
                        lv = 0.3;
                        av = 0;
                    }
                    break;
                    
            
            case State::WANDER_CONTROLLER:
                if (is_yellow_rob) {
                    std::cout << "Yellow robot detected" << std::endl;
                    current_state = State::FOLLOW_YELLOW_ROBOT;
                    lv = 0.0;
                    av = 0.0;
                } else {
                    // Check for other blobs
                    if (bfp.GetCount() > 0) {
                        for (int i = 0; i < bfp.GetCount(); ++i) {
                            playerc_blobfinder_blob_t currBlob = bfp[i];
                            if (currBlob.range < blob_threshold) {
                                is_blob_detected = true;
                                break;
                            }
                        }
                    } else {
                        is_blob_detected = false;
                    }

                    if (is_blob_detected) {
                        lv = 0.1;
                        av = -0.5;
                    } else if (min_front <= threshold) {
                        current_state = State::OBSTACLE_AVOIDANCE;
                        lv = 0.2;
                        av = 0.0;
                    } else {
                       // lv = ((rand() % 11) / 10.0) * max_speed; // [0,max_speed]
                        lv = 0.5;  // Increase the wandering speed
                        av = dtor((rand() % (max_turn - min_turn + 1)) + min_turn);
                    }
                }
                break;
                    
    case State::FOLLOW_YELLOW_ROBOT:
       
        
    	 std::cout<<"follow yellow robot\n";
        if (is_yellow_rob) {
            
          
            
         if (range < 0.5) {
             std::cout << "Yellow robot is too close! Stop" << std::endl;
                lv = 0.0;  //when too close we stop because it was crashing with robot.
                av = 0.0;
            } else {
                lv = 0.5;
            }
            
            
            
            if (direct_blob == 0) {
                av = -0.5;
            } else if (direct_blob == 2) {
                av = 0.5;
            } else {
                av = 0.0;
            }
        } else {
            current_state = State::WANDER_CONTROLLER;
        }
        
        
       break;



            }
            
            ////////////////////////////////////////////////////////////////////////////////////
            
            // set the robot velocities 
            pp.SetSpeed(lv, av);     
            
        }
        
        // stop the robot
        pp.SetSpeed(0.0, 0.0);

        // disable motors
        pp.SetMotorEnable(false);
    }
    catch(PlayerCc::PlayerError& e) {
        std::cerr << e << std::endl;
        return -1;
    }
    
    return 0;
}
