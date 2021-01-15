#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

#include "/home/ubuntu/workspace/project/src/clipper/clipper.hpp"
#include "clipper/clipper.hpp"
#include "dubins.h"
namespace student {

  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;

  
  //---------------------
  const int max_nodes = 100;  //TODO: change
const int max_loops = 2000;
const int least_nodes = 20;

// RRT* switch
const bool RRT_STAR = true;

// Choose mission & params
//const bool mission_2 = false;
const bool mission_1=true;
const int bonus = 2;  // Bonus time for picking up victim (seconds)
const double speed = 0.2; // Should be ~ 0.2 m/s (0.5m/2.5s)  

  //-----------------------------------------
  void RRT(const float theta, Path& path, std::vector<Point>& major_points_path, const Polygon& borders, int kmax, int npts, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center, double& path_length, const std::vector<double> gate_orientn);

//std::vector<int> Dijkstra(std::vector<std::vector<double>> costmap, const std::vector<std::pair<int,Polygon>>& victim_list);
  int inside_polygon(Polygon obstacle, Point pt);

bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b);

double calculate_angle(Point a, Point b);
  //------------------------------------------

  void mouseCallback(int event, int x, int y, int, void* p)
  {
    if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;
    
    result.emplace_back(x*show_scale, y*show_scale);
    cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n) {
      usleep(500*1000);
      done.store(true);
    }
  }

  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)
  {
    result.clear();
    std::cout << "in student interface pickNPoints" << std::endl;
    cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
    cv::resize(img, bg_img, small_size);
    //bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load()) {
      cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());
    return result;
  }


 void loadImage(cv::Mat& img_out, const std::string& config_folder){

   //throw std::logic_error( "STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED" );
 }

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){
    static size_t id = 0;
    static bool init = false;
    static std::string folder_path;

    if(!init){      
      bool exist = true;
      int i = 0;
      while(exist && i < 1000){
        std::stringstream ss;
        ss << config_folder << "/camera_image" << std::setw(3) << std::setfill('0') << i << "/";
        folder_path = ss.str();

        exist = std::experimental::filesystem::exists(folder_path);

        i++;        
      }
      
      if(i > 999 || !std::experimental::filesystem::create_directories(folder_path)){
        throw std::logic_error( "NO EMTY FOLDER" );
      }

      init = true;
    }
          
    cv::imshow( topic, img_in);
    char c;
    c = cv::waitKey(30);
    
    std::stringstream img_file;
    switch (c) {      
    case 's':   
      img_file << folder_path << std::setfill('0') 
          << std::setw(3)  << (id++) << ".jpg";
      cv::imwrite( img_file.str(), img_in );

      std::cout << "Saved image " << img_file.str() << std::endl;
      break;
    default:
        break;
    }
    //throw std::logic_error( "STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED" );
  }

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){
    std::string file_path = config_folder + "/extrinsicCalib.csv";

    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path)){
          
      std::experimental::filesystem::create_directories(config_folder);
      
      image_points = pickNPoints(4, img_in);
      // SAVE POINT TO FILE
      // std::cout << "IMAGE POINTS: " << std::endl;
      // for (const auto pt: image_points) {
      //   std::cout << pt << std::endl;
      // }
      std::ofstream output(file_path);
      if (!output.is_open()){
        throw std::runtime_error("Cannot write file: " + file_path);
      }
      for (const auto pt: image_points) {
        output << pt.x << " " << pt.y << std::endl;
      }
      output.close();
    }else{
      // LOAD POINT FROM FILE
      std::ifstream input(file_path);
      if (!input.is_open()){
        throw std::runtime_error("Cannot read file: " + file_path);
      }
      while (!input.eof()){
        double x, y;
        if (!(input >> x >> y)) {
          if (input.eof()) break;
          else {
            throw std::runtime_error("Malformed file: " + file_path);
          }
        }
        image_points.emplace_back(x, y);
      }
      input.close();
    }
    
    cv::Mat dist_coeffs;
    dist_coeffs   = (cv::Mat1d(1,4) << 0, 0, 0, 0, 0);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    // cv::Mat Rt;
    // cv::Rodrigues(rvec_, Rt);
    // auto R = Rt.t();
    // auto pos = -R * tvec_;

    if (!ok) {
      std::cerr << "FAILED SOLVE_PNP" << std::endl;
    }

    return ok;
    //throw std::logic_error( "STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED" );   
  }

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    static bool maps_initialized = false;
    static cv::Mat full_map1, full_map2;
    //std::cout << "in student interface imageUndistort" << std::endl;
    if(!maps_initialized){
      // Note: m1type=CV_16SC2 to use fast fixed-point maps (see cv::remap)
      cv::Mat R;
      cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, 
                                img_in.size(), CV_16SC2, full_map1, full_map2);

      maps_initialized = true;
    }

    // Initialize output image    
    cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);    

    //throw std::logic_error( "STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED" );  

  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
    cv::Mat image_points;

    // project points
    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points);

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane);
    //throw std::logic_error( "STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED" );  
  }

bool processObstacles(const cv::Mat& hsv_img, const double scale, std::vector<Polygon>& obstacle_list){
    
    cv::Mat red_mask_low, red_mask_high, red_mask;
    cv::inRange(hsv_img, cv::Scalar(0, 72, 105), cv::Scalar(20, 255, 255), red_mask_low);
    //cv::inRange(hsv_img, cv::Scalar(175, 10, 10), cv::Scalar(179, 255, 255), red_mask_high);
    cv::inRange(hsv_img, cv::Scalar(130, 81, 49), cv::Scalar(180, 255, 150), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);
   
    // Find red regions
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    // Process red mask
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
 
    for (int i=0; i<contours.size(); ++i)
    {
      // Approximate polygon w/ fewer vertices if not precise
      // 3rd arg - max distance original curve to approx
      approxPolyDP(contours[i], approx_curve, 3, true);
 
      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      // Add obstacle to list
      obstacle_list.push_back(scaled_contour);
    }
 
    std::vector<Polygon> inflated_obstacles;
 
    const double INT_ROUND = 2000.;
 
    for (int obs = 0; obs < obstacle_list.size(); ++obs) {
 
        ClipperLib::Path srcPoly;
        ClipperLib::Paths newPoly;
        ClipperLib::ClipperOffset co;
 
    // Iterate through obstacles
        for (int ver = 0; ver < obstacle_list[obs].size(); ++ver){
            int x = obstacle_list[obs][ver].x * INT_ROUND;
            int y = obstacle_list[obs][ver].y * INT_ROUND;
            // Add list of points to path
            srcPoly << ClipperLib::IntPoint(x,y);
        }
 
    // Provides methods to offset given path
        co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);

        co.Execute(newPoly, 50);    
        for (const ClipperLib::Path &path: newPoly){
            // Obstacle obst =  data structure for current obstacle
            Polygon obst;
            for (const ClipperLib::IntPoint &pt: path){
                double x = pt.X / INT_ROUND;
                double y = pt.Y / INT_ROUND;
                // Add vertex (x,y) to current obstacle
                obst.emplace_back(x,y);
            }
            // Close and export current obstacle
            inflated_obstacles.push_back(obst);
            obstacle_list[obs] = obst;
        }
   
    }
 
    return true;
 
  }

  bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate){
    
    // Find purple regions
    cv::Mat green_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);    
    //cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);
    
    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    // Process purple mask
    contours_img = hsv_img.clone();
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    //cv::imshow("filterrrr purple", purple_mask);
    //cv::waitKey(0);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 4, cv::LINE_AA);
    // std::cout << "N. contours: " << contours.size() << std::endl;

    
    bool res = false;

    for( auto& contour : contours){
      std::cout << "processGate return fals e" << std::endl;
      const double area = cv::contourArea(contour);
      //std::cout << "AREA " << area << std::endl;
      //std::cout << "SIZE: " << contours.size() << std::endl;
      if (area > 500){
        std::cout << "processGate return fals ffe" << std::endl;
        approxPolyDP(contour, approx_curve, 30, true);

        if(approx_curve.size()!=4) continue;

         //contours_approx = {approx_curve};
         //drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);


        for (const auto& pt: approx_curve) {
          gate.emplace_back(pt.x/scale, pt.y/scale);
          std::cout << "processGate return falsrr e" << std::endl;
        }
        res = true;
        break;
      }          
    }
    //cv::imshow("Original", contours_img);
    //cv::waitKey(0);
    return res;
  }

  cv::Mat rotate_mat(cv::Mat input_ROI, double angle_deg){
    cv::Mat out_ROI;
    cv::Point2f center(input_ROI.cols/2., input_ROI.rows/2.);  
 
    cv::Mat rot_mat = cv::getRotationMatrix2D(center, angle_deg, 1.0);
 
    cv::Rect2f bounding_box = cv::RotatedRect(cv::Point2f(), input_ROI.size(), angle_deg).boundingRect2f();
   
    rot_mat.at<double>(0,2) += bounding_box.width/2.0 - input_ROI.cols/2.0;
    rot_mat.at<double>(1,2) += bounding_box.height/2.0 - input_ROI.rows/2.0;
   
    warpAffine(input_ROI, out_ROI, rot_mat, bounding_box.size());
    return out_ROI;
  }

///std::string template_folder = "/home/ubuntu/workspace/template/";

  std::string template_folder = "/home/ubuntu/workspace/project/src/template";
  
bool processVictims(const cv::Mat& hsv_img, const double scale, std::vector<std::pair<int,Polygon>>& victim_list){

  // Find green regions
    cv::Mat green_mask;
   
    // store a binary image in green_mask where the white pixel are those contained in HSV rage (x,x,x) --> (y,y,y)
    //cv::inRange(hsv_img, cv::Scalar(50, 80, 34), cv::Scalar(75, 255, 255), green_mask_victims); //Simulator
    //cv::inRange(hsv_img, cv::Scalar(13, 68, 41), cv::Scalar(86, 255, 80), green_mask_victims);
    //cv::inRange(hsv_img, cv::Scalar(15, 65, 40), cv::Scalar(85, 255, 95), green_mask_victims);
    // Dark w/ light
  cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), green_mask);

    // Apply some filtering
    // Create the kernel of the filter i.e. a rectangle with dimension 3x3
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    // Dilate using the generated kernel
    cv::dilate(green_mask, green_mask, kernel);
    // Erode using the generated kernel
    cv::erode(green_mask,  green_mask, kernel);

    //cv::imshow("filterrrr", green_mask);
    //cv::waitKey(0);
 
    // Find green contours
    std::vector<std::vector<cv::Point>> contours, contours_approx;    
    // Create an image which we can modify not changing the original image
    cv::Mat contours_img;
    contours_img = hsv_img.clone();
 
    // Finds green contours in a binary (new) image
    cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // create an array of rectangle (i.e. bounding box containing the green area contour)  
    std::vector<cv::Rect> bound_rect(contours.size());
    int victim_id = 0;

    for (int i=0; i<contours.size(); ++i){
      double area = cv::contourArea(contours[i]);
      if (area < 500) continue; // filter too small contours to remove false positives

      std::vector<cv::Point> approx_curve;
      approxPolyDP(contours[i], approx_curve, 10, true);
      if(approx_curve.size() < 6) continue; //fitler out the gate
     
      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      // Add victims to the victim_list
      victim_list.push_back({victim_id++, scaled_contour});
 
      contours_approx = {approx_curve};
      // Draw the contours on image with a line color of BGR=(0,170,220) and a width of 3
      drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
 
      // find the bounding box of the green blob approx curve
      bound_rect[i] = boundingRect(cv::Mat(approx_curve));

    }
    std::cout << " End process, begin digit recognition"  << std::endl;
    


    //-------------------------Digit Recognition----------------------------------------
    cv::Mat green_mask_inv;
    cv::Mat filtered(hsv_img.rows, hsv_img.cols, CV_8UC3, cv::Scalar(255,255,255));
 
    // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
    cv::bitwise_not(green_mask, green_mask_inv);
    //std::cout << "   3 "  << std::endl;
    // Load digits template images
    std::vector<cv::Mat> templROIs;
    for (int i=1; i<=5; ++i) {
      auto num_template = cv::imread(template_folder + std::to_string(i) + ".png");
      // flip the template to get the shape of number in the unwarped ground image
      cv::flip(num_template, num_template, 1);
      // Store the template in templROIs (vector of mat)
      templROIs.emplace_back(num_template);
    }  
 
    // create copy of inverted pixel image
    hsv_img.copyTo(filtered, green_mask_inv);
 
    // create a kernel (3x3) for img filtering
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
    //std::cout << "   4 "  << std::endl;
    // For each green blob in the original image containing a digit
    int victim_count = -1;
    for (int i=0; i<bound_rect.size(); ++i){
      cv::Mat processROI(filtered, bound_rect[i]); // extract the ROI containing the digit
 
      if (processROI.empty()) continue;
      victim_count = victim_count+1;
      std::cout << "victim_count: " << victim_count << std::endl;  
      
      //Resize the processROI as the size of the number in the template image should be similar to the dimension
      cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
      cv::threshold( processROI, processROI, 100, 255, 0 );   // threshold and binarize the image, to suppress some noise
   
      // Apply some additional smoothing and filtering
      cv::erode(processROI, processROI, kernel);
      cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
      cv::erode(processROI, processROI, kernel);
      //std::cout << "   4.1 "  << std::endl;
      // Find the template digit with the best matching
      double maxScore = 0;
      int maxIdx = -1;
      cv::Mat rot_processROI(filtered, bound_rect[i]);
      for(int k=0;k<36;++k){
        //Rotate processROI
        rot_processROI = rotate_mat(processROI, 10*k);
       
        for (int j=0; j<templROIs.size(); ++j) {
          cv::Mat result;
 
          // Matching the ROI with the templROIs j-th
          cv::matchTemplate(rot_processROI, templROIs[j], result, cv::TM_CCOEFF);
          double score;
          cv::minMaxLoc(result, nullptr, &score);
 
          // Comparing the score with the others, if it is higher save this as the best match!
          if (score > maxScore) {
            maxScore = score;
            maxIdx = j;
 
            //cv::imshow("ROI", rot_processROI);
          }
        }
      }
      //std::cout << "   5 "  << std::endl;
      victim_list.at(victim_count).first = maxIdx + 1;
      // Display the best fitting number
      std::cout << "Recognized Digit: " << maxIdx + 1 << std::endl;
      //cv::waitKey(0);
    }
 
    sort(victim_list.begin(), victim_list.end(), sort_pair);
    //cv::imshow("Original", contours_img);
    //cv::waitKey(0);
 
    std::cout << "\n\n - - - @@@ end Digit recognition - - - \n\n\n";   
  return true;
    
  //----------------------------------------------------------------------

  }


  bool processRobot(const cv::Mat& hsv_img, const double scale, Polygon& triangle, double& x, double& y, double& theta){

    cv::Mat blue_mask;    
     
    cv::inRange(hsv_img, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), blue_mask);

    // Process blue mask
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

     //cv::imshow("filterrrr", blue_mask);
     //cv::waitKey(0);

     //cv::Mat contours_img;
     //contours_img = hsv_img.clone();

     //drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA);
     //std::cout << "N. contours: " << contours.size() << std::endl;

       
    bool found = false;
    for (int i=0; i<contours.size(); ++i)
    {
      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      
      cv::approxPolyDP(contours[i], approx_curve, 10, true);
      contours_approx = {approx_curve};

       //cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

      double area = cv::contourArea(approx_curve);
      
      if (approx_curve.size() != 3) continue;
      
      if (area < 300 || area>3000) continue;
      
      
      found = true;
      break;
    }

    if (found) 
    {
      for (const auto& pt: approx_curve) {
        triangle.emplace_back(pt.x/scale, pt.y/scale);
      }

      double cx = 0, cy = 0;
      for (auto item: triangle) 
      {
        cx += item.x;
        cy += item.y;
      }
      cx /= triangle.size();
      cy /= triangle.size();

      double dst = 0;
      Point vertex;
      for (auto& item: triangle)
      {
        double dx = item.x-cx;      
        double dy = item.y-cy;
        double curr_d = dx*dx + dy*dy;
        if (curr_d > dst)
        { 
          dst = curr_d;
          vertex = item;
        }
      }
      
       //cv::Moments m = cv::moments(approx_curve, false);
       //cv::Point center(m.m10/m.m00, m.m01/m.m00);
       //cv::Vec4f line;
       //cv::fitLine(approx_curve, line, cv::DIST_L2, 0, 0.01, 0.01);
       //cv::line(warpedFrame, cv::Point(line[2], line[3]), cv::Point(line[2]+line(0)*80, line(3)+line(1)*80), (0,255,0), 2);


     //cv::line(contours_img, center*scale, vertex*scale, (0,255,0), 2);
     //cv::circle(contours_img, center*scale, 20, cv::Scalar(0,0,0), -1);

      double dx = cx-vertex.x;
      double dy = cy-vertex.y;

      x = cx;
      y = cy;
      theta = std::atan2(dy, dx);


      //covariance = {};

      //std::cout << xc_m << " " << yc_m << " " << theta*180/M_PI << std::endl;
    }

     //cv::imshow("Original", contours_img);
     //cv::waitKey(0);

    return found;
  }

void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED" ); 
    cv::warpPerspective(img_in, img_out, transf, img_in.size());  
  }

  bool processMap(const cv::Mat& img_in, const double scale, std::vector<Polygon>& obstacle_list, std::vector<std::pair<int,Polygon>>& victim_list, Polygon& gate, const std::string& config_folder){
    //throw std::logic_error( "STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED" );  
    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    
    const bool res1 = processObstacles(hsv_img, scale, obstacle_list);
    if(!res1) std::cout << "processObstacles return false" << std::endl;
    const bool res2 = processGate(hsv_img, scale, gate);
    if(!res2) std::cout << "processGate return false" << std::endl;
    const bool res3 = processVictims(hsv_img, scale, victim_list);
    if(!res3) std::cout << "processVictims return false" << std::endl;

    return res1 && res2 && res3; 
  }

  bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){
    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);    
    //cv::imshow("HSV", hsv_img);
      //cv::waitKey(0);
    return processRobot(hsv_img, scale, triangle, x, y, theta);
    //throw std::logic_error( "STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED" );    
  }

  bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b){
    return (a.first < b.first);
}

  struct path_pos{
        double x;
        double y;
        double theta;
        int path_ind;
        int parent_ind;
        double cost;
    };


  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder){
    //Test dubins
    // double x1=0.2;
    // double x2=0.9;
    // float x3= 1.4;

    // double y1=0.2;
    // double y2=0.8;
    // float y3=0.2;

    // double theta1= 0.0;
    // double theta2= 0.0;
    // float theta3= 0.0;

    // double s;
    // int npts=50;

    // int kmax=10;
    // dubinsCurve dubins={};

    //           //-----------INITIAL POINT TO GOAL PATH-----------------

    //           dubins_shortest_path(dubins, x1, y1, theta1, x2, y2, theta2, kmax);
    //           discretize_arc(dubins.arc_1, s, npts, path);
    //           discretize_arc(dubins.arc_2, s, npts, path);
    //           discretize_arc(dubins.arc_3, s, npts, path);


    // std::cout << "inside planPath" << std::endl;
    // float xc = 0, yc = 1.5, r = 1.4;
    // float ds = 0.05;
    // for (float theta = -M_PI/2, s = 0; theta<(-M_PI/2 + 1.2); theta+=ds/r, s+=ds) {
    //   path.points.emplace_back(s, xc+r*std::cos(theta), yc+r*std::sin(theta), theta+M_PI/2, 1./r);    
    // }

    // return true;

    //throw std::logic_error( "STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED" );

    int kmax = 10;      // Max angle of curvature
    int npts = 50;  // Standard discretization unit of arcs
    //---------------------------------------------------------Knowing the Arena-----------------------------------------------
    // --------------------------------------GATE ORIENTATION-------------------------------------
    double gate_x = (gate[0].x + gate[1].x + gate[2].x + gate[3].x)/4;
    double gate_y = (gate[0].y + gate[1].y + gate[2].y + gate[3].y)/4;
 
    // Compute gate orientation
    double gate_th;
    //left corners (when distance of gate center from left border is less than the right border)
    // Left 
    if (fabs(gate_x - borders[0].x) < fabs(gate_x - borders[1].x)){
      // Bottom-left
      if (fabs(gate_y - borders[0].y) < fabs(gate_y - borders[3].y)){
        // Horizontal
        if (fabs(gate_y - borders[0].y) < fabs(gate_x - borders[0].x)){
          gate_th = -M_PI/2;
        // Vertical
        } else {
          gate_th = M_PI;
        }
      // Top-left
      } else {
        // Horizontal
        if (fabs(gate_y - borders[3].y) < fabs(gate_x - borders[0].x)){
          gate_th = M_PI/2;
        // Vertical
        } else {
          gate_th = M_PI;
        }
      }
      // right corners (when distance of gate center from right border is less than the left border)
  // Right
    } else {
      // Bottom-right
      if (fabs(gate_y - borders[0].y) < fabs(gate_y - borders[3].y)){
        // Horizontal
        if (fabs(gate_y - borders[0].y) < fabs(gate_x - borders[1].x)){
          gate_th = -M_PI/2;
        // Vertical
        } else {
          gate_th = 0;
        }
      // Top-right
      } else {
        // Horizontal
        if (fabs(gate_y - borders[3].y) < fabs(gate_x - borders[1].x)){
          gate_th = M_PI/2;
        // Vertical
        } else {
          gate_th = 0;
        }
      }    
    }

    std::vector<double> gate_orientn = {gate_x, gate_y, gate_th};

    //---------------------------------------------------------------- VICTIM CENTER-------------------------------------------------
    std::vector<Point> victim_center;
    double victim_x;
    double victim_y;
 
    for (int i = 0; i < victim_list.size(); i++){
        victim_x = 0;
        victim_y = 0;
        Polygon curr_poly = std::get<1>(victim_list[i]);
        for (int pt = 0; pt < curr_poly.size(); pt++){
            victim_x += curr_poly[pt].x;
            victim_y += curr_poly[pt].y;
        }
        victim_x /= curr_poly.size();
        victim_y /= curr_poly.size();
        
        victim_center.emplace_back(victim_x, victim_y);
    }
 
    //-------------------------------------------------------------OBSTACLE CENTER---------------------------------------------------
    std::vector<double> obs_radius;
    std::vector<Point> obs_center;
   
    double obs_x;
    double obs_y;
 
    //Center
    for (int i = 0; i < obstacle_list.size(); i++){
        obs_x = 0;
        obs_y = 0;
        Polygon curr_poly = obstacle_list[i];
        for (int pt = 0; pt < curr_poly.size(); pt++){
            obs_x += curr_poly[pt].x;
            obs_y += curr_poly[pt].y;
        }
        obs_x /= curr_poly.size();
        obs_y /= curr_poly.size();
        obs_center.emplace_back(obs_x, obs_y);
    }
    //Radius
    for (int i = 0; i < obstacle_list.size(); i++){
        double max_dist = 0.0;
        Polygon curr_poly = obstacle_list[i];
        for (int pt = 0; pt < curr_poly.size(); pt++){
            double dist = sqrt(pow((curr_poly[pt].x-curr_poly[(pt+1) % curr_poly.size()].x),2)+pow((curr_poly[pt].y-curr_poly[(pt+1) % curr_poly.size()].y),2));
            if(dist > max_dist){
                max_dist = dist;
            }  
        }
        obs_radius.emplace_back(max_dist / 2.0);
    }
 
   //------------------------------------------------MISSION 1---------------------------------------------------- 

  std::vector<Point> major_points_path; // (path with all the goal points)

  if (mission_1){

    //Add victims as major points
    major_points_path.push_back(Point(x,y));
    for (int i = 0; i < victim_center.size(); i++){
        major_points_path.push_back(victim_center[i]);
    }
    major_points_path.push_back(Point(gate_x, gate_y));

    double path_length = 0;

    RRT(theta, path, major_points_path, borders, kmax, npts, obstacle_list, obs_radius, obs_center, path_length, gate_orientn);


  } else {
  
    std::cout << "\n\n----------- invalid mission-------- \n\n\n";

    } 
    
    std::cout << "\n\n------ planning end------- \n\n\n";

  } 
  //----------------planning end----------------

  void RRT(const float theta, Path& path, std::vector<Point>& major_points_path, const Polygon& borders, int kmax, int npts, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center, double& path_length, const std::vector<double> gate_orientn){

    //List of all current nodes
  std::vector<path_pos> nodes_list;
  //List of all current paths
  std::vector<Path> paths_list;

  double s;
  //generate random like number (for sampling)
  srand(time(NULL)); 
  int MAX_X = (borders[1].x*100);
  int MIN_X = (borders[0].x*100);
  int MAX_Y = (borders[3].y*100);
  int MIN_Y = (borders[0].y*100);
  int sample_x = 0;
  int sample_y = 0;
  double q_rand_x = 0;
  double q_rand_y = 0;
  int rand_count = 1;

  path_pos first_node = {}; //First node in the tree

  std::vector<Pose> temp_path; //Temporary path from point to point

  int goal = 1; //The first goal is the first number
  bool failed = false;
  bool trying = false;
  
  int wrong_initial_angles = 0;

  while(goal < major_points_path.size()){
  
    double best_goal_cost = 100000; 
    path_pos best_goal_pos = {};  

    std::cout << "Current goal: " << goal << std::endl;
    nodes_list.clear();
    nodes_list.shrink_to_fit(); //For memory purposes
    paths_list.clear();
    paths_list.shrink_to_fit(); //For memory purposes

    //-------------------------------------Robot Initialize with robot position (x,y,theta)------------------------------
    if(goal == 1){
        first_node.x = major_points_path[0].x;
        first_node.y = major_points_path[0].y;
        first_node.theta = theta; 
    }
    //If not goal = 1, take the last position in Path
    else{
        Pose p = path.points.back();
        first_node.x = p.x;
        first_node.y = p.y;
        first_node.theta = p.theta;
    }
    //----------------------------------------------------

    // -----------------------------------------------RRT* 1 - List of nodes and paths----------------------
    Path p;
    nodes_list.push_back(first_node);
    paths_list.push_back(p); //Adding empty path for indexing purposes

    //-----------------------------------------------RRT 2 - Reach each major point (major point path)-----------------
    bool goalReached = false;
    int loop_cnt = 1;
    
    bool atLeastOneFound = false;

    while(goalReached == false){
       
        //-------------------------------------CONDITION :Reset if not found for too long----------------------------------
        if(nodes_list.size() > max_nodes or loop_cnt == max_loops){
        
          if (loop_cnt == max_loops){
            loop_cnt = 1;
            
              throw std::runtime_error("Resetting: NOT CONVERGING");
            // }
            
          } else {
            std::cout << "Resetting: TOO MANY NODES" << std::endl;
          }
        
        // Clear lists and keep only 1st element
            path_pos l = nodes_list.at(0);
            Path lp = paths_list.at(0);
            nodes_list.clear();
            nodes_list.shrink_to_fit();                                
            paths_list.clear();
            paths_list.shrink_to_fit();
            nodes_list.push_back(l);
            paths_list.push_back(lp);         
        }
        //---------------------------------------------Resetting end----------------------------
        // Count loops
        if (loop_cnt % 100 == 0){
        std::cout << "Loop: " << loop_cnt << std::endl;
      }
      loop_cnt++;
      
      
    // --------------------------------------------------------RRT* -------------------------------------------
    
   
    if (RRT_STAR) {  
    
    
  
        int index = 0;
        double tmp_cost = 100000;
        double best_x = 0;
        double best_y = 0;
        double best_theta = 0;
        Path best_path = {};

        //--------------------------------------------------RRT 3 & 4 - Compute next random point------------
        

          sample_x = rand()%(MAX_X-MIN_X+1)+MIN_X;
          sample_y = rand()%(MAX_Y-MIN_Y+1)+MIN_Y;

          q_rand_x = sample_x/100.00;
          q_rand_y = sample_y/100.00;
          rand_count += 1;
          if (rand_count % 5 == 0){ // Pick goal every 5 iterations
          
              q_rand_x =  major_points_path[goal].x;
              q_rand_y =  major_points_path[goal].y;
              trying = true;
          }

          //--------------------------------------------RRT  5 - Find parent node (lowest cost)----------------
        // Make sure able to reach parent
        
        for(int i=0; i<nodes_list.size(); i++){
        
              double dist_points = sqrt(pow((q_rand_x-nodes_list.at(i).x),2)+pow((q_rand_y-nodes_list.at(i).y),2));
              
              if(dist_points > 0.1){
              
             double ANGLE = 0.0;

            // If going to next (not last) goal
            if (q_rand_x == major_points_path[goal].x and q_rand_y == major_points_path[goal].y and major_points_path.size() != goal+1){
              ANGLE = calculate_angle(Point(nodes_list.at(index).x,nodes_list.at(index).y), major_points_path[goal+1]);
            // If point is gate, arrive orthogonally
            } else if(q_rand_x == gate_orientn[0] and q_rand_y == gate_orientn[1]){
              ANGLE = gate_orientn[2];
            // If point before next goal or next goal is last
            } else {
              ANGLE = calculate_angle(Point(nodes_list.at(index).x,nodes_list.at(index).y), major_points_path[goal]);
            }
          
            // Loop to use different arrival angles 
            for (int ang = 0; ang < 3; ang++){
              
              ANGLE += (ang-1)*0.349;   // +/- 20°
              
          
              Path new_path;
              dubinsCurve dubins = {};

              // Finding a path from one initial point to goal
              dubins_shortest_path(dubins, nodes_list.at(i).x, nodes_list.at(i).y, nodes_list.at(i).theta, q_rand_x, q_rand_y, ANGLE, kmax);

              // Dicretize the 3 arcs
              discretize_arc(dubins.arc_1, s, npts, new_path); // Arc 1
              discretize_arc(dubins.arc_2, s, npts, new_path); // Arc 2
              discretize_arc(dubins.arc_3, s, npts, new_path); // Arc 3

              bool collision = false;

              //------------ARENA BORDERS--------------------------
              for(int j=0; j<new_path.points.size(); j++){
                if(new_path.points.at(j).x < (borders[0].x + 0.02) or new_path.points.at(j).x > (borders[1].x - 0.02) or new_path.points.at(j).y < (borders[0].y + 0.02)  or new_path.points.at(j).y > (borders[3].y - 0.02)){
                  collision = true;
                  if(trying){failed = true; trying = false;}        
                  break; 
                }

                //----------OBSTACLE COLLISION--------------------
                for(int k=0; k<obstacle_list.size(); k++){
                  double obs_dist = sqrt(pow((new_path.points.at(j).x-obs_center.at(k).x),2)+pow((new_path.points.at(j).y-obs_center.at(k).y),2));
                  double result = inside_polygon(obstacle_list.at(k), Point(new_path.points.at(j).x,new_path.points.at(j).y)); 
                   
                  if(result != 1 or obs_dist < (obs_radius.at(k)+0.05)){
                    collision = true;
                    if(trying){failed = true;trying = false;}
                    break; 
                  }
                }
              } //-------COLLISION CHECK END-----------------------------
                
                // Update if not collision and better cost
                if (!collision and dubins.L + nodes_list.at(i).cost < tmp_cost){
                
                  index = i;        
                  tmp_cost = dubins.L + nodes_list.at(i).cost;
                  best_x = new_path.points.back().x;
                  best_y = new_path.points.back().y;
                  best_theta = new_path.points.back().theta;
                  best_path = new_path;
              }
              
          } // End angle variations
                
        } // End if condition (distance)
            
      } // End loop parent search


        //-----------------------------------------IF PARENT FOUND-----------------------------------------------
        if (tmp_cost < 100000){ 
        
          failed = false;
          
          std::cout << "New node created" << std::endl;
  
        path_pos new_node = {};
        new_node.x = best_x;
        new_node.y = best_y;
      
        new_node.theta = best_theta;
        new_node.path_ind = nodes_list.size();

        //------------------------------------------RRT 8----------------------------
        new_node.parent_ind = index;
        new_node.cost = tmp_cost;
      
        nodes_list.push_back(new_node);
        paths_list.push_back(best_path);

        //-----------------------------------------------RRT 9 - Check if goal reached------------------
          if(sqrt(pow((new_node.x - major_points_path.at(goal).x),2)+pow((new_node.y - major_points_path.at(goal).y),2)) < 0.1){ 
              
              //goalReached = true;
              atLeastOneFound = true; 
              std::cout << "Goal number" << goal << " reached" << std::endl;
              
              if(tmp_cost < best_goal_cost){
                best_goal_cost = tmp_cost;
                best_goal_pos = nodes_list.back();
              }
              
          }
      }

        //if(goalReached){
        if(atLeastOneFound and (nodes_list.size() > least_nodes or loop_cnt == 1000)){   
        

            path_pos pos = best_goal_pos;
            goalReached = true;
            goal += 1;
     
            while(pos.path_ind != 0){
                
                temp_path.insert(temp_path.begin(),paths_list.at(pos.path_ind).points.begin(),paths_list.at(pos.path_ind).points.end());
               
              // Go backwards to parent  
              pos = nodes_list.at(pos.parent_ind);

            }
        }
        
        
      } // RRT_STAR  
        
    } // Goal reached

    //-------------------------------------------------------- Add points to final path---------------------------------
    path.points.insert(path.points.end(), temp_path.begin(), temp_path.end());
    temp_path.clear();
    temp_path.shrink_to_fit();

    for (int i = 0; i < path.points.size()-1; i++){
      path_length += sqrt(pow(path.points[i].x-path.points[i+1].x,2) + pow(path.points[i].y-path.points[i+1].y,2));
      
    }

  }
  
  std::cout << "\n----------- RRT* OVER with path length: "<< path_length <<' ' <<" ---------- \n\n";


  }

  double calculate_angle(Point a, Point b){
  
    double angle = atan2(fabs(a.y - b.y), fabs(a.x - b.x));
    
    if (b.x > a.x and b.y < a.y){
      angle = -angle;
    } else if (b.x < a.x and b.y > a.y){
      angle = M_PI-angle;
    } else if (b.x < a.x and b.y < a.y){
      angle = M_PI+angle;
    }
  
  return angle;
        
}

  int inside_polygon(Polygon obstacle, Point pt){
    int counter = 0;
    double xinters;
    int N = obstacle.size();
    Point p1, p2;
 
    p1 = obstacle.at(0);
    // Iterate through obstacles
    for(int i = 1; i <= N; i++){
        p2 = obstacle.at(i % N);
        if(pt.y > std::min(p1.y, p2.y)){
            if(pt.y <= std::max(p1.y, p2.y)){
                if(pt.x <= std::max(p1.x, p2.x)){
                    if(p1.y != p2.y){
                        xinters = (pt.y - p1.y) *(p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                        if(p1.x == p2.x or pt.x <= xinters){
                        counter++;                     
                        }
                    }
                }          
            }
        }
        p1 = p2;
    }
 
    if(counter % 2 == 0){
        return 1;  
    }else{
        return 0;  
    }
  }

}//CODE END

