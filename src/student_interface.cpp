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

  const int max_nodes=100;
  const int max_loops=2000;
  const int min_nodes=20;
  //-----------------------------------------
  void rrt_star(const float theta, Path& path, std::vector<Point>& major_points_path, const Polygon& borders, int kmax, int npts, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center, double& path_length, const std::vector<double> gate_orientn);
  int inside_polygon(Polygon obs, Point pt);
  double calculate_angle(Point a, Point b);
  bool sort_pair(const std::pair<int,Polygon>& a, const std::pair<int,Polygon>& b);
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
    
    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
    //cv::imshow("HSV", hsv_img);
    //cv::waitKey(0);

    cv::Mat red_mask_low, red_mask_high, red_mask;     
    cv::inRange(hsv_img, cv::Scalar(0, 102, 86), cv::Scalar(40, 255, 255), red_mask_low);
    cv::inRange(hsv_img, cv::Scalar(164, 102, 86), cv::Scalar(180, 255, 255), red_mask_high);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); 

    // cv::Mat img_small;
    // cv::resize(red_mask, img_small, cv::Size(640, 512));

    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    // Process red mask
    //contours_img = hsv_img.clone();
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // cv::imshow("filterrrr red", red_mask);
    // cv::waitKey(0);
    //drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
    //cv::imshow("contours", contours_img);
    //std::cout << "N. contours: " << contours.size() << std::endl;
    //std::cout << "contours image " << contours_img << std::endl;

    for (int i=0; i<contours.size(); ++i)
    {
      //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 3, true);

      Polygon scaled_contour;
      for (const auto& pt: approx_curve) {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }
      obstacle_list.push_back(scaled_contour);
      contours_approx = {approx_curve};
      drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
      //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }
    //std::cout << std::endl;
    //cv::imshow("Original", contours_img);
    //cv::waitKey(0);

    //-------------------inflated obstacles-------------
    
    std::vector<Polygon> inflated_obs;

    const double inflate_int=500;
    for(int obs=0; obs<obstacle_list.size(); ++obs){

      ClipperLib::Path srcPoly;
      ClipperLib::Paths newPoly;
      ClipperLib::ClipperOffset co;


      // iterating through obs and adding inflate
      for(int vertex=0; vertex<obstacle_list[obs].size(); ++vertex){
        int x=obstacle_list[obs][vertex].x*inflate_int;
        int y=obstacle_list[obs][vertex].y*inflate_int;
        srcPoly<< ClipperLib::IntPoint(x,y);

      }

      co.AddPath(srcPoly, ClipperLib::jtSquare, ClipperLib::etClosedPolygon);
      co.Execute(newPoly,50);

      for(const ClipperLib::Path &path: newPoly){

        Polygon curr_obs;
        for (const ClipperLib::IntPoint &pt: path){
          double x=pt.X/inflate_int;
          double y=pt.Y/inflate_int;
          curr_obs.emplace_back(x,y);
      }

      inflated_obs.push_back(curr_obs);
      obstacle_list[obs]=curr_obs;

    }
  }
  std::cout << "obstacle offset success"  << std::endl;
    return true;
  }

  bool processGate(const cv::Mat& hsv_img, const double scale, Polygon& gate){
    
    // Find purple regions
    cv::Mat purple_mask;
    cv::inRange(hsv_img, cv::Scalar(45, 50, 26), cv::Scalar(100, 255, 255), purple_mask);    
    //cv::inRange(hsv_img, cv::Scalar(130, 10, 10), cv::Scalar(165, 255, 255), purple_mask);
    
    
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    cv::Mat contours_img;

    // Process purple mask
    contours_img = hsv_img.clone();
    cv::findContours(purple_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
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

std::string template_folder = "/home/ubuntu/workspace/template/";
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

  struct path_pose{
        double x;
        double y;
        double theta;
        int path_ind;
        int parent_ind;
        double cost;
    };


  bool planPath(const Polygon& borders, const std::vector<Polygon>& obstacle_list, const std::vector<std::pair<int,Polygon>>& victim_list, const Polygon& gate, const float x, const float y, const float theta, Path& path, const std::string& config_folder){
    
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

    std::cout<< "\n\n---------in path Planning----------\n\n\n";
    int kmax=10;
    int npts=50;

    //----------Finding Centers of all the objects (knowing the Arena)---------
    //----------Obstacles---------------------------
    
    std::vector<Point> obs_center; //obstacle center
    std::vector<double> obs_radius; //obstacle radius

    double obs_x;
    double obs_y;

    // finding center
    for(int i=0; i<obstacle_list.size(); i++){
      obs_x=0;
      obs_y=0;

      Polygon curr_poly= obstacle_list[i];
      for(int pt=0; pt< curr_poly.size(); pt++){
        obs_x+= curr_poly[pt].x;
        obs_y+= curr_poly[pt].y;
      }

      obs_x/= curr_poly.size();
      obs_y/= curr_poly.size();
      obs_center.emplace_back(obs_x,obs_y);

    }
    //Printing centers
    // for (int i = 0; i < obs_center.size(); i++)
    // std::cout << obs_center[i] << ' ';
    //finding radius
    for(int i=0; i<obstacle_list.size(); i++){
      double max_dist=0.0;
      Polygon curr_poly= obstacle_list[i];

      for(int pt=0; pt< curr_poly.size(); pt++){
        double dist=sqrt(pow((curr_poly[pt].x - curr_poly[(pt+1) % curr_poly.size()].x),2) + pow((curr_poly[pt].y - curr_poly[(pt+1) % curr_poly.size()].y),2));
        if(dist > max_dist){
          max_dist=dist;
        }
      }

    
      obs_radius.emplace_back(max_dist/2.0);
    }


    //-----------gate-------------------------------

    //center
    //As gate is a rectangle with 4 sides
    double gate_x=(gate[0].x+gate[1].x+gate[2].x+gate[3].x)/4;
    double gate_y=(gate[0].y+gate[1].y+gate[2].y+gate[3].y)/4;

    //orientation
    double gate_th;
    //left corners (when distance of gate center from left border is less than the right border)

    if(fabs(gate_x - borders[0].x) < fabs(gate_x - borders[1].x)){
      //bottom
      if(fabs(gate_y - borders[0].y)<fabs(gate_y - borders[3].y)){
        //horizontal
        if(fabs(gate_y - borders[0].y)< fabs(gate_x - borders[0].x)){
          gate_th= -M_PI/2;
          std::cout << "Gate- Left-bottom-horizontal" << std::endl;
        }
        else{//vertical
          gate_th= M_PI;
          std::cout << "Gate- Left-bottom-vertical" << std::endl;
        }

      }
      //top
      else{
        //horizontal
        if(fabs(gate_y - borders[3].y)<fabs(gate_x - borders[0].x)){
          gate_th= M_PI/2;
          std::cout << "Gate- Left-top-horizontal" << std::endl;

        }
        else{ //vertical
          gate_th= M_PI;
          std::cout << "Gate- Left-top-vertical" << std::endl;
        }
      }

      }

    // right corners (when distance of gate center from right border is less than the left border)
    else{
      //bottom
      if(fabs(gate_y - borders[0].y)<fabs(gate_y - borders[3].y)){
        //horizontal
        if(fabs(gate_y - borders[0].y)<fabs(gate_x - borders[1].x)){
          gate_th= -M_PI/2;
          std::cout << "Gate- right-bottom-horizontal" << std::endl;

        }
        else{//vertical
          gate_th=0;
          std::cout << "Gate- right-bottom-vertical" << std::endl;

        }

      }
      else{ //top
        //horizontal
        if(fabs(gate_y - borders[3].y) < fabs(gate_x - borders[1].x)){
          gate_th= M_PI/2;
          std::cout << "Gate- right-top-horizontal" << std::endl;
        }
        else{
          gate_th=0;
          std::cout << "Gate- right-top-vertical" << std::endl;
        }

      }

    
    }
    
    std::vector<double> gate_orientn= {gate_x, gate_y, gate_th};


    //-----------victims-------------------------------

    std::vector<Point> victim_center;
    double victim_x;
    double victim_y;

    for(int i=0; i<victim_list.size(); i++){
      victim_x=0;
      victim_y=0;

      Polygon curr_poly=std::get<1>(victim_list[i]);
      for(int pt=0; pt<curr_poly.size(); pt++){
        victim_x+= curr_poly[pt].x;
        victim_y+=curr_poly[pt].y;
      }

      //Avg of coordinates
      victim_x/=curr_poly.size();
      victim_y/=curr_poly.size();
      std::cout<<"victim center  "<<i <<victim_x <<victim_y <<std::endl;
      victim_center.emplace_back(victim_x, victim_y);
    }

      //-----end arena information-------------------------------------------

        //Path newPath;
        // float s=0.0;
        // float x=0.0;
        // float y=0.0;
        // float theta=0.0;
        // float kappa=0.0;

        //path.points.emplace_back(0.0,0,.0,0.0,0.0,0.0);

    //----------------- Path planning---------------------------
    //mission 1
    std::vector<Point> major_points_path; //all victim centers as major points
    //Add initial point
    major_points_path.push_back(Point(x,y));

    for(int i =0; i< victim_center.size(); i++){
      major_points_path.push_back(victim_center[i]);

    }
    //Last point as gate center
    major_points_path.push_back(Point(gate_x, gate_y));

    double path_length=0;

    rrt_star(theta, path, major_points_path, borders, kmax, npts, obstacle_list, obs_radius, obs_center, path_length, gate_orientn);

    std::cout<< "\n\n--------- Planning end----------\n\n\n";

  } 
  //----------------planning end----------------

  void rrt_star(const float theta, Path& path, std::vector<Point>& major_points_path, const Polygon& borders, int kmax, int npts, const std::vector<Polygon>& obstacle_list, std::vector<double> obs_radius, std::vector<Point> obs_center, double& path_length, const std::vector<double> gate_orientn){

    //-----------------------------Initialize variables------------------------
    //List of all the current paths and nodes
    std::vector<Path> path_list;
    std::vector<path_pose> nodes;
    std::vector<Pose> temp_path; //temperory path

    double s;

    srand(time(NULL));//object to represent the current time.
    
    int x_max=(borders[1].x*100);
    int y_max=(borders[3].y*100);
    int x_min=(borders[0].x*100);
    int y_min=(borders[0].y*100);

    // int x_max=(borders[1].x);
    // int y_max=(borders[3].y);
    // int x_min=(borders[0].x);
    // int y_min=(borders[0].y);

    int sample_x=0;
    int sample_y=0;

    double q_rand_x=0;
    double q_rand_y=0;

    int rand_cnt=1;

    path_pose start_node={};

    int goal_number=1;
    bool failed= false;
    bool trying=false;

    int initial_angle_mistake=0;

    int index=0;
    double temp_cost=100000;
    double best_x=0;
    double best_y=0;
    double best_theta=0;
    Path best_path={};
    //---------end initializations------------

    //rrt_star algorithm, runs until all the major points(victims) are covered

    std::cout<<"rrt 1, major_points_path.size():  "<<major_points_path.size()<<std::endl;
    while(goal_number < major_points_path.size()){

      double best_cost= 100000;
      path_pose best_pos= {};

      std::cout<<"rrt 2, current goal number: "<<goal_number<<std::endl;
      nodes.clear();
      nodes.shrink_to_fit();//for memory
      path_list.clear();
      path_list.shrink_to_fit();
      //-----------------------FIRST GOAL CHECK---------------------------
      if(goal_number==1){
        start_node.x=major_points_path[0].x;
        start_node.y=major_points_path[0].y;
        start_node.theta=theta;
        std::cout<<" rrt 3, Initialized robot position ";
      }

      //if not the first goal then take the last element in the path
      else{
        std::cout<<" rrt 3, Else of Initialized robot ";
        Pose p=path.points.back();
        start_node.x=p.x;
        start_node.y=p.y;
        start_node.theta=p.theta;
      }
      //List of nodes and path
      Path path0;
      nodes.push_back(start_node);
      path_list.push_back(path0);

      bool goal_reached=false; //flag for Reaching each major point
      int loop_cnt=1;
      bool at_least_one_found=false;


      //-------------------------- GOAL NOT REACHED------------------------
      while (goal_reached==false){
        //std::cout<< "rrt 4, goal reached false"<< std::endl;
        //-----------------------condition 1 where ROBOT GETS STUCK-----------------
        //reset if taking too long
        if(nodes.size()>max_nodes or loop_cnt==max_loops){
          if(loop_cnt==max_loops){
            loop_cnt==1;
            throw std::runtime_error("resetting");

            }

          else{
            std::cout<< "resetting: Number of nodes exceeded"<< std::endl;
          }

          //keep only first element and clear the list
          path_pose back_to_pose1=nodes.at(0);
          Path back_to_path1= path_list.at(0);
          nodes.clear();
          nodes.shrink_to_fit();
          path_list.clear();
          path_list.shrink_to_fit();
          nodes.push_back(back_to_pose1);
          path_list.push_back(back_to_path1);
        }

        //count loops
        if(loop_cnt % 100==0){
          //std::cout << "loop_cnt: "<<loop_cnt<< std::endl;
        }
        loop_cnt++;
        //--------------initial part end--------------------------------------

        //-----next random point----------
        sample_x=rand()%(x_max - x_min +1)+x_min;
        sample_y=rand()%(y_max - y_min +1)+y_min;

        q_rand_x=sample_x/100.0;
        q_rand_y=sample_y/100.0;

        // q_rand_x=sample_x;
        // q_rand_y=sample_y;
        rand_cnt += 1;

        if(rand_cnt % 5 == 0){ //pick goal every 5 iterations

          q_rand_x = major_points_path[goal_number].x;
          q_rand_y = major_points_path[goal_number].y;
          trying=true;
        }
        //------PARENT NODE SEARCH------------------------------------

        for(int i=0; i<nodes.size(); i++){
          //std::cout<< "rrt 5, for loop nodes"<< std::endl;
          double pt_distances= sqrt(pow((q_rand_x- nodes.at(i).x),2) + pow((q_rand_y- nodes.at(i).y),2) );
          if(pt_distances>0.1){
            double angle= 0.0;

            //----------if random pt NEXT GOAL (NOT LAST)---------------------
            if(q_rand_x == major_points_path[goal_number].x and q_rand_y == major_points_path[goal_number].y and major_points_path.size() != goal_number+1){
              angle=calculate_angle(Point(nodes.at(index).x, nodes.at(index).y), major_points_path[goal_number+1]);
            }
            //---------- if random point GATE-----------------
            else if(q_rand_x == gate_orientn[0] and q_rand_y== gate_orientn[1]){
              angle=gate_orientn[2];
            }
            //-----------LAST GOAL--------------------------
            else{
              angle= calculate_angle(Point(nodes.at(index).x, nodes.at(index).y), major_points_path[goal_number]);
            }

            //different arrival angles

            for (int ang=0; ang<3; ang++){
              angle +=(ang-1)*0.349;

              Path new_path;

              dubinsCurve dubins={};

              //-----------INITIAL POINT TO GOAL PATH-----------------

              dubins_shortest_path(dubins, nodes.at(i).x, nodes.at(i).y, nodes.at(i).theta, q_rand_x, q_rand_y, angle, kmax);
              discretize_arc(dubins.arc_1, s, npts, new_path);
              discretize_arc(dubins.arc_2, s, npts, new_path);
              discretize_arc(dubins.arc_3, s, npts, new_path);

              bool collision=false;

              //------------ARENA BORDERS--------------------------
              for(int j=0; j<new_path.points.size(); j++){
                std::cout<< "rrt 6, collide with borders"<< std::endl;
                if(new_path.points.at(j).x < (borders[0].x+0.02) or new_path.points.at(j).x> (borders[1].x - 0.02) or new_path.points.at(j).y < (borders[0].y + 0.02) or new_path.points.at(j).y>(borders[3].y - 0.02) ){
                  collision= true;
                  if(trying){failed = true; trying= false;}
                  break;
                }

                //----------OBSTACLE COLLISION--------------------
                for(int k=0; k<obstacle_list.size(); k++){
                  double obs_dist = sqrt(pow((new_path.points.at(j).x - obs_center.at(k).x),2) + pow((new_path.points.at(j).y - obs_center.at(k).y),2));
                  double result= inside_polygon(obstacle_list.at(k), Point(new_path.points.at(j).x, new_path.points.at(j).y));
                  if(result !=1 or obs_dist < (obs_radius.at(k)+0.05)){
                    std::cout<< "rrt 7, obs collision"<< std::endl;
                    collision=true;
                    if(trying){failed=true; trying= false;}
                    break;
                  }
                }
              }
              //-------COLLISION CHECK END-----------------------------
              if(!collision and dubins.L + nodes.at(i).cost < temp_cost){

              //if(dubins.L + nodes.at(i).cost < temp_cost){

              index=i;
              temp_cost=dubins.L + nodes.at(i).cost;
              best_x=new_path.points.back().x;
              best_y=new_path.points.back().y;
              best_theta=new_path.points.back().theta;
              best_path= new_path;
            }

            }// angle var


            //---------UPDATE if no collision and better cost---------
            

          }//end if distance
        }// loop parent node search
        //----------IF PARENT FOUND------------------------------------

        if(temp_cost < 100000){
          failed=false;
          //std::cout << "new node"<<std::endl;

          path_pose new_node={};
          new_node.x=best_x;
          new_node.y=best_y;
          new_node.theta= best_theta;
          new_node.path_ind=nodes.size();

          //line 8

          new_node.parent_ind=index;
          new_node.cost= temp_cost;

          nodes.push_back(new_node);
          path_list.push_back(best_path);

          //line 9
          //---------------CHECK GOAL REACHED-------------------

          if(sqrt(pow((new_node.x - major_points_path.at(goal_number).x),2) + pow((new_node.y - major_points_path.at(goal_number).y),2)) < 0.1){
            at_least_one_found=true;
            std::cout<< "goal number: "<<goal_number<<"reached"<< new_node.x << new_node.y <<std::endl;

            if(temp_cost<best_cost){
              best_cost=temp_cost;
              best_pos=nodes.back();
            }
          } 

        }
        //std::cout<< "node size"<< nodes.size() <<std::endl;
        //std::cout<< "loop_cnt"<< loop_cnt<< std::endl;
        //std::cout<< "atleast one found"<<std::endl;

       if(at_least_one_found and (nodes.size()> min_nodes or loop_cnt == 1000)){
        std::cout<< "nodes"<<std::endl;
        //if(at_least_one_found){
          std::cout<< "atleast one found"<<std::endl;

          path_pose pos= best_pos;
          goal_reached=true;
          goal_number+=1;
          std::cout<< "goal_number increment"<< goal_number <<std::endl;
          std::cout<< "pos.path_ind"<< pos.path_ind <<std::endl;
          //std::cout<< "pos.parent_ind"<< pos.path_ind <<std::endl;

          while(pos.path_ind !=0){
            std::cout<<"in while";
            std::cout<< "pos.path_ind decrement in while" << pos.path_ind <<std::endl;            

            temp_path.insert(temp_path.begin(), path_list.at(pos.path_ind).points.begin(), path_list.at(pos.path_ind).points.end());

            pos=nodes.at(pos.parent_ind);
          }
        }

        //end RRT_STAR
      }//Goal Reached
      //-------------ADD PTS TO FINAL PATH--------------------------------------------

      path.points.insert(path.points.end(), temp_path.begin(), temp_path.end());
      temp_path.clear();
      temp_path.shrink_to_fit();

      for(int i=0; i<path.points.size()-1;i++){
        //std::cout<< "\n path.points for loop \n\n";

        path_length += sqrt(pow(path.points[i].x - path.points[i+1].x,2)+pow(path.points[i].y- path.points[i+1].y,2));

      }

      std::cout<< goal_number <<" : Path saved";




    }
    std::cout<< "\n -------RRT* END------- \n\n";



  }

  double calculate_angle(Point a ,Point b){
    double angle= atan2(fabs(a.y - b.y), fabs(a.x - b.x));

    if(b.x> a.x and b.y < a.y){
      angle=-angle;

    }
    else if(b.x < a.x and b.y>a.y){
      angle=M_PI-angle;

    }
    else if(b.x < a.x and b.y<a.y){
      angle=M_PI+angle;
      
    }
    return angle;
  }

  int inside_polygon(Polygon obs, Point pt){
    int cnt=0;
    double xint;
    int N= obs.size();
    Point p1,p2;

    p1 = obs.at(0);
    // Iterating through obstacles
    for(int i = 1; i <= N; i++){
        p2 = obs.at(i % N);
        if(pt.y > std::min(p1.y, p2.y)){
            if(pt.y <= std::max(p1.y, p2.y)){
                if(pt.x <= std::max(p1.x, p2.x)){
                    if(p1.y != p2.y){
                        xint = (pt.y - p1.y) *(p2.x - p1.x) / (p2.y - p1.y) + p1.x;
                        if(p1.x == p2.x or pt.x <= xint){
                        cnt++;                     
                        }
                    }
                }          
            }
        }
        p1 = p2;
    }
 
    if(cnt % 2 == 0){
        return 1;  
    }else{
        return 0;  
    }

  }

}//CODE END

