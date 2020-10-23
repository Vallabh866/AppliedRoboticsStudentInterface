// ocr.cpp:
// Use tesseract-ocr to recognize digits inside the green circles of the image

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>

const double MIN_AREA_SIZE = 100;

void processImage(const std::string& filename)
{
  // Load image from file
  cv::Mat img = cv::imread(filename.c_str());
  if(img.empty()) {
    throw std::runtime_error("Failed to open the file " + filename);
  }
  
  // Display original image
  cv::imshow("Original", img);
  
  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
  
  
  // Find green regions
  cv::Mat green_mask;
  cv::inRange(hsv_img, cv::Scalar(45, 30, 30), cv::Scalar(75, 255, 255), green_mask);

  // Apply some filtering
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
  cv::dilate(green_mask, green_mask, kernel);
  cv::erode(green_mask, green_mask, kernel);
  
  // Display image
  cv::imshow("GREEN_filter", green_mask);  
  
  
  
  // Find contours
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;

  contours_img = img.clone();
  cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
  
  std::vector<cv::Rect> boundRect(contours.size());
  for (int i=0; i<contours.size(); ++i)
  {
    double area = cv::contourArea(contours[i]);
    if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives
    approxPolyDP(contours[i], approx_curve, 2, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    boundRect[i] = boundingRect(cv::Mat(approx_curve)); // find bounding box for each green blob
  }
  cv::imshow("Original", contours_img);
  cv::waitKey(0);
  
  
  cv::Mat green_mask_inv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255,255,255));
  cv::bitwise_not(green_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
  
  cv::imshow("Numbers", green_mask_inv);
  cv::waitKey(0);

  // Create Tesseract object
  tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
  // Initialize tesseract to use English (eng) 
  ocr->Init(NULL, "eng");
  // Set Page segmentation mode to PSM_SINGLE_CHAR (10)
  ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);
  // Only digits are valid output characters
  ocr->SetVariable("tessedit_char_whitelist", "0123456789");
  
  img.copyTo(filtered, green_mask_inv);   // create copy of image without green shapes

  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2*2) + 1, (2*2)+1));
  
  // For each green blob in the original image containing a digit
  for (int i=0; i<boundRect.size(); ++i)
  {
    cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit
    
    if (processROI.empty()) continue;
    
    cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI
    cv::threshold( processROI, processROI, 100, 255, 0 ); // threshold and binarize the image, to suppress some noise
    
    // Apply some additional smoothing and filtering
    cv::erode(processROI, processROI, kernel);
    cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
    cv::erode(processROI, processROI, kernel);
    
    // Show the actual image passed to the ocr engine
    cv::imshow("ROI", processROI);
    
     // Set image data
    ocr->SetImage(processROI.data, processROI.cols, processROI.rows, 3, processROI.step);
    
    // Run Tesseract OCR on image and print recognized digit
    std::cout << "Recognized digit: " << std::string(ocr->GetUTF8Text()) << std::endl;
    
    cv::waitKey(0);
  }
  
  ocr->End(); // destroy the ocr object (release resources)

}

int main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <image>" << std::endl;
    return 0;
  }
  processImage(argv[1]);
  return 0;
}
