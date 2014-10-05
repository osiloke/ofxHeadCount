//
//  segmenter.h
//  headCount
//
//  Created by Osiloke Emoekpere on 1/27/14.
//
//

#ifndef headCount_segmenter_h
#define headCount_segmenter_h

#include "opencv2/opencv.hpp"

using namespace cv;

class WatershedSegmenter{
private:
    cv::Mat markers;
    cv::Mat bg,fg,dist,dist_8u;
public:
    
    const Mat getBackground() const{
        return bg;
    }
    const Mat getDistance() const{
        return dist;
    }
    const Mat getDistance8U() const{
        return dist_8u;
    }
    const Mat getForeground() const{
        return fg;
    }
    void update();
    void createMarkers(const cv::Mat threshImage){
        auto const structure = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                         cv::Size(5, 5));
        
        cv::dilate(threshImage, bg, cv::Mat(), cv::Point(-1, 1), 4);
//        cv::dilate(threshImage, fg, structure, cv::Point(-1, 1), 4);
        cv::erode(threshImage, fg, structure, cv::Point(-1, 1), 4);
        cv::threshold(bg, bg, 1, 128, cv::THRESH_BINARY_INV);
        
        // Perform the distance transform algorithm 
        cv::distanceTransform(threshImage, dist, CV_DIST_L2, 3);
        
        // Normalize the distance image for range = {0.0, 1.0}
        // so we can visualize and threshold it
        cv::normalize(dist, dist, 0, 1., cv::NORM_MINMAX);
        
        // Create the CV_8U version of the distance image
        // It is needed for cv::findContours() 
        dist.convertTo(dist_8u, CV_8U);
        
        // Find total markers
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
        
        int ncomp = contours.size();
        
        // Create the marker image for the watershed algorithm
        markers = cv::Mat::zeros(dist.size(), CV_32SC1);
        
        // Draw the foreground markers
        for (int i = 0; i < ncomp; i++)
            cv::drawContours(markers, contours, i, cv::Scalar::all(i+1), -1);
        
        // Draw the background marker
        cv::circle(markers, cv::Point(5,5), 3, CV_RGB(255,255,255), -1);
        
        
//        markers = cv::Mat(threshImage.size(), CV_8U);
//        markers = fg + bg;

        return markers;
    }
    
    cv::Mat update(const cv::Mat threshImage, const cv::Mat &image) {
//        Mat im_coloured = Mat::zeros(threshImage.rows, threshImage.cols,CV_8UC3);
//        
//        vector<Mat> planes;
//        
//        for(int i=0;i<3;i++)
//            planes.push_back(threshImage);
//        
//        merge(planes,im_coloured);
        createMarkers(threshImage);
        cv::watershed(image, markers);
        return markers;
    }
    
    cv::Mat getSegmentation(){
        cv::Mat tmp;
        markers.convertTo(tmp, CV_8U);
        return tmp;
    }

    cv::Mat getWatersheds(){
        cv::Mat tmp;
        markers.convertTo(tmp, CV_8U,255,255);
        
        return tmp;
    }
};

#endif
