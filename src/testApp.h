#pragma once

#include "ofMain.h"
#include "ofxKinect.h"
#include "opencv2/opencv.hpp"
#include "ofxCv.h" 
#include "segmenter.h"
#include "histogram.h"

struct object_state {
    int label;
    int state;
};

class testApp : public ofBaseApp{

public:
    void setup();
    void update();
    void draw();

    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);

    ofxKinect kinect;
    ofImage depthImage;
    ofImage colorImage;
    ofImage threshImage;
    ofImage segImage;
    ofImage bgImage;
    ofImage fgImage;
    ofImage distImage;
    ofImage hist;
    Mat m_hist;
    

    int minHeadHeight;
    int nearThreshold;
    int farThreshold;

    int entryCount;
    int exitCount;

    int entryStayBoundary;
    int entryLeaveBoundary;

    int exitStayBoundary;
    int exitLeaveBoundary;

    map<int, int> object_states;
    map<int, int> object_hexes;

    ofxCv::ContourFinder contourFinder;
    ofxCv::ContourFinder headFinder;
    WatershedSegmenter segmenter;
    Histogram1D histd;
		
};
