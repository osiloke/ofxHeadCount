#include "testApp.h"

#include "libfreenect.h" 
using namespace ofxCv;
using namespace cv;
 

//--------------------------------------------------------------
void testApp::setup(){
    
    //Setup contour finder
    contourFinder.setMinAreaRadius(30);
	contourFinder.setMaxAreaRadius(300);
	contourFinder.setThreshold(15);
	// wait for half a frame before forgetting something
	contourFinder.getTracker().setPersistence(1000000);
	// an object can move up to 32 pixels per frame
	contourFinder.getTracker().setMaximumDistance(320);
    
    headFinder.setMinAreaRadius(50);
	headFinder.setMaxAreaRadius(300);
	headFinder.setThreshold(15);
	// wait for half a frame before forgetting something
	headFinder.getTracker().setPersistence(1000000);
	// an object can move up to 32 pixels per frame
	headFinder.getTracker().setMaximumDistance(320);
    
    
    
    farThreshold = 146;
    nearThreshold = 250;
    
    entryCount = 0;
    exitCount = 0;
    
    entryStayBoundary = exitStayBoundary = kinect.height/2;
    entryLeaveBoundary = 90;
    exitLeaveBoundary = 400;
    
    kinect.setRegistration(true);
    
    kinect.init();
    
    kinect.open();
    
    // print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
    
    
    ofLogNotice() << "Entry Stay Boundary "<<entryStayBoundary;
    ofLogNotice() << "Entry Leave Boundary "<<entryLeaveBoundary;
    ofLogNotice() << "Exit Stay Boundary "<<exitStayBoundary;
    ofLogNotice() << "Exit Leave Boundary "<<exitLeaveBoundary;
    
    depthImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    colorImage.allocate(kinect.width, kinect.height, OF_IMAGE_COLOR);
    segImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    fgImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    bgImage.allocate(kinect.width, kinect.height, OF_IMAGE_GRAYSCALE);
    
    object_hexes[0] = 0x3054C9;
    
    ofSetFrameRate(60);
}

//--------------------------------------------------------------
void testApp::update(){
    ofBackground(100, 100, 100);
	
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
        Mat tempImage = Mat(kinect.height, kinect.width, CV_8UC1, kinect.getDepthPixels(), 0);
        Mat crImage = Mat(kinect.height, kinect.width, CV_8UC3, kinect.getPixels(), 0);
        Mat _thresh;
        
        Mat bg, dist, fg;
        copy(tempImage, _thresh);
        
        //Background subtraction
        unsigned char * pix = _thresh.data;
//        unsigned short * rawPix = kinect.getRawDepthPixels();
        
        int numPixels = _thresh.size().width * _thresh.size().height ;
         
        for (int i = 0; i < numPixels; i++ ){ 
            if(pix[i] < nearThreshold && pix[i] > farThreshold) {
//                pix[i] = 255;
            } else {
                pix[i] = 0; 
            }
        }
        
        
        //http://stackoverflow.com/questions/11294859/how-to-define-the-markers-for-watershed-in-opencv
        //https://opencv-code.com/tutorials/count-and-segment-overlapping-objects-with-watershed-and-distance-transform/#more-476
        
        segmenter.update(_thresh, crImage);
        
        Mat m_seg = segmenter.getSegmentation();
        toOf(m_seg, segImage);
        segImage.update();
        
        Mat m_fg = segmenter.getForeground();
        toOf(m_fg, fgImage);
        fgImage.update();
        
        Mat m_bg = segmenter.getBackground();
        toOf(m_bg, bgImage);
        bgImage.update();
        
        Mat m_dist = segmenter.getDistance8U();
        toOf(m_dist, distImage);
        distImage.update();
         
        blur(_thresh, 10);
        contourFinder.findContours(_thresh);
        
        toOf(_thresh, threshImage);
        threshImage.update();
        
        toOf(tempImage, depthImage);
        depthImage.update();
        cv::Mat adjMap;
        adjMap = histd.getHistogramImage(tempImage);
//http://stackoverflow.com/questions/13840013/opencv-how-to-visualize-a-depth-image
//        cv::convertScaleAbs(m_hist, m_hist, 255 / histd.getMinValue());
        
        // expand your range to 0..255. Similar to histEq();
        adjMap.convertTo(m_hist,CV_8UC1,  (255 / (histd.getMaxValue()-histd.getMinValue()) ) );
        // this is great. It converts your grayscale image into a tone-mapped one,
        // much more pleasing for the eye
        // function is found in contrib module, so include contrib.hpp
        // and link accordingly
//        cv::Mat falseColorsMap;
//        applyColorMap(adjMap, falseColorsMap, COLORMAP_AUTUMN);
//        m_hist = adjMap;
    }
}

//--------------------------------------------------------------
void testApp::draw(){
	RectTracker& tracker = contourFinder.getTracker();
    
    int width = 400, height = 300;
    ofSetColor(255, 255, 255);
    threshImage.draw(0, 0, width, height);
    depthImage.draw(400, 0, width, height);
    kinect.draw(800, 0, width, height);
    segImage.draw(0, 300, width, height);
    distImage.draw(400, 300, width, height); 
    drawMat(m_hist, 800, 300, width, height);
    
    
    float scalex = (float)width/640;
    float scaley = (float)height/480;
    
    for(int i = 0; i < contourFinder.size(); i++) {
        ofPoint center = toOf(contourFinder.getCenter(i));
        
        int label = contourFinder.getLabel(i);
        
        if(tracker.existsPrevious(label)) {
            int state;
            
            try {
                state = object_states.at(label);
                
                switch (state) {
                    case 0: //Entry enter position
                        if (center.y < entryStayBoundary && center.y > entryLeaveBoundary){
                            object_states.at(label) = 1;
                        } 
                        break;
                        
                    case 1: //Entry Stay position
                        if (center.y < entryLeaveBoundary){
                            object_states.at(label) = 2;
                            object_hexes[label] = 0x84CF34;
                        }
                        else{
//                            ofLogNotice() << label << " deciding entry";
                            object_hexes[label] = 0xCF8434;
                        }
                        break;
                        
                    case 2: //Entry Exit Position
                        //Elvis has entered the building
                        object_states.erase(label);
                        entryCount ++;
                        ofLogNotice() << "Number of entries "<< entryCount;
                        break;
                        
                    case 3: //Exit enter position
                        if (center.y > exitStayBoundary && center.y < exitLeaveBoundary){
                            object_states.at(label) = 4;
                        }
                        break;
                        
                    case 4: //Exit stay position
                        if (center.y > exitLeaveBoundary){
                            object_states.at(label) = 5;
                            object_hexes[label] = 0xC93030;
                        }
                        else{ 
//                            ofLogNotice() << label <<" deciding exit";
                              object_hexes[label] = 0x347FCF;
                        }
                        break;
                        
                    case 5: //Exit leave position
                        //Elvis has left the building
                        object_states.erase(label);
                        exitCount ++;
                        ofLogNotice() << "Number of exits " << exitCount;
                        break;
                        
                    default:
                        object_states.at(label) = -1;
                        break;
                }
            } catch (const std::out_of_range& oor) {
                if (center.y > entryStayBoundary){
                    state = object_states[label] = 0; //Entry state
                    ofLogNotice() << "Initial Entry for object " << label; 
                }
                else{
                    state = object_states[label] = 3; //Exit state
                    ofLogNotice() << "Initial Exit for object " << label; 
                }
                try {
                    object_hexes.at(label);
                } catch (const std::out_of_range& oor) {
                    object_hexes[label] = 0x3054C9;
                }
                ofLogNotice() << "Color " << object_hexes[label];
            }
             
        }
        ofPushMatrix();
        ofTranslate(400, 0);
        ofScale( scalex, scaley, 0.0);
        
        ofFill();
        ofSetHexColor(object_hexes[label]);
        
        ofPolyline line = contourFinder.getPolyline(i);
        line.simplify();
        
        ofBeginShape();
        for( int i = 0; i < line.getVertices().size(); i++) {
            ofVertex(line.getVertices().at(i).x, line.getVertices().at(i).y);
        }
        ofEndShape();
        ofPopMatrix();
        
        ofPushMatrix();
        ofScale( scalex, scaley, 0.0);
        ofNoFill(); 
        ofRect(toOf(contourFinder.getBoundingRect(i)));
        ofPopMatrix();
        
        ofPushMatrix(); 
        ofSetColor(255, 0, 0);
        
        ofScale( scalex, scaley, 0.0);
        ofTranslate(center.x, center.y);
        
        
        string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
        ofDrawBitmapString(msg, 0, 0);
        ofVec2f velocity = toOf(contourFinder.getVelocity(i));
        ofScale(5, 5);
        ofLine(0, 0, velocity.x, velocity.y);
        ofPopMatrix();
        
    }

    
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    switch (key) {
		case '>':
		case '.':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '<':
		case ',':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case '+':
		case '=':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '-':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
//		case 'o':
//			kinect.setCameraTiltAngle(angle); // go back to prev tilt
//			kinect.open();
//			break;
			
//		case 'c':
//			kinect.setCameraTiltAngle(0); // zero the tilt
//			kinect.close();
//			break;
			
		 
			
//		case OF_KEY_UP:
//			angle++;
//			if(angle>30) angle=30;
//			kinect.setCameraTiltAngle(angle);
//			break;
//			
//		case OF_KEY_DOWN:
//			angle--;
//			if(angle<-30) angle=-30;
//			kinect.setCameraTiltAngle(angle);
//			break;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
