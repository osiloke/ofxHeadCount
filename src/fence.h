//
//  fence.h
//  headCount
//
//  Created by Osiloke Emoekpere on 1/31/14.
//
//

#ifndef headCount_fence_h
#define headCount_fence_h


/* Exit Boundary
 * ========================
 *
 * ------------------------
 * Stay Boundary + Threshold
 * ------------------------
 *
 * Entry Boundary
 * ========================
 */

class Fence{
private:
    int exitBoundary;
    int exitThreshold;
    
    int entryBoundary;
    int entryThreshold;
    
    int stayBoundary;
    int stayThreshold;
    
};

#endif
