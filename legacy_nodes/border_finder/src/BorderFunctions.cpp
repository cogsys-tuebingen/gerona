/**************************************************************************
    @project RA Outdoor Robot System
    @author andreas masselli
    @date 9/26/2010 early 21st century
    (c) Universitaet Tuebingen 2010

**************************************************************************/
#include "BorderFunctions.h"

#include "BorderFinder.h"

int gFloddedCellCounter = 0;

void SetFloddedCellCounter(int val) {
    gFloddedCellCounter = val;
}

bool mVerbose = false;

void SetVerbose(int val) {
    mVerbose = val;
}


// returns the total user time of the process
static double totalUserTime() {
    struct timeval ti;
    double t;

    gettimeofday(&ti, NULL);
    t = (double)ti.tv_usec / 1000;
    t += (double)ti.tv_sec * 100;
    return t;
}

// Melkman convex hull
// Copyright 2002, softSurfer (www.softsurfer.com)
// This code may be freely used and modified for any purpose
// providing that this copyright notice is included with it.
// SoftSurfer makes no warranty for this code, and cannot be held
// liable for any real or imagined damage resulting from its use.
// Users of this code must verify correctness for their application.

// Assume that a class is already given for the object:
//    Point with coordinates {float x, y;}
//===================================================================

/**
  * isRight(): Test if a point is left or on or right of an infinite line.
  * Input:    Three points A, B and P
  * Return:   > 0 for P right of the line through A and B
  *           = 0 for P on the line
  *           < 0 for P left of the line
  */
inline int isRight(iPoint A, iPoint B, iPoint P) {
    return (P.x - A.x)*(B.y - A.y) - (B.x - A.x)*(P.y - A.y);
}

float signedDistanceToVector(iPoint A, iPoint B, iPoint P) {
   iPoint normalAB;
   normalAB.x =   B.y - A.y;
   normalAB.y = -(B.x - A.x);

   float length = sqrt(normalAB.x*normalAB.x + normalAB.y*normalAB.y);
   length += 0.01; // avoid div by zero

   return ((float)((P.x - A.x)*normalAB.x + (P.y - A.y)*normalAB.y)) / length;
}

/**
  * convexHull2D(): Melkman convex hull algorithm
  * Input:  V[] = polyline array of 2D vertex points
  *         n   = the number of points in V[]
  * Throughput:  D[] = deque: array of 2D vertex points. length must be at least 2*n + 1
  *              deque space is used to store convex hull
  * Output: H* = pointer to start of convex hull array of vertices.
  *              deque space is used to store convex hull. max length is n
  * Return: h   = the number of points in H[]
  */
int convexHull2D(iPoint* V, int n, iPoint* D, iPoint** H) {
    // initialize a deque D[] from bottom to top so that the
    // first three vertices of V[] are a counterclockwise triangle
    int bottom = n - 2;
    int top = bottom + 3;   // initial bottom and top deque indices
    D[bottom] = D[top] = V[2];       // 3rd vertex is at both bottom and top
    if (isRight(V[0], V[1], V[2]) > 0) {
        D[bottom+1] = V[0];
        D[bottom+2] = V[1];          // ccw vertices are: 2,0,1,2
    }
    else {
        D[bottom+1] = V[1];
        D[bottom+2] = V[0];          // ccw vertices are: 2,1,0,2
    }

    // compute the hull on the deque D[]
    for (int i = 3; i < n; ++i) {   // process the rest of vertices

/*        for (int j = 0; j < (top-bottom); ++j)
            cout << D[bottom + j].x << "," << D[bottom + j].y << " ";
        cout << "\n Adding (" << V[i].x << "," << V[i].y << ")" << endl;
*/
        // test if next vertex is inside the deque hull
        if ( (isRight(D[bottom], D[bottom+1], V[i]) > 0)
          && (isRight(D[top-1], D[top], V[i]) > 0) )
            continue;         // skip an interior vertex

        // incrementally add an exterior vertex to the deque hull
        // get the rightmost tangent at the deque bot
        while ( (isRight(D[bottom], D[bottom+1], V[i]) <= 0)
             && (top-bottom > 3) )  // always keep an initial set of 3 points
            ++bottom;                // remove bottom of deque
        D[--bottom] = V[i];          // insert V[i] at bottom of deque

        // get the leftmost tangent at the deque top
        while ( (isRight(D[top-1], D[top], V[i]) <= 0)
             && (top-bottom > 3) ) // always keep an initial set of 3 points
            --top;                // pop top of deque
        D[++top] = V[i];          // push V[i] onto top of deque
    }

    *H = D + bottom+1;

    return top-bottom;
}

// end Melkman convex hull


// works only on closed paths (start = end)
iPoint findLargestFrontier(iPoint* contour, int contourLength, bool* occupied, float* frontierRatio) {
    // for frontierRatio
    int unknownCounter = 0;
    int currentLength = 0; // length of current frontier
    int maxLength = 0;
    int maxIndex = 0;

    for (int i = 0; i < contourLength; ++i) {

        if (occupied[i]) {
            // update maximum
            if (currentLength > maxLength) {
                maxLength = currentLength;
                maxIndex = i;
            }

            currentLength = 0;
        }
        else { // unknown
            ++currentLength;

            // for frontierRatio
            ++unknownCounter;
        }
    }

//    OUT("MAXL: " << maxLength);
    maxIndex -= maxLength;

    // get frontierRatio
    *frontierRatio = ((float)unknownCounter) / contourLength;

    return contour[maxIndex];
}

// works only on closed paths (start = end)
void classifyContour(iPoint* contour, int contourLength,
                     iPoint* convexHull, int convexHullLength,
                     bool* occupied, float* border) {
    if (convexHullLength < 2) {
       for (int i = 0; i < convexHullLength; ++i)
                border[i] = 0;
        return;
    }

/*    for (int i = 0; i < contourLength; ++i)
        cout << "(" << contour[i].x << "," << contour[i].y << ")";
    cout << endl;
    for (int i = 0; i < convexHullLength; ++i)
        cout << "(" << convexHull[i].x << "," << convexHull[i].y << ")";
    cout << endl;
*/

//    int unexploredCounter = 0;
//    int closeToCVAmount = 0;

    int borderCounter = 0;
    int allCounter = 0;

    int borderLength = 0;

    iPoint* A = convexHull + convexHullLength-2;
    iPoint* B = convexHull;

    for (int i = 0; i < contourLength; ++i) {
        if ( (contour[i].x == B->x)
          && (contour[i].y == B->y) ) { // if the point lies on the convex hull
            int maxPoints = MAX(abs(A->x - B->x), abs(A->y - B->y));
            *border = ((float)borderCounter) / maxPoints;//allCounter;
            ++border;

            borderLength += borderCounter;
            borderCounter = 0;
            allCounter = 0;

            // choose next convex hull line
            A = B;
            ++B;
            if (B == convexHull+convexHullLength) { // this should not happen
                ROS_INFO("classifyContour blasted array.");
                --A;
                --B;
            }
          }

        // measure distance
        float distance = signedDistanceToVector(*A, *B, contour[i]);
        if (distance < 0)
            ROS_INFO_STREAM("Negative distance: " << distance);

        bool closeToConvexHull = (distance < 3);

        // examine cell type
        if ( occupied[i] && closeToConvexHull )
            ++borderCounter;

        ++allCounter;

//           if (distance < 1)
//               ++closeToCVAmount;
    }
//    OUT(closeToCVAmount);
}


// 4 neighbor flood fill algorithm
// returns true if something has been filled, false otherwise
// min and max represent the bounding box
bool floodfill(uchar* image, int x, int y, iPoint* max, iPoint* min, int width, int height) {
    // check image boundaries
    if((x < 0) || (x >= width)) return false;
    if((y < 0) || (y >= height)) return false;

    // get current pixel color
    uchar* pixel = image + y*width + x;

    // cancel if cell has not the desired color
    if(*pixel != CELL_EMPTY) return false;

    // otherwise proceed
    // fill cell
    *pixel = CELL_FILLED;
    ++gFloddedCellCounter;

    // update bounding box
    if(x < min->x)      min->x = x;
    else if(x > max->x) max->x = x;
    if(y < min->y)      min->y = y;
    else if(y > max->y) max->y = y;

    // recursively call floodfill on all 4 neighbors
    floodfill(image, x, y-1, max, min, width, height);
    floodfill(image, x, y+1, max, min, width, height);
    floodfill(image, x-1, y, max, min, width, height);
    floodfill(image, x+1, y, max, min, width, height);
    return true;
}

#define TIMEDISPLAY 1


// Contour following for region
int contourFollow(uchar* image, iPoint* contour,
                  int left, int top, int right, int bottom,
                  int width, int height, bool* occupied) {
    // check if there is a region
    if (left >= right) return -1;
    if (top >= bottom) return -1;

    // clip bounds
    if (left < 0) left = 0;
    if (top  < 0) top  = 0;
    if (right > width-1) right = width-1;
    if (bottom > height-1) bottom = height-1;

    bool found = false;

#ifdef TIMEDISPLAY
    double start, end;
    start = totalUserTime();
#endif

    int contourLength = 0;

    // find start of countour // todo: hangs if no region is present
    int j = top;
    int i = left-1;

    while(!found) {
        ++i;
        if(i > right) {
            i = left;
            ++j;
        }
        if(image[i + j*width] == CELL_FILLED) { // if desired color
            found = true;
            // go back one pixel
            --i;
            if (i < left) {
                i = right;
                --j;
            }
        }
    }

#ifdef DEBUG
    printf("BorderFinder: i=%d j=%d bottomRight.x=%d bottomRight.y=%d\n", i, j, right, bottom);
#endif

    // add first point to contour
    contour[contourLength].x = i;
    contour[contourLength].y = j;
    ++contourLength;

    *occupied = (image[i + j*width] == CELL_OCCUPIED);
    ++occupied;
    image[i + j*width] = CELL_CONTOUR; // mark cell for debugging

    // next task: follow the contour
    found = false;

    static const int nx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int ny[8] = {0, -1, -1, -1, 0, 1, 1, 1};
    int direction = 5;

    while (contourLength < MAXCONTOURLENGTH) {
        // advance direction
        if(direction % 2 == 0)
            direction += 1; // +1
        else
            direction += 2; // +2
        direction %= 8;
        //printf("dir %d ", dir);

        int turnaround = 0;
        bool nextPoint = false;
        iPoint hyp; // hypothesis point

        while(!nextPoint) {
            hyp.x = contour[contourLength-1].x + nx[direction];
            hyp.y = contour[contourLength-1].y + ny[direction];

            if( (hyp.x > 0) && (hyp.x < width)
             && (hyp.y > 0) && (hyp.y < height)
             && (image[hyp.x + hyp.y*width] != CELL_FILLED) ) {
                nextPoint = true;
            }
            else {
                direction += 7; // -1
                direction %= 8;
                ++turnaround;

                if (turnaround >= 8) { // if we searched all directions without finding a next point
                    ROS_INFO("turnaround >= 8\n");
                    return -1;
                }
            }
        }

        //printf("dir %d i %d j%d\n", dir,i,j);dir = (dir+1)%8;}

        // add next point to contour
        contour[contourLength] = hyp;
        ++contourLength;

        *occupied = (image[hyp.x + hyp.y*width] == CELL_OCCUPIED);
        ++occupied;
        image[hyp.x + hyp.y*width] = CELL_CONTOUR; // mark cell for debugging

         // check if we reached the starting point again and our job is done
        if ( (contourLength > 2)
          && (contour[contourLength-1].x == contour[1].x)
          && (contour[contourLength-1].y == contour[1].y)
          && (contour[contourLength-2].x == contour[0].x)
          && (contour[contourLength-2].y == contour[0].y) ) {

#ifdef TIMEDISPLAY
            end = totalUserTime();

#endif

            return contourLength-2;
        }
    }

    ROS_INFO("WARNING: Max contour length reached!");
    return -1;
}
