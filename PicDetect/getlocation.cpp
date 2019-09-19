#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <math.h>
#include <string.h>

using namespace cv;
using namespace std;

static vector<Point2f> getloc(vector<Point2f> Pi) {
	vector<Point2f> finalloc(4);
	int len = 0;
	int index = 0;
	/*for (int i = 0; i < Pi.size(); i++) {
	for (int j = i + 1; j < Pi.size; j++) {
	if (fabs(Pi[i].x - Pi[i].x) < 1){}
	index = j;
	break;
	}
	}*/
	for (int i = 0; i < 4; i++) {
		finalloc[i] = Point2d(Pi[i + 1].x, Pi[i + 1].y);
	}

	return finalloc;
}