#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>
#include "dbscan.h"

DBSCAN::DBSCAN(int n, double eps, int minPts, vector<Point> points) {
	this->n = n;
	this->eps = eps;
	this->minPts = minPts;
	this->points = points;
	this->size = (int)points.size();
	adjPoints.resize(size);
	this->clusterIdx=-1;
}

void DBSCAN::run () {
	checkNearPoints();

	for(int i=0;i<size;i++) {
		if(points[i].cluster != NOT_CLASSIFIED) continue;

		if(isCoreObject(i)) {
			dfs(i, ++clusterIdx);
		} else {
			points[i].cluster = NOISE;
		}
	}

	cluster.resize(clusterIdx+1);
	for(int i=0;i<size;i++) {
		if(points[i].cluster != NOISE) {
			cluster[points[i].cluster].push_back(i);
		}
	}
}

void DBSCAN::dfs (int now, int c) {
	points[now].cluster = c;
	if(!isCoreObject(now)) return;

	for(auto&next:adjPoints[now]) {
		if(points[next].cluster != NOT_CLASSIFIED) continue;
		dfs(next, c);
	}
}

void DBSCAN::checkNearPoints() {
	for(int i=0;i<size;i++) {
		for(int j=0;j<size;j++) {
			if(i==j) continue;
			if(points[i].getDis(points[j]) <= eps) {
				points[i].ptsCnt++;
				adjPoints[i].push_back(j);
			}
		}
	}
}

    // is idx'th point core object?
bool DBSCAN::isCoreObject(int idx) {
	return points[idx].ptsCnt >= minPts;
}

vector<vector<int> > DBSCAN::getCluster() {
	return cluster;
}


double Point::getDis(const Point & ot) {
	return sqrt((x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y));
}

vector<Point> InputReader::getPoints() {
	return points;
}

//int main(int argc, const char * argv[]) {
//    if(argc!=5) {
//        cout << "Please follow this format. clustering.exe [intput] [n] [eps] [minPts]";
//        return 0;
//    }
//
//    string inputFileName(argv[1]);
//    string n(argv[2]);
//    float eps = 0.05; // vizinhaça
//    int minPts = 4; // minimo de pontos para formar uma região densa
//
//    InputReader inputReader(inputFileName);
//
//    DBSCAN dbScan(stoi(n),eps, minPts, inputReader.getPoints());
//    dbScan.run();
//
//    OutputPrinter outputPrinter(stoi(n), inputFileName, dbScan.getCluster());
//    outputPrinter.print();
//
//    return 0;
//}
