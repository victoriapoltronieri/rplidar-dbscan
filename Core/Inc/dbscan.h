#ifndef CARRO_H
#define CARRO_H
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>

using namespace std;

const int NOISE = -2;
const int NOT_CLASSIFIED = -1;
const int angle_min = 155;
const int angle_max = 255;

class Point {
public:
    double x, y;
    int ptsCnt, cluster;
    double getDis(const Point & ot);
};

class DBSCAN {
public:
    int n, minPts;
    double eps;
    vector<Point> points;
    int size;
    vector<vector<int> > adjPoints;
    vector<bool> visited;
    vector<vector<int> > cluster;
    int clusterIdx;

    DBSCAN(int n, double eps, int minPts, vector<Point> points);

    void run ();

    void dfs (int now, int c);

    void checkNearPoints();
    // is idx'th point core object?
    bool isCoreObject(int idx);

    vector<vector<int> > getCluster();
};

class InputReader {
private:
    vector<Point> points;
public:
    vector<Point> getPoints();
};

//class OutputPrinter {
//private:
//    ofstream fout;
//    vector<vector<int> > cluster;
//    string filename;
//    int n;
//public:
//    OutputPrinter(int n, string filename, vector<vector<int> > cluster) {
//        this->n = n;
//        this->cluster = cluster;
//
//        // remove ".txt" from filename
//        if(filename.size()<4){
//            cout << filename << "input file name's format is wrong\n";
//            exit(0);
//        }
//        for(int i=0;i<4;i++) filename.pop_back();
//        this->filename = filename;
//
//        // sort by size decending order
//        sort(cluster.begin(), cluster.end(), [&](const vector<int> i, const vector<int> j) {
//            return (int)i.size() > (int)j.size();
//        });
//    }
//    void print() {
//        for(int i=0;i<n;i++) {
//            fout.open(filename+"_cluster_"+to_string(i)+".txt");
//
//            for(int j=0;j<cluster[i].size();j++) {
//                fout << cluster[i][j] << endl;
//            }
//
//            fout.close();
//        }
//    }
//};

#endif
