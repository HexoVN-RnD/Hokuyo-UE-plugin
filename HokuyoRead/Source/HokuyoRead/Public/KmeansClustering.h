// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include <cmath>
#include "HokuyoRead.h"

class Point
{
private:
    int pointId, clusterId;
    int dimensions;
    TArray<double> values;

public:
    Point(int id, FString line);

    int getDimensions();

    int getCluster();

    int getID();

    void setCluster(int val);

    double getVal(int pos);
};

class Cluster
{
private:
    int clusterId;
    TArray<double> centroid;
    TArray<Point> points;

public:
    Cluster(int clusterId, Point centroid);

    void addPoint(Point p);

    bool removePoint(int pointId);

    void removeAllPoints();

    int getId();

    Point getPoint(int pos);

    int getSize();

    double getCentroidByPos(int pos) const;

    void setCentroidByPos(int pos, double val);
};

class KMeans
{
private:
    int K, iters, dimensions, total_points;
    TArray<Cluster> clusters;

    void clearClusters();
    int getNearestClusterId(Point point);

public:
    KMeans(int K, int iterations);

    void run(TArray<Point>& all_points);

    const TArray<Cluster>& getClusters() const;
};
