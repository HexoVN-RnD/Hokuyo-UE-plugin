// Fill out your copyright notice in the Description page of Project Settings.

#include "KmeansClustering.h"

TArray<double> lineToVec(FString& line)
{
    TArray<double> values;
    FString tmp = "";

    for (int i = 0; i < line.Len(); i++)
    {
        if ((48 <= int(line[i]) && int(line[i]) <= 57) || line[i] == '.' || line[i] == '+' || line[i] == '-' || line[i] == 'e')
        {
            tmp += line[i];
        }
        else if (tmp.Len() > 0)
        {

            values.Add(FCString::Atof(*tmp));
            tmp = "";
        }
    }
    if (tmp.Len() > 0)
    {
        values.Add(FCString::Atof(*tmp));
        tmp = "";
    }

    return values;
}

Point::Point(int id, FString line)
{
    pointId = id;
    values = lineToVec(line);
    dimensions = values.Num();
    clusterId = 0; // Initially not assigned to any cluster
}

int Point::getDimensions() { return dimensions; }

int Point::getCluster() { return clusterId; }

int Point::getID() { return pointId; }

void Point::setCluster(int val) { clusterId = val; }

double Point::getVal(int pos) { return values[pos]; }

Cluster::Cluster(int clusterId, Point centroid)
{
    this->clusterId = clusterId;
    for (int i = 0; i < centroid.getDimensions(); i++)
    {
        this->centroid.Add(centroid.getVal(i));
    }
    this->addPoint(centroid);
}

void Cluster::addPoint(Point p)
{
    p.setCluster(this->clusterId);
    points.Add(p);
}

bool Cluster::removePoint(int pointId)
{
    int size = points.Num();

    for (int i = 0; i < size; i++)
    {
        if (points[i].getID() == pointId)
        {
            points.RemoveAt(i);
            return true;
        }
    }
    return false;
}

void Cluster::removeAllPoints() { points.Empty(); }

int Cluster::getId() { return clusterId; }

Point Cluster::getPoint(int pos) { return points[pos]; }

int Cluster::getSize() { return points.Num(); }

double Cluster::getCentroidByPos(int pos) const { return centroid[pos]; }

void Cluster::setCentroidByPos(int pos, double val) { this->centroid[pos] = val; }

void KMeans::clearClusters()
{
    for (int i = 0; i < K; i++)
    {
        clusters[i].removeAllPoints();
    }
}

int KMeans::getNearestClusterId(Point point)
{
    double sum = 0.0, min_dist;
    int NearestClusterId;
    if (dimensions == 1) {
        min_dist = std::abs(clusters[0].getCentroidByPos(0) - point.getVal(0));
    }
    else
    {
        for (int i = 0; i < dimensions; i++)
        {
            sum += std::pow(clusters[0].getCentroidByPos(i) - point.getVal(i), 2.0);
            // sum += abs(clusters[0].getCentroidByPos(i) - point.getVal(i));
        }
        min_dist = std::sqrt(sum);
    }
    NearestClusterId = clusters[0].getId();

    for (int i = 1; i < K; i++)
    {
        double dist;
        sum = 0.0;

        if (dimensions == 1) {
            dist = std::abs(clusters[i].getCentroidByPos(0) - point.getVal(0));
        }
        else {
            for (int j = 0; j < dimensions; j++)
            {
                sum += std::pow(clusters[i].getCentroidByPos(j) - point.getVal(j), 2.0);
                // sum += abs(clusters[i].getCentroidByPos(j) - point.getVal(j));
            }

            dist = std::sqrt(sum);
            // dist = sum;
        }
        if (dist < min_dist)
        {
            min_dist = dist;
            NearestClusterId = clusters[i].getId();
        }
    }

    return NearestClusterId;
}

KMeans::KMeans(int K, int iterations)
{
    this->K = K;
    this->iters = iterations;
}

void KMeans::run(TArray<Point>& all_points)
{
    total_points = all_points.Num();
    dimensions = all_points[0].getDimensions();

    // Initializing Clusters
    TArray<int> used_pointIds;
    for (int i = 1; i <= K; i++)
    {
        while (true)
        {
            int index = rand() % total_points;

            if (!used_pointIds.Contains(index))
            {
                used_pointIds.Add(index);
                all_points[index].setCluster(i);
                Cluster cluster(i, all_points[index]);
                clusters.Add(cluster);
                break;
            }
        }
    }
    /*cout << "Clusters initialized = " << clusters.size() << endl
        << endl;

    cout << "Running K-Means Clustering.." << endl;*/

    int iter = 1;
    while (true)
    {
        /*cout << "Iter - " << iter << "/" << iters << endl;*/
        bool done = true;

        // Add all points to their nearest cluster
#pragma omp parallel for reduction(&&: done) num_threads(16)
        for (int i = 0; i < total_points; i++)
        {
            int currentClusterId = all_points[i].getCluster();
            int nearestClusterId = getNearestClusterId(all_points[i]);

            if (currentClusterId != nearestClusterId)
            {
                all_points[i].setCluster(nearestClusterId);
                done = false;
            }
        }

        // clear all existing clusters
        clearClusters();

        // reassign points to their new clusters
        for (int i = 0; i < total_points; i++)
        {
            // cluster index is ID-1
            clusters[all_points[i].getCluster() - 1].addPoint(all_points[i]);
        }

        // Recalculating the center of each cluster
        for (int i = 0; i < K; i++)
        {
            int ClusterSize = clusters[i].getSize();

            for (int j = 0; j < dimensions; j++)
            {
                double sum = 0.0;
                if (ClusterSize > 0)
                {
#pragma omp parallel for reduction(+: sum) num_threads(16)
                    for (int p = 0; p < ClusterSize; p++)
                    {
                        sum += clusters[i].getPoint(p).getVal(j);
                    }
                    clusters[i].setCentroidByPos(j, sum / ClusterSize);
                }
            }
        }

        if (done || iter >= iters)
        {
            /*cout << "Clustering completed in iteration : " << iter << endl
                << endl;*/
            break;
        }
        iter++;
    }
}
// Add a new method to get a reference to the clusters vector
const TArray<Cluster>& KMeans::getClusters() const
{
    return clusters;
}
