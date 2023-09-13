
#include "DBSCANClustering.h"

TArray<DBPoint> DBScan::RegionQuery(const TArray<DBPoint>& Points, const DBPoint& P, float Eps)
{
    TArray<DBPoint> Neighbors;
    float EpsSquared = FMath::Pow(Eps, 2); // Calculate Eps^2 once outside the loop
    for (const DBPoint& Q : Points)
    {
        float DistSquared = FMath::Pow(P.X - Q.X, 2) + FMath::Pow(P.Y - Q.Y, 2); // Calculate squared distance
        if (DistSquared <= EpsSquared) // Compare squared distances
        {
            Neighbors.Add(Q);
        }
    }
    return Neighbors;
}

void DBScan::ExpandCluster(TArray<DBPoint>& Points, int32 PIdx, TArray<int32>& Cluster, int32 ClusterId, float Eps, int32 MinPts)
{
    TArray<DBPoint> Seeds = RegionQuery(Points, Points[PIdx], Eps);
    if (Seeds.Num() < MinPts)
    {
        Cluster[PIdx] = -1;
    }
    else
    {
        Cluster[PIdx] = ClusterId;
        for (int32 i = 0; i < Seeds.Num(); i++)
        {
            int32 QIdx = Points.IndexOfByKey(Seeds[i]);
            if (Cluster[QIdx] == 0)
            {
                Cluster[QIdx] = ClusterId;
                ExpandCluster(Points, QIdx, Cluster, ClusterId, Eps, MinPts);
            }
        }
    }
}

TArray<TArray<DBPoint>> DBScan::PerformDBScan(TArray<DBPoint>& Points, float Eps, int32 MinPts)
{
    int32 N = Points.Num();
    TArray<int32> Cluster;
    Cluster.Init(0, N);
    int32 ClusterId = 0;
    for (int32 i = 0; i < N; i++)
    {
        if (Cluster[i] == 0)
        {
            TArray<DBPoint> Neighbors = RegionQuery(Points, Points[i], Eps);
            if (Neighbors.Num() < MinPts)
            {
                Cluster[i] = -1;
            }
            else
            {
                ClusterId++;
                ExpandCluster(Points, i, Cluster, ClusterId, Eps, MinPts);
            }
        }
    }

    TArray<TArray<DBPoint>> Clusters;
    Clusters.Init(TArray<DBPoint>(), ClusterId);
    for (int32 i = 0; i < N; i++)
    {
        if (Cluster[i] > 0)
        {
            Clusters[Cluster[i] - 1].Add(Points[i]);
        }
    }
    return Clusters;
}

TArray<FVector2D> DBScan::GetCentroids(TArray<TArray<DBPoint>> Clusters)
{
    TArray<FVector2D> Centroids;
    for (auto& Cluster : Clusters)
    {
        if (Cluster.Num() <= 1)
        {
			continue;
		}
        float SumX = 0.f;
        float SumY = 0.f;
        for (auto& Point : Cluster)
        {
            SumX += Point.X;
            SumY += Point.Y;
        }
        FVector2D Centroid{ SumX / static_cast<float>(Cluster.Num()), SumY / static_cast<float>(Cluster.Num()) };
        Centroids.Add(Centroid);
    }
    return Centroids;
}
