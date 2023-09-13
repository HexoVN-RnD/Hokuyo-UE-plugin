#pragma once

#include "CoreMinimal.h"
#include "HokuyoRead.h"

struct DBPoint
{
	float X;
	float Y;

	bool operator==(const DBPoint& Other) const
	{
		return X == Other.X && Y == Other.Y;
	}
};

class DBScan
{
public:
	static TArray<DBPoint> RegionQuery(const TArray<DBPoint>& Points, const DBPoint& P, float Eps);
	static void ExpandCluster(TArray<DBPoint>& Points, int32 PIdx, TArray<int32>& Cluster, int32 ClusterId, float Eps, int32 MinPts);
	static TArray<TArray<DBPoint>> PerformDBScan(TArray<DBPoint>& Points, float Eps, int32 MinPts);
	static TArray<FVector2D> GetCentroids(TArray<TArray<DBPoint>> Clusters);
};
