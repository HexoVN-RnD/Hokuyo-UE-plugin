// Copyright Epic Games, Inc. All Rights Reserved.

#include "HokuyoReadBPLibrary.h"
#include "HokuyoRead.h"
#include "Async/AsyncWork.h"

// Define the global kmeans_centroids variable
TArray<FVector2D> kmeans_centroids;
TArray<FVector2D> db_centroids;

void UHokuyoReadBPLibrary::closeConnectionifFalse(bool Result, UPARAM(ref) FUrgWrapper& UrgWrapper)
{
	if (!Result) {
		closeConnection(UrgWrapper);
	}
}

bool UHokuyoReadBPLibrary::openConnection(UPARAM(ref) FUrgWrapper& UrgWrapper, int32 connectionType, FString ipAddress, int32 portNumber)
{
	urg_t& urg = UrgWrapper.Urg;
	int ret;
	// Convert the connectionType parameter to an urg_connection_type_t value
	urg_connection_type_t urgConnectionType = static_cast<urg_connection_type_t>(connectionType);
	// Convert the ipAddress parameter to a const char* value
	const char* urgIpAddress = TCHAR_TO_UTF8(*ipAddress);
	// Open the connection with the device
	ret = urg_open(&urg, urgConnectionType, urgIpAddress, portNumber);
	if (ret < 0) {
		return false;
	}
	return true;
}

bool UHokuyoReadBPLibrary::setScanningDegree(UPARAM(ref) FUrgWrapper& UrgWrapper, int LeftDegree, int RightDegree)
{
	urg_t& urg = UrgWrapper.Urg;

	// \~english Case where the measurement range (start/end steps) is defined
	urg_set_scanning_parameter(&urg,
		urg_deg2step(&urg, -LeftDegree),
		urg_deg2step(&urg, +RightDegree), 0);

	return true;
}

bool UHokuyoReadBPLibrary::closeConnection(UPARAM(ref) FUrgWrapper& UrgWrapper)
{
	urg_t& urg = UrgWrapper.Urg;
	// Close the connection
	urg_close(&urg);

	return true;
}

void bubbleSort(TArray<FVector2D>& centroids) {
	for (int i = 0; i < centroids.Num() - 1; i++) {
		for (int j = 0; j < centroids.Num() - i - 1; j++) {
			if (centroids[j].X > centroids[j + 1].X) {
				Swap(centroids[j], centroids[j + 1]);
			}
		}
	}
}

bool UHokuyoReadBPLibrary::getDistance(UPARAM(ref) FUrgWrapper& UrgWrapper, int CaptureTimes, bool enableKMeans, int k, bool enableDBSCAN, float eps, int min_pts)
{
	urg_t& urg = UrgWrapper.Urg;
	int ret;
	long* data;
	int data_max;
	long time_stamp;
	int i;
	TArray<Point> kmeans_all_points;
	TArray<DBPoint> db_all_points;

	// Get length datas
	data_max = urg_max_data_size(&urg);
	data = (long*)malloc(sizeof(long) * data_max);
	if (data == nullptr) {
		urg_close(&urg);
		return false;
	}

	urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0, 1);
	// Get datas
	for (i = 0; CaptureTimes == 0 || i < CaptureTimes; ++i) {
		kmeans_all_points.Empty();
		db_all_points.Empty();
		ret = urg_get_distance(&urg, data, &time_stamp);
		if (ret <= 0) {
			free(data);
			urg_close(&urg);
			return false;
		}
		(void)time_stamp;

		int j;
		long min_distance;
		long max_distance;

		int point_id = 0;
		urg_distance_min_max(&urg, &min_distance, &max_distance);
		for (j = 0; j < ret; ++j) {
			long l = data[j];
			double radian;

			if ((l <= min_distance) || (l >= max_distance)) {
				continue;
			}
			radian = urg_index2rad(&urg, j);
			int32 x = (int32)(l * sin(radian)); 
			int32 y = (int32)(l * cos(radian));

			// Print x and y values in real-time
			/*FString xString = FString::FromInt(x);
			FString yString = FString::FromInt(y);
			FString xyString = "x: " + xString + ", y: " + yString;
			if (GEngine)
				GEngine->AddOnScreenDebugMessage(-1, 0.025f, FColor::Red, xyString);*/

			if (enableDBSCAN)
			{
				// Add points to db_points TArray
				DBPoint dbpoint = { x, y };
				db_all_points.Add(dbpoint);
			}
			
			if (enableKMeans)
			{
				// Add points to the kmeans_all_points TArray.
				FString coordinates = FString::FromInt(x) + " " + FString::FromInt(y);
				Point point(point_id, coordinates);
				kmeans_all_points.Add(point);
			}

			// Increment point number
			point_id++;
		}

		if (enableDBSCAN)
		{
			// Run the DBSCAN algorithm on your data
			TArray<TArray<DBPoint>> db_clusters = DBScan::PerformDBScan(db_all_points, eps, min_pts);
			// Create a local copy of the centroids for printing
			db_centroids = DBScan::GetCentroids(db_clusters);

			bubbleSort(db_centroids);

			// Output all points coordinates and cluster labels
			/*GEngine->AddOnScreenDebugMessage(-1, 0.1f, FColor::Yellow, TEXT("All points:"));
			for (int32 z = 0; z < db_all_points.Num(); z++)
			{
				FString ClusterLabel;
				for (int32 x = 0; x < db_clusters.Num(); x++)
				{
					if (db_clusters[x].Contains(db_all_points[i]))
					{
						ClusterLabel = FString::Printf(TEXT("%d"), x + 1);
						break;
					}
				}
				GEngine->AddOnScreenDebugMessage(-1, 0.1f, FColor::Yellow, FString::Printf(TEXT("(%f, %f) - cluster %s"), db_all_points[i].X, db_all_points[i].Y, *ClusterLabel));
			}*/

			/*GEngine->AddOnScreenDebugMessage(-1, 0.02f, FColor::Yellow, FString::Printf(TEXT("Centroids: %d"), db_centroids.Num()));
			for (auto& Centroid : db_centroids)
			{
				GEngine->AddOnScreenDebugMessage(-1, 0.02f, FColor::Yellow, FString::Printf(TEXT("(%f, %f)"), Centroid.X, Centroid.Y));
			}

			UE_LOG(LogTemp, Warning, TEXT("Centroids: %d"), db_centroids.Num());
			for (auto& Centroid : db_centroids)
			{
				UE_LOG(LogTemp, Warning, TEXT("(%f, %f)"), Centroid.X, Centroid.Y);
			}*/
		}
		if (enableKMeans)
		{
			// Set the number of clusters
			int K = k;

			// Set a fixed seed (starting point) for the random number generator
			srand(245);

			// Run the KMeans clustering algorithm
			int iters = 100;
			KMeans kmeans(K, iters);
			kmeans.run(kmeans_all_points);

			// Print the final cluster assignments for each point
			/*for (int q = 0; q < kmeans_all_points.Num(); q++)
			{
				UE_LOG(LogTemp, Log, TEXT("point %d: (%f, %f) -> cluster %d"), q, kmeans_all_points[q].getVal(0), kmeans_all_points[q].getVal(1), kmeans_all_points[q].getCluster());
			}*/

			// Create a TArray to store the coordinates of the centroids at this time step
			kmeans_centroids.SetNum(K);
			const TArray<Cluster>& clusters = kmeans.getClusters();

			for (int w = 0; w < K; w++)
			{
				double x = clusters[w].getCentroidByPos(0);
				double y = clusters[w].getCentroidByPos(1);

				// Store the coordinates of the centroid in the kmeans_centroids TArray
				kmeans_centroids[w] = FVector2D(x, y);
			}

			bubbleSort(kmeans_centroids);

			// Print the sorted kmeans_centroids TArray
			//for (int r = 0; r < kmeans_centroids.Num(); r++) {
			//	UE_LOG(LogTemp, Warning, TEXT("Centroid %d: (%f, %f)"), r + 1, kmeans_centroids[r].X, kmeans_centroids[r].Y);
			//}
		}
	}
	// Free memory
	free(data);

	return true;
}

TArray<FVector2D> UHokuyoReadBPLibrary::CalibrateCentroids(TArray<FVector2D> current_centroids, FVector2D sensorMinXY, FVector2D sensorMaxXY, FVector2D screenMinXY, FVector2D screenMaxXY)
{
	// Create a new TArray to store the calibrated centroids
	TArray<FVector2D> calibrated_centroids;
	calibrated_centroids.SetNum(current_centroids.Num());

	for (int i = 0; i < current_centroids.Num(); i++)
	{
		// Scale the sensor coordinates to the screen size
		float screenX = ((current_centroids[i].X - sensorMinXY.X) / (sensorMaxXY.X - sensorMinXY.X)) * (screenMaxXY.X - screenMinXY.X) + screenMinXY.X;
		float screenY = ((current_centroids[i].Y - sensorMinXY.Y) / (sensorMaxXY.Y - sensorMinXY.Y)) * (screenMaxXY.Y - screenMinXY.Y) + screenMinXY.Y;

		// Negate the screenX value to invert the X-axis
		//screenX = screenMaxXY.X - screenX;

		// Store the calibrated coordinates in the calibrated_centroids TArray
		calibrated_centroids[i] = FVector2D(screenX, screenY);
	}

	return calibrated_centroids;
}

TArray<FVector2D> UHokuyoReadBPLibrary::GetKMeansCentroids()
{
	return kmeans_centroids;
}
TArray<FVector2D> UHokuyoReadBPLibrary::GetDBSCANCentroids()
{
	return db_centroids;
}

void UHokuyoReadBPLibrary::ClearKMeansCentroids()
{
	kmeans_centroids.Empty();
}

void UHokuyoReadBPLibrary::ClearDBSCANCentroids()
{
	db_centroids.Empty();
}

class FGetDistanceTask : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask<FGetDistanceTask>;

	FUrgWrapper& UrgWrapper;
	int CaptureTimes;
	bool enableKMeans;
	int k;
	bool enableDBSCAN;
	float eps;
	int min_pts;

public:
	FGetDistanceTask(FUrgWrapper& InUrgWrapper, int InCaptureTimes, bool InEnableKmeans, int InK, bool InEnableDBSCAN, float InEps, int InMin_pts)
		: UrgWrapper(InUrgWrapper), CaptureTimes(InCaptureTimes), enableKMeans(InEnableKmeans), k(InK), enableDBSCAN(InEnableDBSCAN), eps(InEps), min_pts(InMin_pts)
	{
	}

	void DoWork()
	{
		UHokuyoReadBPLibrary::getDistance(UrgWrapper, CaptureTimes, enableKMeans, k, enableDBSCAN, eps, min_pts);
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FGetDistanceTask, STATGROUP_ThreadPoolAsyncTasks);
	}
};

void UHokuyoReadBPLibrary::getDistanceNonBlocking(UPARAM(ref) FUrgWrapper& UrgWrapper, int CaptureTimes, bool enableKMeans, int k, bool enableDBSCAN, float eps, int min_pts)
{
	(new FAutoDeleteAsyncTask<FGetDistanceTask>(UrgWrapper, CaptureTimes, enableKMeans, k, enableDBSCAN, eps, min_pts))->StartBackgroundTask();
}