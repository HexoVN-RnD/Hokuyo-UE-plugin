// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "KmeansClustering.h"
#include "DBSCANClustering.h"
#include "urg_sensor.h"
#include "urg_utils.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "HokuyoReadBPLibrary.generated.h"

// Declare the global kmeans_centroids variable
extern TArray<FVector2D> kmeans_centroids;
extern TArray<FVector2D> db_centroids;

USTRUCT(BlueprintType)
struct FUrgWrapper //This is a wrapper struct that will be used to pass the urg_t type to the blueprint. Can't pass the urg_t type directly to the blueprint.
{
	GENERATED_BODY()

	// Add an instance of the urg_t type
	urg_t Urg;
};

UCLASS(BlueprintType)
class UHokuyoReadBPLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
public:
	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Calibrate Centroids Coordinates to Screen", KeyWords = "Hokuyo", ToolTip = "Turn sensor coordinates to screen coordinates.\nFor example, if your sensor X-axis runs from (-10) to (10), but, on a 1920x1080 screen, the X-axis of the screen runs from (0) to (1920),\nwe will need to scale the change so that (-10) sensor is (0) screen and (10) sensor is (1920) screen."), Category = "Hokuyo | Object Detection")
	static TArray<FVector2D> CalibrateCentroids(TArray<FVector2D> current_centroids, FVector2D sensorMinXY = FVector2D(-50.0f, 150.0f), FVector2D sensorMaxXY = FVector2D(50.0f, 400.0f), FVector2D screenMinXY = FVector2D(0.0f, 0.0f), FVector2D screenMaxXY = FVector2D(1920.0f, 1080.0f));

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get KMeans Centroids", KeyWords = "Hokuyo", ToolTip = "Need to enable in C++ if not working"), Category = "Hokuyo | Object Detection")
	static TArray<FVector2D> GetKMeansCentroids();

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Get DBSCAN Centroids", KeyWords = "Hokuyo", ToolTip = "Need to enable in C++ if not working"), Category = "Hokuyo | Object Detection")
	static TArray<FVector2D> GetDBSCANCentroids();

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Clear KMeans Centroids", KeyWords = "Hokuyo", ToolTip = "Clear the global variable"), Category = "Hokuyo | Object Detection")
	static void ClearKMeansCentroids();

	UFUNCTION(BlueprintCallable, meta = (DisplayName = "Clear DBSCAN Centroids", KeyWords = "Hokuyo", ToolTip = "Clear the global variable"), Category = "Hokuyo | Object Detection")
	static void ClearDBSCANCentroids();

	//get scanning distance results
	UFUNCTION(BlueprintCallable, Category = "Hokuyo | Scanning", meta = (DisplayName = "Get Distance Result (Blocking)", KeyWords = "Hokuyo", ToolTip = "This will execute on the main thread and freeze your program\n @param CaptureTimes Number of time it will run. 0 means it will run indefinitely"))
	static bool getDistance(UPARAM(ref) FUrgWrapper& UrgWrapper, int CaptureTimes, bool enableKMeans = false, int k = 1, bool enableDBSCAN = true, float eps = 30, int min_pts = 3);

	//async function that calls getDistance. This is used to call getDistance in a non-blocking way.
	UFUNCTION(BlueprintCallable, Category = "Hokuyo | Scanning", meta = (DisplayName = "Get Distance Result", KeyWords = "Hokuyo", ToolTip = "Get the Sensor Distance Measurement Result using KMeans or DBSCAN\n@param CaptureTimes Number of time it will run. 0 means it will run indefinitely\n@param k For KMeans: Number of clusters\n@param eps For DBSCAN: Epsilon, the maximum distance two points can be from one another while still belonging to the same cluster\n@param min_pts For DBSCAN: the fewest number of points required to form a cluster"))
	static void getDistanceNonBlocking(UPARAM(ref) FUrgWrapper& UrgWrapper, int CaptureTimes, bool enableKMeans = false, int k = 1, bool enableDBSCAN = true, float eps = 30, int min_pts = 3);


public:
	//open the connection
	UFUNCTION(BlueprintCallable, meta = (KeyWords = "Hokuyo", ToolTip = "Open Connection with Sensor @param connectionType 0 is Serial. 1 is Ethernet @param ipAddressOrDevice For Serial connection, enter device name like 'COM3'. For Ethernet connection, enter ip address like '192.168.1.1' @param portNumberOrBaudrate For Serial connection, enter baudrate. For Ethernet connection, enter port number"), Category = "Hokuyo | Connection")
	static bool openConnection(UPARAM(ref) FUrgWrapper& UrgWrapper, int32 connectionType = 1, FString ipAddressOrDevice = "192.168.3.202", int32 portNumberOrBaudrate = 10940); 
	//UPARAM(ref) is used to pass a reference to the struct. UE doesn't support references by default using '&'.

	//close the connection
	UFUNCTION(BlueprintCallable, meta = (KeyWords = "Hokuyo"), Category = "Hokuyo | Connection")
	static bool closeConnection(UPARAM(ref) FUrgWrapper& UrgWrapper);

	//check the boolean result, if it's false, close the connection
	UFUNCTION(BlueprintCallable, meta = (KeyWords = "Hokuyo"), Category = "Hokuyo | Connection")
	static void closeConnectionifFalse(bool Result, UPARAM(ref) FUrgWrapper& UrgWrapper);

	//set scanning degrees
	UFUNCTION(BlueprintCallable, meta = (KeyWords = "Hokuyo", ToolTip = "Set Sensor Scanning Degree to the left and right of the vertical axis from origin point @param LeftDegree Scanning Degree to the left of the vertical axis from origin point @param RightDegree Scanning Degree to the right of the vertical axis from origin point"), Category = "Hokuyo | Connection")
	static bool setScanningDegree(UPARAM(ref) FUrgWrapper& UrgWrapper, int LeftDegree, int RightDegree);
};


