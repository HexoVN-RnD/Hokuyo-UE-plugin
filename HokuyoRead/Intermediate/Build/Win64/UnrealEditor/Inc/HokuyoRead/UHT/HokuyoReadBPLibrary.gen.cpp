// Copyright Epic Games, Inc. All Rights Reserved.
/*===========================================================================
	Generated code exported from UnrealHeaderTool.
	DO NOT modify this manually! Edit the corresponding .h files instead!
===========================================================================*/

#include "UObject/GeneratedCppIncludes.h"
#include "../../Public/HokuyoReadBPLibrary.h"
PRAGMA_DISABLE_DEPRECATION_WARNINGS
void EmptyLinkFunctionForGeneratedCodeHokuyoReadBPLibrary() {}
// Cross Module References
	COREUOBJECT_API UScriptStruct* Z_Construct_UScriptStruct_FVector2D();
	ENGINE_API UClass* Z_Construct_UClass_UBlueprintFunctionLibrary();
	HOKUYOREAD_API UClass* Z_Construct_UClass_UHokuyoReadBPLibrary();
	HOKUYOREAD_API UClass* Z_Construct_UClass_UHokuyoReadBPLibrary_NoRegister();
	HOKUYOREAD_API UScriptStruct* Z_Construct_UScriptStruct_FUrgWrapper();
	UPackage* Z_Construct_UPackage__Script_HokuyoRead();
// End Cross Module References
	static FStructRegistrationInfo Z_Registration_Info_UScriptStruct_UrgWrapper;
class UScriptStruct* FUrgWrapper::StaticStruct()
{
	if (!Z_Registration_Info_UScriptStruct_UrgWrapper.OuterSingleton)
	{
		Z_Registration_Info_UScriptStruct_UrgWrapper.OuterSingleton = GetStaticStruct(Z_Construct_UScriptStruct_FUrgWrapper, (UObject*)Z_Construct_UPackage__Script_HokuyoRead(), TEXT("UrgWrapper"));
	}
	return Z_Registration_Info_UScriptStruct_UrgWrapper.OuterSingleton;
}
template<> HOKUYOREAD_API UScriptStruct* StaticStruct<FUrgWrapper>()
{
	return FUrgWrapper::StaticStruct();
}
	struct Z_Construct_UScriptStruct_FUrgWrapper_Statics
	{
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Struct_MetaDataParams[];
#endif
		static void* NewStructOps();
		static const UECodeGen_Private::FStructParams ReturnStructParams;
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UScriptStruct_FUrgWrapper_Statics::Struct_MetaDataParams[] = {
		{ "BlueprintType", "true" },
		{ "Comment", "//This is a wrapper struct that will be used to pass the urg_t type to the blueprint. Can't pass the urg_t type directly to the blueprint.\n" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "This is a wrapper struct that will be used to pass the urg_t type to the blueprint. Can't pass the urg_t type directly to the blueprint." },
	};
#endif
	void* Z_Construct_UScriptStruct_FUrgWrapper_Statics::NewStructOps()
	{
		return (UScriptStruct::ICppStructOps*)new UScriptStruct::TCppStructOps<FUrgWrapper>();
	}
	const UECodeGen_Private::FStructParams Z_Construct_UScriptStruct_FUrgWrapper_Statics::ReturnStructParams = {
		(UObject* (*)())Z_Construct_UPackage__Script_HokuyoRead,
		nullptr,
		&NewStructOps,
		"UrgWrapper",
		sizeof(FUrgWrapper),
		alignof(FUrgWrapper),
		nullptr,
		0,
		RF_Public|RF_Transient|RF_MarkAsNative,
		EStructFlags(0x00000001),
		METADATA_PARAMS(Z_Construct_UScriptStruct_FUrgWrapper_Statics::Struct_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UScriptStruct_FUrgWrapper_Statics::Struct_MetaDataParams))
	};
	UScriptStruct* Z_Construct_UScriptStruct_FUrgWrapper()
	{
		if (!Z_Registration_Info_UScriptStruct_UrgWrapper.InnerSingleton)
		{
			UECodeGen_Private::ConstructUScriptStruct(Z_Registration_Info_UScriptStruct_UrgWrapper.InnerSingleton, Z_Construct_UScriptStruct_FUrgWrapper_Statics::ReturnStructParams);
		}
		return Z_Registration_Info_UScriptStruct_UrgWrapper.InnerSingleton;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execsetScanningDegree)
	{
		P_GET_STRUCT_REF(FUrgWrapper,Z_Param_Out_UrgWrapper);
		P_GET_PROPERTY(FIntProperty,Z_Param_LeftDegree);
		P_GET_PROPERTY(FIntProperty,Z_Param_RightDegree);
		P_FINISH;
		P_NATIVE_BEGIN;
		*(bool*)Z_Param__Result=UHokuyoReadBPLibrary::setScanningDegree(Z_Param_Out_UrgWrapper,Z_Param_LeftDegree,Z_Param_RightDegree);
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execcloseConnectionifFalse)
	{
		P_GET_UBOOL(Z_Param_Result);
		P_GET_STRUCT_REF(FUrgWrapper,Z_Param_Out_UrgWrapper);
		P_FINISH;
		P_NATIVE_BEGIN;
		UHokuyoReadBPLibrary::closeConnectionifFalse(Z_Param_Result,Z_Param_Out_UrgWrapper);
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execcloseConnection)
	{
		P_GET_STRUCT_REF(FUrgWrapper,Z_Param_Out_UrgWrapper);
		P_FINISH;
		P_NATIVE_BEGIN;
		*(bool*)Z_Param__Result=UHokuyoReadBPLibrary::closeConnection(Z_Param_Out_UrgWrapper);
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execopenConnection)
	{
		P_GET_STRUCT_REF(FUrgWrapper,Z_Param_Out_UrgWrapper);
		P_GET_PROPERTY(FIntProperty,Z_Param_connectionType);
		P_GET_PROPERTY(FStrProperty,Z_Param_ipAddressOrDevice);
		P_GET_PROPERTY(FIntProperty,Z_Param_portNumberOrBaudrate);
		P_FINISH;
		P_NATIVE_BEGIN;
		*(bool*)Z_Param__Result=UHokuyoReadBPLibrary::openConnection(Z_Param_Out_UrgWrapper,Z_Param_connectionType,Z_Param_ipAddressOrDevice,Z_Param_portNumberOrBaudrate);
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execgetDistanceNonBlocking)
	{
		P_GET_STRUCT_REF(FUrgWrapper,Z_Param_Out_UrgWrapper);
		P_GET_PROPERTY(FIntProperty,Z_Param_CaptureTimes);
		P_GET_UBOOL(Z_Param_enableKMeans);
		P_GET_PROPERTY(FIntProperty,Z_Param_k);
		P_GET_UBOOL(Z_Param_enableDBSCAN);
		P_GET_PROPERTY(FFloatProperty,Z_Param_eps);
		P_GET_PROPERTY(FIntProperty,Z_Param_min_pts);
		P_FINISH;
		P_NATIVE_BEGIN;
		UHokuyoReadBPLibrary::getDistanceNonBlocking(Z_Param_Out_UrgWrapper,Z_Param_CaptureTimes,Z_Param_enableKMeans,Z_Param_k,Z_Param_enableDBSCAN,Z_Param_eps,Z_Param_min_pts);
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execgetDistance)
	{
		P_GET_STRUCT_REF(FUrgWrapper,Z_Param_Out_UrgWrapper);
		P_GET_PROPERTY(FIntProperty,Z_Param_CaptureTimes);
		P_GET_UBOOL(Z_Param_enableKMeans);
		P_GET_PROPERTY(FIntProperty,Z_Param_k);
		P_GET_UBOOL(Z_Param_enableDBSCAN);
		P_GET_PROPERTY(FFloatProperty,Z_Param_eps);
		P_GET_PROPERTY(FIntProperty,Z_Param_min_pts);
		P_FINISH;
		P_NATIVE_BEGIN;
		*(bool*)Z_Param__Result=UHokuyoReadBPLibrary::getDistance(Z_Param_Out_UrgWrapper,Z_Param_CaptureTimes,Z_Param_enableKMeans,Z_Param_k,Z_Param_enableDBSCAN,Z_Param_eps,Z_Param_min_pts);
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execClearDBSCANCentroids)
	{
		P_FINISH;
		P_NATIVE_BEGIN;
		UHokuyoReadBPLibrary::ClearDBSCANCentroids();
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execClearKMeansCentroids)
	{
		P_FINISH;
		P_NATIVE_BEGIN;
		UHokuyoReadBPLibrary::ClearKMeansCentroids();
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execGetDBSCANCentroids)
	{
		P_FINISH;
		P_NATIVE_BEGIN;
		*(TArray<FVector2D>*)Z_Param__Result=UHokuyoReadBPLibrary::GetDBSCANCentroids();
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execGetKMeansCentroids)
	{
		P_FINISH;
		P_NATIVE_BEGIN;
		*(TArray<FVector2D>*)Z_Param__Result=UHokuyoReadBPLibrary::GetKMeansCentroids();
		P_NATIVE_END;
	}
	DEFINE_FUNCTION(UHokuyoReadBPLibrary::execCalibrateCentroids)
	{
		P_GET_TARRAY(FVector2D,Z_Param_current_centroids);
		P_GET_STRUCT(FVector2D,Z_Param_sensorMinXY);
		P_GET_STRUCT(FVector2D,Z_Param_sensorMaxXY);
		P_GET_STRUCT(FVector2D,Z_Param_screenMinXY);
		P_GET_STRUCT(FVector2D,Z_Param_screenMaxXY);
		P_FINISH;
		P_NATIVE_BEGIN;
		*(TArray<FVector2D>*)Z_Param__Result=UHokuyoReadBPLibrary::CalibrateCentroids(Z_Param_current_centroids,Z_Param_sensorMinXY,Z_Param_sensorMaxXY,Z_Param_screenMinXY,Z_Param_screenMaxXY);
		P_NATIVE_END;
	}
	void UHokuyoReadBPLibrary::StaticRegisterNativesUHokuyoReadBPLibrary()
	{
		UClass* Class = UHokuyoReadBPLibrary::StaticClass();
		static const FNameNativePtrPair Funcs[] = {
			{ "CalibrateCentroids", &UHokuyoReadBPLibrary::execCalibrateCentroids },
			{ "ClearDBSCANCentroids", &UHokuyoReadBPLibrary::execClearDBSCANCentroids },
			{ "ClearKMeansCentroids", &UHokuyoReadBPLibrary::execClearKMeansCentroids },
			{ "closeConnection", &UHokuyoReadBPLibrary::execcloseConnection },
			{ "closeConnectionifFalse", &UHokuyoReadBPLibrary::execcloseConnectionifFalse },
			{ "GetDBSCANCentroids", &UHokuyoReadBPLibrary::execGetDBSCANCentroids },
			{ "getDistance", &UHokuyoReadBPLibrary::execgetDistance },
			{ "getDistanceNonBlocking", &UHokuyoReadBPLibrary::execgetDistanceNonBlocking },
			{ "GetKMeansCentroids", &UHokuyoReadBPLibrary::execGetKMeansCentroids },
			{ "openConnection", &UHokuyoReadBPLibrary::execopenConnection },
			{ "setScanningDegree", &UHokuyoReadBPLibrary::execsetScanningDegree },
		};
		FNativeFunctionRegistrar::RegisterFunctions(Class, Funcs, UE_ARRAY_COUNT(Funcs));
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics
	{
		struct HokuyoReadBPLibrary_eventCalibrateCentroids_Parms
		{
			TArray<FVector2D> current_centroids;
			FVector2D sensorMinXY;
			FVector2D sensorMaxXY;
			FVector2D screenMinXY;
			FVector2D screenMaxXY;
			TArray<FVector2D> ReturnValue;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_current_centroids_Inner;
		static const UECodeGen_Private::FArrayPropertyParams NewProp_current_centroids;
		static const UECodeGen_Private::FStructPropertyParams NewProp_sensorMinXY;
		static const UECodeGen_Private::FStructPropertyParams NewProp_sensorMaxXY;
		static const UECodeGen_Private::FStructPropertyParams NewProp_screenMinXY;
		static const UECodeGen_Private::FStructPropertyParams NewProp_screenMaxXY;
		static const UECodeGen_Private::FStructPropertyParams NewProp_ReturnValue_Inner;
		static const UECodeGen_Private::FArrayPropertyParams NewProp_ReturnValue;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_current_centroids_Inner = { "current_centroids", nullptr, (EPropertyFlags)0x0000000000000000, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, 0, Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FArrayPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_current_centroids = { "current_centroids", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Array, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventCalibrateCentroids_Parms, current_centroids), EArrayPropertyFlags::None, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_sensorMinXY = { "sensorMinXY", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventCalibrateCentroids_Parms, sensorMinXY), Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_sensorMaxXY = { "sensorMaxXY", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventCalibrateCentroids_Parms, sensorMaxXY), Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_screenMinXY = { "screenMinXY", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventCalibrateCentroids_Parms, screenMinXY), Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_screenMaxXY = { "screenMaxXY", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventCalibrateCentroids_Parms, screenMaxXY), Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_ReturnValue_Inner = { "ReturnValue", nullptr, (EPropertyFlags)0x0000000000000000, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, 0, Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FArrayPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_ReturnValue = { "ReturnValue", nullptr, (EPropertyFlags)0x0010000000000580, UECodeGen_Private::EPropertyGenFlags::Array, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventCalibrateCentroids_Parms, ReturnValue), EArrayPropertyFlags::None, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_current_centroids_Inner,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_current_centroids,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_sensorMinXY,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_sensorMaxXY,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_screenMinXY,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_screenMaxXY,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_ReturnValue_Inner,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::NewProp_ReturnValue,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Object Detection" },
		{ "CPP_Default_screenMaxXY", "(X=1920.000,Y=1080.000)" },
		{ "CPP_Default_screenMinXY", "(X=0.000,Y=0.000)" },
		{ "CPP_Default_sensorMaxXY", "(X=50.000,Y=400.000)" },
		{ "CPP_Default_sensorMinXY", "(X=-50.000,Y=150.000)" },
		{ "DisplayName", "Calibrate Centroids Coordinates to Screen" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Turn sensor coordinates to screen coordinates.\nFor example, if your sensor X-axis runs from (-10) to (10), but, on a 1920x1080 screen, the X-axis of the screen runs from (0) to (1920),\nwe will need to scale the change so that (-10) sensor is (0) screen and (10) sensor is (1920) screen." },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "CalibrateCentroids", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::HokuyoReadBPLibrary_eventCalibrateCentroids_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04822401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids_Statics
	{
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Object Detection" },
		{ "DisplayName", "Clear DBSCAN Centroids" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Clear the global variable" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "ClearDBSCANCentroids", nullptr, nullptr, 0, nullptr, 0, RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04022401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids_Statics
	{
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Object Detection" },
		{ "DisplayName", "Clear KMeans Centroids" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Clear the global variable" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "ClearKMeansCentroids", nullptr, nullptr, 0, nullptr, 0, RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04022401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics
	{
		struct HokuyoReadBPLibrary_eventcloseConnection_Parms
		{
			FUrgWrapper UrgWrapper;
			bool ReturnValue;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_UrgWrapper;
		static void NewProp_ReturnValue_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_ReturnValue;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::NewProp_UrgWrapper = { "UrgWrapper", nullptr, (EPropertyFlags)0x0010000008000180, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventcloseConnection_Parms, UrgWrapper), Z_Construct_UScriptStruct_FUrgWrapper, METADATA_PARAMS(nullptr, 0) }; // 3668095923
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::NewProp_ReturnValue_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventcloseConnection_Parms*)Obj)->ReturnValue = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::NewProp_ReturnValue = { "ReturnValue", nullptr, (EPropertyFlags)0x0010000000000580, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventcloseConnection_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::NewProp_ReturnValue_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::NewProp_UrgWrapper,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::NewProp_ReturnValue,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Connection" },
		{ "Comment", "//close the connection\n" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "close the connection" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "closeConnection", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::HokuyoReadBPLibrary_eventcloseConnection_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04422401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics
	{
		struct HokuyoReadBPLibrary_eventcloseConnectionifFalse_Parms
		{
			bool Result;
			FUrgWrapper UrgWrapper;
		};
		static void NewProp_Result_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_Result;
		static const UECodeGen_Private::FStructPropertyParams NewProp_UrgWrapper;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::NewProp_Result_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventcloseConnectionifFalse_Parms*)Obj)->Result = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::NewProp_Result = { "Result", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventcloseConnectionifFalse_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::NewProp_Result_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::NewProp_UrgWrapper = { "UrgWrapper", nullptr, (EPropertyFlags)0x0010000008000180, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventcloseConnectionifFalse_Parms, UrgWrapper), Z_Construct_UScriptStruct_FUrgWrapper, METADATA_PARAMS(nullptr, 0) }; // 3668095923
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::NewProp_Result,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::NewProp_UrgWrapper,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Connection" },
		{ "Comment", "//check the boolean result, if it's false, close the connection\n" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "check the boolean result, if it's false, close the connection" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "closeConnectionifFalse", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::HokuyoReadBPLibrary_eventcloseConnectionifFalse_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04422401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics
	{
		struct HokuyoReadBPLibrary_eventGetDBSCANCentroids_Parms
		{
			TArray<FVector2D> ReturnValue;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_ReturnValue_Inner;
		static const UECodeGen_Private::FArrayPropertyParams NewProp_ReturnValue;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::NewProp_ReturnValue_Inner = { "ReturnValue", nullptr, (EPropertyFlags)0x0000000000000000, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, 0, Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FArrayPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::NewProp_ReturnValue = { "ReturnValue", nullptr, (EPropertyFlags)0x0010000000000580, UECodeGen_Private::EPropertyGenFlags::Array, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventGetDBSCANCentroids_Parms, ReturnValue), EArrayPropertyFlags::None, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::NewProp_ReturnValue_Inner,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::NewProp_ReturnValue,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Object Detection" },
		{ "DisplayName", "Get DBSCAN Centroids" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Need to enable in C++ if not working" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "GetDBSCANCentroids", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::HokuyoReadBPLibrary_eventGetDBSCANCentroids_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04022401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics
	{
		struct HokuyoReadBPLibrary_eventgetDistance_Parms
		{
			FUrgWrapper UrgWrapper;
			int32 CaptureTimes;
			bool enableKMeans;
			int32 k;
			bool enableDBSCAN;
			float eps;
			int32 min_pts;
			bool ReturnValue;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_UrgWrapper;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_CaptureTimes;
		static void NewProp_enableKMeans_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_enableKMeans;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_k;
		static void NewProp_enableDBSCAN_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_enableDBSCAN;
		static const UECodeGen_Private::FFloatPropertyParams NewProp_eps;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_min_pts;
		static void NewProp_ReturnValue_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_ReturnValue;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_UrgWrapper = { "UrgWrapper", nullptr, (EPropertyFlags)0x0010000008000180, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistance_Parms, UrgWrapper), Z_Construct_UScriptStruct_FUrgWrapper, METADATA_PARAMS(nullptr, 0) }; // 3668095923
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_CaptureTimes = { "CaptureTimes", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistance_Parms, CaptureTimes), METADATA_PARAMS(nullptr, 0) };
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableKMeans_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventgetDistance_Parms*)Obj)->enableKMeans = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableKMeans = { "enableKMeans", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventgetDistance_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableKMeans_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_k = { "k", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistance_Parms, k), METADATA_PARAMS(nullptr, 0) };
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableDBSCAN_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventgetDistance_Parms*)Obj)->enableDBSCAN = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableDBSCAN = { "enableDBSCAN", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventgetDistance_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableDBSCAN_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FFloatPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_eps = { "eps", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Float, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistance_Parms, eps), METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_min_pts = { "min_pts", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistance_Parms, min_pts), METADATA_PARAMS(nullptr, 0) };
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_ReturnValue_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventgetDistance_Parms*)Obj)->ReturnValue = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_ReturnValue = { "ReturnValue", nullptr, (EPropertyFlags)0x0010000000000580, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventgetDistance_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_ReturnValue_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_UrgWrapper,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_CaptureTimes,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableKMeans,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_k,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_enableDBSCAN,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_eps,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_min_pts,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::NewProp_ReturnValue,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Scanning" },
		{ "CPP_Default_enableDBSCAN", "true" },
		{ "CPP_Default_enableKMeans", "false" },
		{ "CPP_Default_eps", "30.000000" },
		{ "CPP_Default_k", "1" },
		{ "CPP_Default_min_pts", "3" },
		{ "DisplayName", "Get Distance Result (Blocking)" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "This will execute on the main thread and freeze your program\n @param CaptureTimes Number of time it will run. 0 means it will run indefinitely" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "getDistance", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::HokuyoReadBPLibrary_eventgetDistance_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04422401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics
	{
		struct HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms
		{
			FUrgWrapper UrgWrapper;
			int32 CaptureTimes;
			bool enableKMeans;
			int32 k;
			bool enableDBSCAN;
			float eps;
			int32 min_pts;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_UrgWrapper;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_CaptureTimes;
		static void NewProp_enableKMeans_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_enableKMeans;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_k;
		static void NewProp_enableDBSCAN_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_enableDBSCAN;
		static const UECodeGen_Private::FFloatPropertyParams NewProp_eps;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_min_pts;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_UrgWrapper = { "UrgWrapper", nullptr, (EPropertyFlags)0x0010000008000180, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms, UrgWrapper), Z_Construct_UScriptStruct_FUrgWrapper, METADATA_PARAMS(nullptr, 0) }; // 3668095923
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_CaptureTimes = { "CaptureTimes", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms, CaptureTimes), METADATA_PARAMS(nullptr, 0) };
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableKMeans_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms*)Obj)->enableKMeans = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableKMeans = { "enableKMeans", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableKMeans_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_k = { "k", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms, k), METADATA_PARAMS(nullptr, 0) };
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableDBSCAN_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms*)Obj)->enableDBSCAN = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableDBSCAN = { "enableDBSCAN", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableDBSCAN_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FFloatPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_eps = { "eps", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Float, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms, eps), METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_min_pts = { "min_pts", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms, min_pts), METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_UrgWrapper,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_CaptureTimes,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableKMeans,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_k,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_enableDBSCAN,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_eps,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::NewProp_min_pts,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Scanning" },
		{ "CPP_Default_enableDBSCAN", "true" },
		{ "CPP_Default_enableKMeans", "false" },
		{ "CPP_Default_eps", "30.000000" },
		{ "CPP_Default_k", "1" },
		{ "CPP_Default_min_pts", "3" },
		{ "DisplayName", "Get Distance Result" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Get the Sensor Distance Measurement Result using KMeans or DBSCAN\n@param CaptureTimes Number of time it will run. 0 means it will run indefinitely\n@param k For KMeans: Number of clusters\n@param eps For DBSCAN: Epsilon, the maximum distance two points can be from one another while still belonging to the same cluster\n@param min_pts For DBSCAN: the fewest number of points required to form a cluster" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "getDistanceNonBlocking", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::HokuyoReadBPLibrary_eventgetDistanceNonBlocking_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04422401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics
	{
		struct HokuyoReadBPLibrary_eventGetKMeansCentroids_Parms
		{
			TArray<FVector2D> ReturnValue;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_ReturnValue_Inner;
		static const UECodeGen_Private::FArrayPropertyParams NewProp_ReturnValue;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::NewProp_ReturnValue_Inner = { "ReturnValue", nullptr, (EPropertyFlags)0x0000000000000000, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, 0, Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FArrayPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::NewProp_ReturnValue = { "ReturnValue", nullptr, (EPropertyFlags)0x0010000000000580, UECodeGen_Private::EPropertyGenFlags::Array, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventGetKMeansCentroids_Parms, ReturnValue), EArrayPropertyFlags::None, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::NewProp_ReturnValue_Inner,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::NewProp_ReturnValue,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Object Detection" },
		{ "DisplayName", "Get KMeans Centroids" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Need to enable in C++ if not working" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "GetKMeansCentroids", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::HokuyoReadBPLibrary_eventGetKMeansCentroids_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04022401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics
	{
		struct HokuyoReadBPLibrary_eventopenConnection_Parms
		{
			FUrgWrapper UrgWrapper;
			int32 connectionType;
			FString ipAddressOrDevice;
			int32 portNumberOrBaudrate;
			bool ReturnValue;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_UrgWrapper;
		static const UECodeGen_Private::FIntPropertyParams NewProp_connectionType;
		static const UECodeGen_Private::FStrPropertyParams NewProp_ipAddressOrDevice;
		static const UECodeGen_Private::FIntPropertyParams NewProp_portNumberOrBaudrate;
		static void NewProp_ReturnValue_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_ReturnValue;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_UrgWrapper = { "UrgWrapper", nullptr, (EPropertyFlags)0x0010000008000180, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventopenConnection_Parms, UrgWrapper), Z_Construct_UScriptStruct_FUrgWrapper, METADATA_PARAMS(nullptr, 0) }; // 3668095923
	const UECodeGen_Private::FIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_connectionType = { "connectionType", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventopenConnection_Parms, connectionType), METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FStrPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_ipAddressOrDevice = { "ipAddressOrDevice", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Str, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventopenConnection_Parms, ipAddressOrDevice), METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_portNumberOrBaudrate = { "portNumberOrBaudrate", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventopenConnection_Parms, portNumberOrBaudrate), METADATA_PARAMS(nullptr, 0) };
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_ReturnValue_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventopenConnection_Parms*)Obj)->ReturnValue = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_ReturnValue = { "ReturnValue", nullptr, (EPropertyFlags)0x0010000000000580, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventopenConnection_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_ReturnValue_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_UrgWrapper,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_connectionType,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_ipAddressOrDevice,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_portNumberOrBaudrate,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::NewProp_ReturnValue,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Connection" },
		{ "CPP_Default_connectionType", "1" },
		{ "CPP_Default_ipAddressOrDevice", "192.168.3.202" },
		{ "CPP_Default_portNumberOrBaudrate", "10940" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Open Connection with Sensor @param connectionType 0 is Serial. 1 is Ethernet @param ipAddressOrDevice For Serial connection, enter device name like 'COM3'. For Ethernet connection, enter ip address like '192.168.1.1' @param portNumberOrBaudrate For Serial connection, enter baudrate. For Ethernet connection, enter port number" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "openConnection", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::HokuyoReadBPLibrary_eventopenConnection_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04422401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	struct Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics
	{
		struct HokuyoReadBPLibrary_eventsetScanningDegree_Parms
		{
			FUrgWrapper UrgWrapper;
			int32 LeftDegree;
			int32 RightDegree;
			bool ReturnValue;
		};
		static const UECodeGen_Private::FStructPropertyParams NewProp_UrgWrapper;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_LeftDegree;
		static const UECodeGen_Private::FUnsizedIntPropertyParams NewProp_RightDegree;
		static void NewProp_ReturnValue_SetBit(void* Obj);
		static const UECodeGen_Private::FBoolPropertyParams NewProp_ReturnValue;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_UrgWrapper = { "UrgWrapper", nullptr, (EPropertyFlags)0x0010000008000180, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventsetScanningDegree_Parms, UrgWrapper), Z_Construct_UScriptStruct_FUrgWrapper, METADATA_PARAMS(nullptr, 0) }; // 3668095923
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_LeftDegree = { "LeftDegree", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventsetScanningDegree_Parms, LeftDegree), METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FUnsizedIntPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_RightDegree = { "RightDegree", nullptr, (EPropertyFlags)0x0010000000000080, UECodeGen_Private::EPropertyGenFlags::Int, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoReadBPLibrary_eventsetScanningDegree_Parms, RightDegree), METADATA_PARAMS(nullptr, 0) };
	void Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_ReturnValue_SetBit(void* Obj)
	{
		((HokuyoReadBPLibrary_eventsetScanningDegree_Parms*)Obj)->ReturnValue = 1;
	}
	const UECodeGen_Private::FBoolPropertyParams Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_ReturnValue = { "ReturnValue", nullptr, (EPropertyFlags)0x0010000000000580, UECodeGen_Private::EPropertyGenFlags::Bool | UECodeGen_Private::EPropertyGenFlags::NativeBool, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, sizeof(bool), sizeof(HokuyoReadBPLibrary_eventsetScanningDegree_Parms), &Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_ReturnValue_SetBit, METADATA_PARAMS(nullptr, 0) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_UrgWrapper,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_LeftDegree,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_RightDegree,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::NewProp_ReturnValue,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Connection" },
		{ "KeyWords", "Hokuyo" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
		{ "ToolTip", "Set Sensor Scanning Degree to the left and right of the vertical axis from origin point @param LeftDegree Scanning Degree to the left of the vertical axis from origin point @param RightDegree Scanning Degree to the right of the vertical axis from origin point" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoReadBPLibrary, nullptr, "setScanningDegree", nullptr, nullptr, sizeof(Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::HokuyoReadBPLibrary_eventsetScanningDegree_Parms), Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x04422401, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	IMPLEMENT_CLASS_NO_AUTO_REGISTRATION(UHokuyoReadBPLibrary);
	UClass* Z_Construct_UClass_UHokuyoReadBPLibrary_NoRegister()
	{
		return UHokuyoReadBPLibrary::StaticClass();
	}
	struct Z_Construct_UClass_UHokuyoReadBPLibrary_Statics
	{
		static UObject* (*const DependentSingletons[])();
		static const FClassFunctionLinkInfo FuncInfo[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Class_MetaDataParams[];
#endif
		static const FCppClassTypeInfoStatic StaticCppClassTypeInfo;
		static const UECodeGen_Private::FClassParams ClassParams;
	};
	UObject* (*const Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::DependentSingletons[])() = {
		(UObject* (*)())Z_Construct_UClass_UBlueprintFunctionLibrary,
		(UObject* (*)())Z_Construct_UPackage__Script_HokuyoRead,
	};
	const FClassFunctionLinkInfo Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::FuncInfo[] = {
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_CalibrateCentroids, "CalibrateCentroids" }, // 276357171
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearDBSCANCentroids, "ClearDBSCANCentroids" }, // 911219002
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_ClearKMeansCentroids, "ClearKMeansCentroids" }, // 2783082243
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnection, "closeConnection" }, // 2938648371
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_closeConnectionifFalse, "closeConnectionifFalse" }, // 735012480
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_GetDBSCANCentroids, "GetDBSCANCentroids" }, // 2469542561
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistance, "getDistance" }, // 871696623
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_getDistanceNonBlocking, "getDistanceNonBlocking" }, // 3210822216
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_GetKMeansCentroids, "GetKMeansCentroids" }, // 2226067403
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_openConnection, "openConnection" }, // 962609297
		{ &Z_Construct_UFunction_UHokuyoReadBPLibrary_setScanningDegree, "setScanningDegree" }, // 1014760031
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::Class_MetaDataParams[] = {
		{ "BlueprintType", "true" },
		{ "IncludePath", "HokuyoReadBPLibrary.h" },
		{ "ModuleRelativePath", "Public/HokuyoReadBPLibrary.h" },
	};
#endif
	const FCppClassTypeInfoStatic Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::StaticCppClassTypeInfo = {
		TCppClassTypeTraits<UHokuyoReadBPLibrary>::IsAbstract,
	};
	const UECodeGen_Private::FClassParams Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::ClassParams = {
		&UHokuyoReadBPLibrary::StaticClass,
		nullptr,
		&StaticCppClassTypeInfo,
		DependentSingletons,
		FuncInfo,
		nullptr,
		nullptr,
		UE_ARRAY_COUNT(DependentSingletons),
		UE_ARRAY_COUNT(FuncInfo),
		0,
		0,
		0x000000A0u,
		METADATA_PARAMS(Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::Class_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::Class_MetaDataParams))
	};
	UClass* Z_Construct_UClass_UHokuyoReadBPLibrary()
	{
		if (!Z_Registration_Info_UClass_UHokuyoReadBPLibrary.OuterSingleton)
		{
			UECodeGen_Private::ConstructUClass(Z_Registration_Info_UClass_UHokuyoReadBPLibrary.OuterSingleton, Z_Construct_UClass_UHokuyoReadBPLibrary_Statics::ClassParams);
		}
		return Z_Registration_Info_UClass_UHokuyoReadBPLibrary.OuterSingleton;
	}
	template<> HOKUYOREAD_API UClass* StaticClass<UHokuyoReadBPLibrary>()
	{
		return UHokuyoReadBPLibrary::StaticClass();
	}
	UHokuyoReadBPLibrary::UHokuyoReadBPLibrary(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer) {}
	DEFINE_VTABLE_PTR_HELPER_CTOR(UHokuyoReadBPLibrary);
	UHokuyoReadBPLibrary::~UHokuyoReadBPLibrary() {}
	struct Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_Statics
	{
		static const FStructRegisterCompiledInInfo ScriptStructInfo[];
		static const FClassRegisterCompiledInInfo ClassInfo[];
	};
	const FStructRegisterCompiledInInfo Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_Statics::ScriptStructInfo[] = {
		{ FUrgWrapper::StaticStruct, Z_Construct_UScriptStruct_FUrgWrapper_Statics::NewStructOps, TEXT("UrgWrapper"), &Z_Registration_Info_UScriptStruct_UrgWrapper, CONSTRUCT_RELOAD_VERSION_INFO(FStructReloadVersionInfo, sizeof(FUrgWrapper), 3668095923U) },
	};
	const FClassRegisterCompiledInInfo Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_Statics::ClassInfo[] = {
		{ Z_Construct_UClass_UHokuyoReadBPLibrary, UHokuyoReadBPLibrary::StaticClass, TEXT("UHokuyoReadBPLibrary"), &Z_Registration_Info_UClass_UHokuyoReadBPLibrary, CONSTRUCT_RELOAD_VERSION_INFO(FClassReloadVersionInfo, sizeof(UHokuyoReadBPLibrary), 3837717703U) },
	};
	static FRegisterCompiledInInfo Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_1929878311(TEXT("/Script/HokuyoRead"),
		Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_Statics::ClassInfo, UE_ARRAY_COUNT(Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_Statics::ClassInfo),
		Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_Statics::ScriptStructInfo, UE_ARRAY_COUNT(Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_Statics::ScriptStructInfo),
		nullptr, 0);
PRAGMA_ENABLE_DEPRECATION_WARNINGS
