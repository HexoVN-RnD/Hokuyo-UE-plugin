// Copyright Epic Games, Inc. All Rights Reserved.
/*===========================================================================
	Generated code exported from UnrealHeaderTool.
	DO NOT modify this manually! Edit the corresponding .h files instead!
===========================================================================*/

// IWYU pragma: private, include "HokuyoReadBPLibrary.h"
#include "UObject/ObjectMacros.h"
#include "UObject/ScriptMacros.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS
struct FUrgWrapper;
#ifdef HOKUYOREAD_HokuyoReadBPLibrary_generated_h
#error "HokuyoReadBPLibrary.generated.h already included, missing '#pragma once' in HokuyoReadBPLibrary.h"
#endif
#define HOKUYOREAD_HokuyoReadBPLibrary_generated_h

#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_19_GENERATED_BODY \
	friend struct Z_Construct_UScriptStruct_FUrgWrapper_Statics; \
	HOKUYOREAD_API static class UScriptStruct* StaticStruct();


template<> HOKUYOREAD_API UScriptStruct* StaticStruct<struct FUrgWrapper>();

#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_SPARSE_DATA
#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_RPC_WRAPPERS \
 \
	DECLARE_FUNCTION(exechokuyoTest); \
	DECLARE_FUNCTION(exectestFunction); \
	DECLARE_FUNCTION(execsetScanningDegree); \
	DECLARE_FUNCTION(execcloseConnectionifFalse); \
	DECLARE_FUNCTION(execcloseConnection); \
	DECLARE_FUNCTION(execopenConnection); \
	DECLARE_FUNCTION(execgetDistanceNonBlocking); \
	DECLARE_FUNCTION(execgetDistance); \
	DECLARE_FUNCTION(execClearDBSCANCentroids); \
	DECLARE_FUNCTION(execClearKMeansCentroids); \
	DECLARE_FUNCTION(execGetDBSCANCentroids); \
	DECLARE_FUNCTION(execGetKMeansCentroids); \
	DECLARE_FUNCTION(execCalibrateCentroids);


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_RPC_WRAPPERS_NO_PURE_DECLS \
 \
	DECLARE_FUNCTION(exechokuyoTest); \
	DECLARE_FUNCTION(exectestFunction); \
	DECLARE_FUNCTION(execsetScanningDegree); \
	DECLARE_FUNCTION(execcloseConnectionifFalse); \
	DECLARE_FUNCTION(execcloseConnection); \
	DECLARE_FUNCTION(execopenConnection); \
	DECLARE_FUNCTION(execgetDistanceNonBlocking); \
	DECLARE_FUNCTION(execgetDistance); \
	DECLARE_FUNCTION(execClearDBSCANCentroids); \
	DECLARE_FUNCTION(execClearKMeansCentroids); \
	DECLARE_FUNCTION(execGetDBSCANCentroids); \
	DECLARE_FUNCTION(execGetKMeansCentroids); \
	DECLARE_FUNCTION(execCalibrateCentroids);


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_ACCESSORS
#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_INCLASS_NO_PURE_DECLS \
private: \
	static void StaticRegisterNativesUHokuyoReadBPLibrary(); \
	friend struct Z_Construct_UClass_UHokuyoReadBPLibrary_Statics; \
public: \
	DECLARE_CLASS(UHokuyoReadBPLibrary, UBlueprintFunctionLibrary, COMPILED_IN_FLAGS(0), CASTCLASS_None, TEXT("/Script/HokuyoRead"), NO_API) \
	DECLARE_SERIALIZER(UHokuyoReadBPLibrary)


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_INCLASS \
private: \
	static void StaticRegisterNativesUHokuyoReadBPLibrary(); \
	friend struct Z_Construct_UClass_UHokuyoReadBPLibrary_Statics; \
public: \
	DECLARE_CLASS(UHokuyoReadBPLibrary, UBlueprintFunctionLibrary, COMPILED_IN_FLAGS(0), CASTCLASS_None, TEXT("/Script/HokuyoRead"), NO_API) \
	DECLARE_SERIALIZER(UHokuyoReadBPLibrary)


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_STANDARD_CONSTRUCTORS \
	/** Standard constructor, called after all reflected properties have been initialized */ \
	NO_API UHokuyoReadBPLibrary(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get()); \
	DEFINE_DEFAULT_OBJECT_INITIALIZER_CONSTRUCTOR_CALL(UHokuyoReadBPLibrary) \
	DECLARE_VTABLE_PTR_HELPER_CTOR(NO_API, UHokuyoReadBPLibrary); \
	DEFINE_VTABLE_PTR_HELPER_CTOR_CALLER(UHokuyoReadBPLibrary); \
private: \
	/** Private move- and copy-constructors, should never be used */ \
	NO_API UHokuyoReadBPLibrary(UHokuyoReadBPLibrary&&); \
	NO_API UHokuyoReadBPLibrary(const UHokuyoReadBPLibrary&); \
public: \
	NO_API virtual ~UHokuyoReadBPLibrary();


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_ENHANCED_CONSTRUCTORS \
	/** Standard constructor, called after all reflected properties have been initialized */ \
	NO_API UHokuyoReadBPLibrary(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get()); \
private: \
	/** Private move- and copy-constructors, should never be used */ \
	NO_API UHokuyoReadBPLibrary(UHokuyoReadBPLibrary&&); \
	NO_API UHokuyoReadBPLibrary(const UHokuyoReadBPLibrary&); \
public: \
	DECLARE_VTABLE_PTR_HELPER_CTOR(NO_API, UHokuyoReadBPLibrary); \
	DEFINE_VTABLE_PTR_HELPER_CTOR_CALLER(UHokuyoReadBPLibrary); \
	DEFINE_DEFAULT_OBJECT_INITIALIZER_CONSTRUCTOR_CALL(UHokuyoReadBPLibrary) \
	NO_API virtual ~UHokuyoReadBPLibrary();


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_25_PROLOG
#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_GENERATED_BODY_LEGACY \
PRAGMA_DISABLE_DEPRECATION_WARNINGS \
public: \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_SPARSE_DATA \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_RPC_WRAPPERS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_ACCESSORS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_INCLASS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_STANDARD_CONSTRUCTORS \
public: \
PRAGMA_ENABLE_DEPRECATION_WARNINGS


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_GENERATED_BODY \
PRAGMA_DISABLE_DEPRECATION_WARNINGS \
public: \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_SPARSE_DATA \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_RPC_WRAPPERS_NO_PURE_DECLS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_ACCESSORS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_INCLASS_NO_PURE_DECLS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h_28_ENHANCED_CONSTRUCTORS \
private: \
PRAGMA_ENABLE_DEPRECATION_WARNINGS


template<> HOKUYOREAD_API UClass* StaticClass<class UHokuyoReadBPLibrary>();

#undef CURRENT_FILE_ID
#define CURRENT_FILE_ID FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoReadBPLibrary_h


PRAGMA_ENABLE_DEPRECATION_WARNINGS
