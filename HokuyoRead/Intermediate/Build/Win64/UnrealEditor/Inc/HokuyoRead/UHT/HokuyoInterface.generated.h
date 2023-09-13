// Copyright Epic Games, Inc. All Rights Reserved.
/*===========================================================================
	Generated code exported from UnrealHeaderTool.
	DO NOT modify this manually! Edit the corresponding .h files instead!
===========================================================================*/

// IWYU pragma: private, include "HokuyoInterface.h"
#include "UObject/ObjectMacros.h"
#include "UObject/ScriptMacros.h"

PRAGMA_DISABLE_DEPRECATION_WARNINGS
#ifdef HOKUYOREAD_HokuyoInterface_generated_h
#error "HokuyoInterface.generated.h already included, missing '#pragma once' in HokuyoInterface.h"
#endif
#define HOKUYOREAD_HokuyoInterface_generated_h

#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_SPARSE_DATA
#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_RPC_WRAPPERS \
	virtual void ProcessCentroids_Implementation(TArray<FVector2D> const& Centroids) {}; \
 \
	DECLARE_FUNCTION(execProcessCentroids);


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_RPC_WRAPPERS_NO_PURE_DECLS \
	virtual void ProcessCentroids_Implementation(TArray<FVector2D> const& Centroids) {}; \
 \
	DECLARE_FUNCTION(execProcessCentroids);


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_ACCESSORS
#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_CALLBACK_WRAPPERS
#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_STANDARD_CONSTRUCTORS \
	/** Standard constructor, called after all reflected properties have been initialized */ \
	HOKUYOREAD_API UHokuyoInterface(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get()); \
	DEFINE_ABSTRACT_DEFAULT_OBJECT_INITIALIZER_CONSTRUCTOR_CALL(UHokuyoInterface) \
	DECLARE_VTABLE_PTR_HELPER_CTOR(HOKUYOREAD_API, UHokuyoInterface); \
	DEFINE_VTABLE_PTR_HELPER_CTOR_CALLER(UHokuyoInterface); \
private: \
	/** Private move- and copy-constructors, should never be used */ \
	HOKUYOREAD_API UHokuyoInterface(UHokuyoInterface&&); \
	HOKUYOREAD_API UHokuyoInterface(const UHokuyoInterface&); \
public: \
	HOKUYOREAD_API virtual ~UHokuyoInterface();


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_ENHANCED_CONSTRUCTORS \
	/** Standard constructor, called after all reflected properties have been initialized */ \
	HOKUYOREAD_API UHokuyoInterface(const FObjectInitializer& ObjectInitializer = FObjectInitializer::Get()); \
private: \
	/** Private move- and copy-constructors, should never be used */ \
	HOKUYOREAD_API UHokuyoInterface(UHokuyoInterface&&); \
	HOKUYOREAD_API UHokuyoInterface(const UHokuyoInterface&); \
public: \
	DECLARE_VTABLE_PTR_HELPER_CTOR(HOKUYOREAD_API, UHokuyoInterface); \
	DEFINE_VTABLE_PTR_HELPER_CTOR_CALLER(UHokuyoInterface); \
	DEFINE_ABSTRACT_DEFAULT_OBJECT_INITIALIZER_CONSTRUCTOR_CALL(UHokuyoInterface) \
	HOKUYOREAD_API virtual ~UHokuyoInterface();


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_GENERATED_UINTERFACE_BODY() \
private: \
	static void StaticRegisterNativesUHokuyoInterface(); \
	friend struct Z_Construct_UClass_UHokuyoInterface_Statics; \
public: \
	DECLARE_CLASS(UHokuyoInterface, UInterface, COMPILED_IN_FLAGS(CLASS_Abstract | CLASS_Interface), CASTCLASS_None, TEXT("/Script/HokuyoRead"), HOKUYOREAD_API) \
	DECLARE_SERIALIZER(UHokuyoInterface)


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_GENERATED_BODY_LEGACY \
		PRAGMA_DISABLE_DEPRECATION_WARNINGS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_GENERATED_UINTERFACE_BODY() \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_STANDARD_CONSTRUCTORS \
	PRAGMA_ENABLE_DEPRECATION_WARNINGS


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_GENERATED_BODY \
	PRAGMA_DISABLE_DEPRECATION_WARNINGS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_GENERATED_UINTERFACE_BODY() \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_ENHANCED_CONSTRUCTORS \
private: \
	PRAGMA_ENABLE_DEPRECATION_WARNINGS


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_INCLASS_IINTERFACE_NO_PURE_DECLS \
protected: \
	virtual ~IHokuyoInterface() {} \
public: \
	typedef UHokuyoInterface UClassType; \
	typedef IHokuyoInterface ThisClass; \
	static void Execute_ProcessCentroids(UObject* O, TArray<FVector2D> const& Centroids); \
	virtual UObject* _getUObject() const { return nullptr; }


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_INCLASS_IINTERFACE \
protected: \
	virtual ~IHokuyoInterface() {} \
public: \
	typedef UHokuyoInterface UClassType; \
	typedef IHokuyoInterface ThisClass; \
	static void Execute_ProcessCentroids(UObject* O, TArray<FVector2D> const& Centroids); \
	virtual UObject* _getUObject() const { return nullptr; }


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_10_PROLOG
#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_21_GENERATED_BODY_LEGACY \
PRAGMA_DISABLE_DEPRECATION_WARNINGS \
public: \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_SPARSE_DATA \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_RPC_WRAPPERS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_ACCESSORS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_CALLBACK_WRAPPERS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_INCLASS_IINTERFACE \
public: \
PRAGMA_ENABLE_DEPRECATION_WARNINGS


#define FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_21_GENERATED_BODY \
PRAGMA_DISABLE_DEPRECATION_WARNINGS \
public: \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_SPARSE_DATA \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_RPC_WRAPPERS_NO_PURE_DECLS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_ACCESSORS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_CALLBACK_WRAPPERS \
	FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_13_INCLASS_IINTERFACE_NO_PURE_DECLS \
private: \
PRAGMA_ENABLE_DEPRECATION_WARNINGS


template<> HOKUYOREAD_API UClass* StaticClass<class UHokuyoInterface>();

#undef CURRENT_FILE_ID
#define CURRENT_FILE_ID FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h


PRAGMA_ENABLE_DEPRECATION_WARNINGS
