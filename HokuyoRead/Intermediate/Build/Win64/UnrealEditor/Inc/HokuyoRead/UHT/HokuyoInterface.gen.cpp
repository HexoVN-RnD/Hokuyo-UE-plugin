// Copyright Epic Games, Inc. All Rights Reserved.
/*===========================================================================
	Generated code exported from UnrealHeaderTool.
	DO NOT modify this manually! Edit the corresponding .h files instead!
===========================================================================*/

#include "UObject/GeneratedCppIncludes.h"
#include "../../Public/HokuyoInterface.h"
PRAGMA_DISABLE_DEPRECATION_WARNINGS
void EmptyLinkFunctionForGeneratedCodeHokuyoInterface() {}
// Cross Module References
	COREUOBJECT_API UClass* Z_Construct_UClass_UInterface();
	COREUOBJECT_API UScriptStruct* Z_Construct_UScriptStruct_FVector2D();
	HOKUYOREAD_API UClass* Z_Construct_UClass_UHokuyoInterface();
	HOKUYOREAD_API UClass* Z_Construct_UClass_UHokuyoInterface_NoRegister();
	UPackage* Z_Construct_UPackage__Script_HokuyoRead();
// End Cross Module References
	DEFINE_FUNCTION(IHokuyoInterface::execProcessCentroids)
	{
		P_GET_TARRAY_REF(FVector2D,Z_Param_Out_Centroids);
		P_FINISH;
		P_NATIVE_BEGIN;
		P_THIS->ProcessCentroids_Implementation(Z_Param_Out_Centroids);
		P_NATIVE_END;
	}
	struct HokuyoInterface_eventProcessCentroids_Parms
	{
		TArray<FVector2D> Centroids;
	};
	void IHokuyoInterface::ProcessCentroids(TArray<FVector2D> const& Centroids)
	{
		check(0 && "Do not directly call Event functions in Interfaces. Call Execute_ProcessCentroids instead.");
	}
	void UHokuyoInterface::StaticRegisterNativesUHokuyoInterface()
	{
		UClass* Class = UHokuyoInterface::StaticClass();
		static const FNameNativePtrPair Funcs[] = {
			{ "ProcessCentroids", &IHokuyoInterface::execProcessCentroids },
		};
		FNativeFunctionRegistrar::RegisterFunctions(Class, Funcs, UE_ARRAY_COUNT(Funcs));
	}
	struct Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics
	{
		static const UECodeGen_Private::FStructPropertyParams NewProp_Centroids_Inner;
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam NewProp_Centroids_MetaData[];
#endif
		static const UECodeGen_Private::FArrayPropertyParams NewProp_Centroids;
		static const UECodeGen_Private::FPropertyParamsBase* const PropPointers[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Function_MetaDataParams[];
#endif
		static const UECodeGen_Private::FFunctionParams FuncParams;
	};
	const UECodeGen_Private::FStructPropertyParams Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::NewProp_Centroids_Inner = { "Centroids", nullptr, (EPropertyFlags)0x0000000000000000, UECodeGen_Private::EPropertyGenFlags::Struct, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, 0, Z_Construct_UScriptStruct_FVector2D, METADATA_PARAMS(nullptr, 0) };
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::NewProp_Centroids_MetaData[] = {
		{ "NativeConst", "" },
	};
#endif
	const UECodeGen_Private::FArrayPropertyParams Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::NewProp_Centroids = { "Centroids", nullptr, (EPropertyFlags)0x0010000008000182, UECodeGen_Private::EPropertyGenFlags::Array, RF_Public|RF_Transient|RF_MarkAsNative, 1, nullptr, nullptr, STRUCT_OFFSET(HokuyoInterface_eventProcessCentroids_Parms, Centroids), EArrayPropertyFlags::None, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::NewProp_Centroids_MetaData, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::NewProp_Centroids_MetaData)) };
	const UECodeGen_Private::FPropertyParamsBase* const Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::PropPointers[] = {
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::NewProp_Centroids_Inner,
		(const UECodeGen_Private::FPropertyParamsBase*)&Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::NewProp_Centroids,
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::Function_MetaDataParams[] = {
		{ "Category", "Hokuyo | Interface" },
		{ "ModuleRelativePath", "Public/HokuyoInterface.h" },
	};
#endif
	const UECodeGen_Private::FFunctionParams Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::FuncParams = { (UObject*(*)())Z_Construct_UClass_UHokuyoInterface, nullptr, "ProcessCentroids", nullptr, nullptr, sizeof(HokuyoInterface_eventProcessCentroids_Parms), Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::PropPointers, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::PropPointers), RF_Public|RF_Transient|RF_MarkAsNative, (EFunctionFlags)0x0C420C00, 0, 0, METADATA_PARAMS(Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::Function_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::Function_MetaDataParams)) };
	UFunction* Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids()
	{
		static UFunction* ReturnFunction = nullptr;
		if (!ReturnFunction)
		{
			UECodeGen_Private::ConstructUFunction(&ReturnFunction, Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids_Statics::FuncParams);
		}
		return ReturnFunction;
	}
	IMPLEMENT_CLASS_NO_AUTO_REGISTRATION(UHokuyoInterface);
	UClass* Z_Construct_UClass_UHokuyoInterface_NoRegister()
	{
		return UHokuyoInterface::StaticClass();
	}
	struct Z_Construct_UClass_UHokuyoInterface_Statics
	{
		static UObject* (*const DependentSingletons[])();
		static const FClassFunctionLinkInfo FuncInfo[];
#if WITH_METADATA
		static const UECodeGen_Private::FMetaDataPairParam Class_MetaDataParams[];
#endif
		static const FCppClassTypeInfoStatic StaticCppClassTypeInfo;
		static const UECodeGen_Private::FClassParams ClassParams;
	};
	UObject* (*const Z_Construct_UClass_UHokuyoInterface_Statics::DependentSingletons[])() = {
		(UObject* (*)())Z_Construct_UClass_UInterface,
		(UObject* (*)())Z_Construct_UPackage__Script_HokuyoRead,
	};
	const FClassFunctionLinkInfo Z_Construct_UClass_UHokuyoInterface_Statics::FuncInfo[] = {
		{ &Z_Construct_UFunction_UHokuyoInterface_ProcessCentroids, "ProcessCentroids" }, // 3911741641
	};
#if WITH_METADATA
	const UECodeGen_Private::FMetaDataPairParam Z_Construct_UClass_UHokuyoInterface_Statics::Class_MetaDataParams[] = {
		{ "ModuleRelativePath", "Public/HokuyoInterface.h" },
	};
#endif
	const FCppClassTypeInfoStatic Z_Construct_UClass_UHokuyoInterface_Statics::StaticCppClassTypeInfo = {
		TCppClassTypeTraits<IHokuyoInterface>::IsAbstract,
	};
	const UECodeGen_Private::FClassParams Z_Construct_UClass_UHokuyoInterface_Statics::ClassParams = {
		&UHokuyoInterface::StaticClass,
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
		0x000840A1u,
		METADATA_PARAMS(Z_Construct_UClass_UHokuyoInterface_Statics::Class_MetaDataParams, UE_ARRAY_COUNT(Z_Construct_UClass_UHokuyoInterface_Statics::Class_MetaDataParams))
	};
	UClass* Z_Construct_UClass_UHokuyoInterface()
	{
		if (!Z_Registration_Info_UClass_UHokuyoInterface.OuterSingleton)
		{
			UECodeGen_Private::ConstructUClass(Z_Registration_Info_UClass_UHokuyoInterface.OuterSingleton, Z_Construct_UClass_UHokuyoInterface_Statics::ClassParams);
		}
		return Z_Registration_Info_UClass_UHokuyoInterface.OuterSingleton;
	}
	template<> HOKUYOREAD_API UClass* StaticClass<UHokuyoInterface>()
	{
		return UHokuyoInterface::StaticClass();
	}
	UHokuyoInterface::UHokuyoInterface(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer) {}
	DEFINE_VTABLE_PTR_HELPER_CTOR(UHokuyoInterface);
	UHokuyoInterface::~UHokuyoInterface() {}
	static FName NAME_UHokuyoInterface_ProcessCentroids = FName(TEXT("ProcessCentroids"));
	void IHokuyoInterface::Execute_ProcessCentroids(UObject* O, TArray<FVector2D> const& Centroids)
	{
		check(O != NULL);
		check(O->GetClass()->ImplementsInterface(UHokuyoInterface::StaticClass()));
		HokuyoInterface_eventProcessCentroids_Parms Parms;
		UFunction* const Func = O->FindFunction(NAME_UHokuyoInterface_ProcessCentroids);
		if (Func)
		{
			Parms.Centroids=Centroids;
			O->ProcessEvent(Func, &Parms);
		}
		else if (auto I = (IHokuyoInterface*)(O->GetNativeInterfaceAddress(UHokuyoInterface::StaticClass())))
		{
			I->ProcessCentroids_Implementation(Centroids);
		}
	}
	struct Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_Statics
	{
		static const FClassRegisterCompiledInInfo ClassInfo[];
	};
	const FClassRegisterCompiledInInfo Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_Statics::ClassInfo[] = {
		{ Z_Construct_UClass_UHokuyoInterface, UHokuyoInterface::StaticClass, TEXT("UHokuyoInterface"), &Z_Registration_Info_UClass_UHokuyoInterface, CONSTRUCT_RELOAD_VERSION_INFO(FClassReloadVersionInfo, sizeof(UHokuyoInterface), 3871923590U) },
	};
	static FRegisterCompiledInInfo Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_1805737341(TEXT("/Script/HokuyoRead"),
		Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_Statics::ClassInfo, UE_ARRAY_COUNT(Z_CompiledInDeferFile_FID_HostProject_Plugins_HokuyoRead_Source_HokuyoRead_Public_HokuyoInterface_h_Statics::ClassInfo),
		nullptr, 0,
		nullptr, 0);
PRAGMA_ENABLE_DEPRECATION_WARNINGS
