// Some copyright should be here...

using UnrealBuildTool;
using System.IO;

public class HokuyoRead : ModuleRules
{
	public HokuyoRead(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicIncludePaths.AddRange(
			new string[] {
		Path.Combine(ModuleDirectory, "include", "c"),
		Path.Combine(ModuleDirectory, "include", "cpp")
			}
		);

		PublicAdditionalLibraries.AddRange(
			new string[] {
		Path.Combine(ModuleDirectory, "lib", "urg.lib")
			}
		);
		
		PrivateIncludePaths.AddRange(
			new string[] {
				// ... add other private include paths required here ...
			}
			);


		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				// ... add other public dependencies that you statically link with here ...
			}
			);


		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				// ... add private dependencies that you statically link with here ...	
			}
			);


		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
