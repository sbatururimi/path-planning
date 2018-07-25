# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.path_planning.Debug:
/Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/Debug/path_planning:
	/bin/rm -f /Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/Debug/path_planning


PostBuild.path_planning.Release:
/Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/Release/path_planning:
	/bin/rm -f /Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/Release/path_planning


PostBuild.path_planning.MinSizeRel:
/Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/MinSizeRel/path_planning:
	/bin/rm -f /Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/MinSizeRel/path_planning


PostBuild.path_planning.RelWithDebInfo:
/Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/RelWithDebInfo/path_planning:
	/bin/rm -f /Users/virt/Developer/self-driving\ cars/batururimi/Term3/path-planning/ide_profiles/xcode/RelWithDebInfo/path_planning




# For each target create a dummy ruleso the target does not have to exist
