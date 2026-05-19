package org.team100.foreign;

import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.nio.file.Paths;

/**
 * Requires the correct java version,
 * so I set it to 25 in build.gradle.
 */
public class Lib {
    public static final Arena arena = Arena.ofAuto();
    // The vendordep library is unpacked somewhere the loader can find it.
    // Where does the name, "libgtsamwrapper.so" come from?
    // it is mentioned in build.gradle model.components
    // and build.gradle nativeUtils.privateExportsConfigs
    // and publish.gradle model.publishing.driverTaskList.
    public static final SymbolLookup lib;
    static {
        SymbolLookup slib;
        // Where are we?
        String cwd = Paths.get("").toAbsolutePath().toString();
        System.out.println("CWD: " + cwd);
        System.out.flush();
        // Where is the library?
        try {
            // Systemcore location.
            slib = SymbolLookup.libraryLookup("frc/third-party/lib/libgtsamwrapper.so", arena);
        } catch (IllegalArgumentException e) {
            // Desktop location.
            try {
                slib = SymbolLookup.libraryLookup("libgtsamwrapper.so", arena);
            } catch (IllegalArgumentException ee) {
                // Test location.
                slib = SymbolLookup.libraryLookup("build/libs/gtsamwrapper/shared/linuxx86-64/debug/libgtsamwrapper.so",
                        arena);
            }
        }
        lib = slib;
    }

    public static final Linker linker = Linker.nativeLinker();

    public static MethodHandle down(
            String name, ValueLayout returnType, ValueLayout... parameterTypes) {
        return linker.downcallHandle(
                lib.findOrThrow(name),
                FunctionDescriptor.of(returnType, parameterTypes));
    }

    public static MethodHandle downVoid(
            String name, ValueLayout... parameterTypes) {
        return linker.downcallHandle(
                lib.findOrThrow(name),
                FunctionDescriptor.ofVoid(parameterTypes));
    }
}
