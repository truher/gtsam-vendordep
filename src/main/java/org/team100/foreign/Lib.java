package org.team100.foreign;

import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

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
    public static final SymbolLookup lib = SymbolLookup.libraryLookup("libgtsamwrapper.so", arena);
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
