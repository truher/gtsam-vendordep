package org.team100.foreign;

import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.StructLayout;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Requires the correct java version,
 * so I set it to 25 in build.gradle.
 */
public class Lib {
    public static final Arena arena = Arena.ofAuto();

    public static final SymbolLookup lib = load();

    private static SymbolLookup load() {
        // The vendordep library is unpacked somewhere the loader can find it.
        // Where does the name, "libgtsamwrapper.so" come from?
        // It is mentioned in build.gradle model.components
        // and build.gradle nativeUtils.privateExportsConfigs
        // and publish.gradle model.publishing.driverTaskList.
        // Where are we?
        String cwd = Paths.get("").toAbsolutePath().toString();
        System.out.println("CWD: " + cwd);
        System.out.flush();

        // Try a few locations for the library.
        try {
            // Deployed on the Systemcore.
            return SymbolLookup.libraryLookup("frc/third-party/lib/libgtsamwrapper.so", arena);
        } catch (IllegalArgumentException e) {
        }

        try {
            // In simulation on Desktop.
            return SymbolLookup.libraryLookup("libgtsamwrapper.so", arena);
        } catch (IllegalArgumentException ee) {
        }
        try {
            // For running unit tests.
            return SymbolLookup.libraryLookup(
                    "build/libs/gtsamwrapper/shared/linuxx86-64/libgtsamwrapper.so",
                    arena);
        } catch (IllegalArgumentException ee) {
        }
        try {
            // For Github Actions.
            return SymbolLookup.libraryLookup(
                    "build/libs/gtsamwrapper/shared/libgtsamwrapper.so",
                    arena);
        } catch (IllegalArgumentException ee) {
        }
        // For running unit tests in MacOS, it wants an absolute path, ignores CWD.
        Path libpath = Path.of(
                "/Users/runner/work/gtsam-vendordep/gtsam-vendordep/build/libs/gtsamwrapper/shared/osxuniversal/libgtsamwrapper.dylib")
                .toAbsolutePath();
        return SymbolLookup.libraryLookup(
                libpath,
                arena);
    }

    public static final Linker linker = Linker.nativeLinker();

    public static MethodHandle ff(Enum<?> fn, ValueLayout returnType, ValueLayout... parameterTypes) {
        return linker.downcallHandle(
                lib.findOrThrow(fn.name()),
                of(returnType, parameterTypes));
    }

    public static MethodHandle ff(String name, ValueLayout returnType, ValueLayout... parameterTypes) {
        return linker.downcallHandle(
                lib.findOrThrow(name),
                of(returnType, parameterTypes));
    }

    public static MethodHandle ff(Enum<?> fn, StructLayout returnType, ValueLayout... parameterTypes) {
        return linker.downcallHandle(
                lib.findOrThrow(fn.name()),
                of(returnType, parameterTypes));
    }

    public static FunctionDescriptor of(ValueLayout returnType, ValueLayout... parameterTypes) {
        if (returnType == null)
            return FunctionDescriptor.ofVoid(parameterTypes);
        return FunctionDescriptor.of(returnType, parameterTypes);
    }

    public static FunctionDescriptor of(StructLayout returnType, ValueLayout... parameterTypes) {
        if (returnType == null)
            return FunctionDescriptor.ofVoid(parameterTypes);
        return FunctionDescriptor.of(returnType, parameterTypes);
    }

    public static Object invoke(MethodHandle h, Object... args) {
        try {
            return h.invokeWithArguments(args);
        } catch (Throwable throwable) {
            throw new RuntimeException(throwable);
        }
    }
}
