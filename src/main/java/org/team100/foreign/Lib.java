package org.team100.foreign;

import java.io.IOException;
import java.io.InputStream;
import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.StructLayout;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HexFormat;

/**
 * Requires the correct java version,
 * so I set it to 25 in build.gradle.
 */
public class Lib {
    public static final Arena arena = Arena.ofAuto();

    public static final SymbolLookup lib = load();

    private static void readbytes(Path path) {
        if (!Files.exists(path))
            return;
        byte[] bytes = new byte[20];
        try (InputStream is = Files.newInputStream(path)) {
            if (is.read(bytes) < 20)
                return;
            String hexUpper = HexFormat.of().withUpperCase().formatHex(bytes);
            System.out.println(path.toAbsolutePath());
            System.out.println(hexUpper);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    private static void info(Path p) {
        System.out.printf("TRYING TO LOAD %s\n", p.toAbsolutePath());
        System.out.printf("EXISTS? %b\n", Files.exists(p));
        readbytes(p);
    }

    private static Path libPath() {

        // Deployed on the Systemcore.
        Path p = Paths.get("frc/third-party/lib/libgtsamwrapper.so");
        info(p);
        if (Files.exists(p))
            return p;
        // For running unit tests.
        p = Paths.get(
                "build/libs/gtsamwrapper/shared/linuxx86-64/libgtsamwrapper.so");
        info(p);
        if (Files.exists(p))
            return p;
        // For Github Actions.
        p = Paths.get("build/libs/gtsamwrapper/shared/libgtsamwrapper.so");
        info(p);
        if (Files.exists(p))
            return p;
        // For running unit tests in MacOS.
        p = Paths.get(System.mapLibraryName("gtsamwrapper"));
        info(p);
        if (Files.exists(p))
            return p;
        System.out.println("Can't find the library.");
        return null;
    }

    private static SymbolLookup load() {
        // The vendordep library is unpacked somewhere the loader can find it.
        // Where does the name, "libgtsamwrapper.so" come from?
        // It is mentioned in build.gradle model.components
        // and build.gradle nativeUtils.privateExportsConfigs
        // and publish.gradle model.publishing.driverTaskList.
        // Where are we?
        Path path = Paths.get("");
        String cwd = path.toAbsolutePath().toString();
        System.out.println("===CWD: " + cwd);
        try {
            Files.list(path).forEach(System.out::println);
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.println("===shared===");
        Path p2 = Paths.get("build/libs/gtsamwrapper/shared/");
        try {
            Files.list(p2).forEach(System.out::println);
        } catch (IOException e) {
            e.printStackTrace();
        }
        System.out.flush();

        try {
            // In simulation on Desktop.
            // TODO: figure out where this file really is.
            return SymbolLookup.libraryLookup("libgtsamwrapper.so", arena);
        } catch (IllegalArgumentException ee) {
            System.out.printf("failed %s\n", ee.getMessage());
            ee.printStackTrace();
        }

        try {
            return SymbolLookup.libraryLookup(libPath().toAbsolutePath(), arena);
        } catch (IllegalArgumentException e) {
            System.out.printf("lib load failed %s\n", e.getMessage());
            e.printStackTrace();
            throw new RuntimeException(e);
        }

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
