package org.team100.foreign;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.lang.foreign.Arena;
import java.lang.foreign.FunctionDescriptor;
import java.lang.foreign.Linker;
import java.lang.foreign.StructLayout;
import java.lang.foreign.SymbolLookup;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.nio.charset.StandardCharsets;
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

    private static void otool(Path path) {
        try {
            if (!Files.exists(path))
                return;
            String[] command = { "otool", "-L", path.toAbsolutePath().toString() };
            ProcessBuilder builder = new ProcessBuilder(command);
            builder.redirectErrorStream(true);
            Process process = builder.start();
            try (BufferedReader reader = new BufferedReader(
                    new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    System.out.println(line);
                }
            }
            int exitCode = process.waitFor();
            System.out.println("Exited with code: " + exitCode);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    private static void readelf(Path path) {
        try {
            if (!Files.exists(path))
                return;
            String[] command = { "readelf", "-d", "-l", path.toAbsolutePath().toString() };
            ProcessBuilder builder = new ProcessBuilder(command);
            builder.redirectErrorStream(true);
            Process process = builder.start();
            try (BufferedReader reader = new BufferedReader(
                    new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    System.out.println(line);
                }
            }
            int exitCode = process.waitFor();
            System.out.println("Exited with code: " + exitCode);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

    private static void ldd(Path path) {
        try {
            if (!Files.exists(path))
                return;
            String[] command = { "ldd", path.toAbsolutePath().toString() };
            ProcessBuilder builder = new ProcessBuilder(command);
            builder.redirectErrorStream(true);
            Process process = builder.start();
            try (BufferedReader reader = new BufferedReader(
                    new InputStreamReader(process.getInputStream(), StandardCharsets.UTF_8))) {
                String line;
                while ((line = reader.readLine()) != null) {
                    System.out.println(line);
                }
            }
            int exitCode = process.waitFor();
            System.out.println("Exited with code: " + exitCode);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

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
        System.out.println("READBYTES");
        readbytes(p);
        System.out.println("LDD");
        ldd(p);
        System.out.println("READELF");
        readelf(p);
        System.out.println("OTOOL");
        otool(p);
    }

    private static Path libPath() {
        // For Systemcore.
        // Path p = Paths.get("frc/third-party/lib/libgtsamwrapper.so");
        // Joel found this path on 11/13/26.
        Path p = Paths.get("wpilib/third-party/lib/libgtsamwrapper.so");
        info(p);
        if (Files.exists(p))
            return p;

        // For running unit tests on Linux.
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

        // For running unit tests on MacOS.
        p = Paths.get(System.mapLibraryName("gtsamwrapper"));
        info(p);
        if (Files.exists(p))
            return p;

        // For Github Actions on MacOS.
        p = Paths.get(
                "/Users/runner/work/gtsam-vendordep/gtsam-vendordep/build/libs/gtsamwrapper/shared/osxuniversal/libgtsamwrapper.dylib");
        info(p);
        if (Files.exists(p))
            return p;

        // For Github Actions on Windows.
        p = Paths.get("build/libs/gtsamwrapper/shared/windowsx86-64/gtsamwrapper.dll");
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

        System.out.println("===build===");
        Path p3 = Paths.get("build");
        try {
            Files.list(p3).forEach(System.out::println);
        } catch (IOException e) {
            e.printStackTrace();
        }

        System.out.flush();

        // try {
        // // In simulation on Desktop.
        // // TODO: figure out where this file really is.
        // return SymbolLookup.libraryLookup("libgtsamwrapper.so", arena);
        // } catch (IllegalArgumentException ee) {
        // // System.out.printf("failed %s\n", ee.getMessage());
        // // ee.printStackTrace();
        // }

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
