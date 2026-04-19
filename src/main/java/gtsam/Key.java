package gtsam;

import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.FunctionDescriptor;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/** in gtsam a key is a typedef for uint64. */
public class Key {

    private static final MethodHandle symbol_shorthand_X = Lib.linker.downcallHandle(
            Lib.lib.findOrThrow("symbol_shorthand_X"),
            FunctionDescriptor.of(JAVA_LONG, JAVA_LONG));

    public final long j;

    public Key(long _j) {
        j = _j;
    }

    public static Key X(long j) throws Throwable {
        return new Key((long) symbol_shorthand_X.invokeExact(j));
    }

}
