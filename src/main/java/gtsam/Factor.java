package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class Factor extends ForeignObject {
    public enum FF {
        Factor_keys(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    Factor(MemorySegment p) {
        super(p, null);
    }

    public KeyVector keys() throws Throwable {
        return new KeyVector((MemorySegment) FF.Factor_keys.h.invokeExact(ptr));
    }

}
