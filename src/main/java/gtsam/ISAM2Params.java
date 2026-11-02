package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class ISAM2Params extends ForeignObject {

    public enum FF {
        ISAM2Params(ADDRESS),
        ISAM2Params_delete(null, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    ISAM2Params(MemorySegment pointer) {
        super(pointer, FF.ISAM2Params_delete.h);
    }

    public ISAM2Params() {
        this((MemorySegment) FF.ISAM2Params.invoke());
    }

}
