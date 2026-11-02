package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class ISAM2Result extends ForeignObject {
    public enum FF {
        ISAM2Result_delete(null, ADDRESS),
        ISAM2Result_newFactorsIndices(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    ISAM2Result(MemorySegment pointer) {
        super(pointer, FF.ISAM2Result_delete.h);
    }

    public FactorIndices newFactorsIndices() {
        return new FactorIndices((MemorySegment) FF.ISAM2Result_newFactorsIndices.invoke(ptr));
    }

}
