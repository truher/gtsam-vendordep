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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    ISAM2Result(MemorySegment pointer) {
        super(pointer, FF.ISAM2Result_delete.h);
    }

    public FactorIndices newFactorsIndices() throws Throwable {
        return new FactorIndices((MemorySegment) FF.ISAM2Result_newFactorsIndices.h.invokeExact(ptr));
    }

}
