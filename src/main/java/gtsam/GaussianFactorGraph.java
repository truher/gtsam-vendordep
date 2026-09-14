package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class GaussianFactorGraph extends ForeignObject {
    public enum FF {
        GaussianFactorGraph_optimize(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    GaussianFactorGraph(MemorySegment pointer) {
        super(pointer, null);
    }

    public VectorValues optimize() throws Throwable {
        return new VectorValues((MemorySegment) FF.GaussianFactorGraph_optimize.h.invokeExact(ptr));
    }

}
