package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public abstract class FactorGraph extends ForeignObject {
    public enum FF {
        FactorGraph_add_factorsNonlinearFactorGraph(ADDRESS, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    FactorGraph(MemorySegment pointer, MethodHandle deleter) {
        // abstract, deleted by subclass
        super(pointer, deleter);
    }

    public FactorIndices add_factors(NonlinearFactorGraph g) throws Throwable {
        return new FactorIndices(
                (MemorySegment) FF.FactorGraph_add_factorsNonlinearFactorGraph.h.invokeExact(
                        ptr, g.ptr));
    }

}
