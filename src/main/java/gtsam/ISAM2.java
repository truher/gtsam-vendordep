package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class ISAM2 extends ForeignObject {
    public enum FF {
        ISAM2(ADDRESS, ADDRESS),
        ISAM2_delete(null, ADDRESS),
        ISAM2_update(ADDRESS, ADDRESS, ADDRESS, ADDRESS),
        ISAM2_calculateEstimate(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    ISAM2(MemorySegment pointer) {
        super(pointer, null);
    }

    ISAM2(MemorySegment p, MethodHandle d) {
        super(p, d);
    }

    public ISAM2(ISAM2Params params) throws Throwable {
        this((MemorySegment) FF.ISAM2.h.invokeExact(params.ptr),
                FF.ISAM2_delete.h);
    }

    public ISAM2Result update(NonlinearFactorGraph graph, Values values) throws Throwable {
        return new ISAM2Result(
                (MemorySegment) FF.ISAM2_update.h.invokeExact(
                        ptr, graph.ptr, values.ptr));
    }

    public Values calculateEstimate() throws Throwable {
        return Values.owned(
                (MemorySegment) FF.ISAM2_calculateEstimate.h.invokeExact(ptr));
    }

}
