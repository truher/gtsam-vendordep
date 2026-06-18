package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class ISAM2Params extends ForeignObject {

    public enum FF {
        ISAM2Params(ADDRESS),
        ISAM2Params_delete(null, ADDRESS),
        ISAM2Params_enableDetailedResults(null, ADDRESS, JAVA_BOOLEAN),
        ISAM2Params_findUnusedFactorSlots(null, ADDRESS, JAVA_BOOLEAN),
        ISAM2Params_relinearizeSkip(null, ADDRESS, JAVA_INT),
        ISAM2Params_relinearizeThreshold(null, ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    ISAM2Params(MemorySegment pointer) {
        super(pointer, FF.ISAM2Params_delete.h);
    }

    public ISAM2Params() throws Throwable {
        this((MemorySegment) FF.ISAM2Params.h.invokeExact());
    }

    public void enableDetailedResults(boolean val) throws Throwable {
        FF.ISAM2Params_enableDetailedResults.h.invokeExact(ptr, val);
    }

    public void findUnusedFactorSlots(boolean val) throws Throwable {
        FF.ISAM2Params_findUnusedFactorSlots.h.invokeExact(ptr, val);
    }

    public void relinearizeSkip(int val) throws Throwable {
        FF.ISAM2Params_relinearizeSkip.h.invokeExact(ptr, val);
    }

    public void relinearizeThreshold(double val) throws Throwable {
        FF.ISAM2Params_relinearizeThreshold.h.invokeExact(ptr, val);
    }
}
