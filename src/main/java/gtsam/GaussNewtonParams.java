package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class GaussNewtonParams extends ForeignObject {
    public enum FF {
        GaussNewtonParams(ADDRESS),
        GaussNewtonParams_delete(null, ADDRESS),
        GaussNewtonParams_relativeErrorTol(null, ADDRESS, JAVA_DOUBLE),
        GaussNewtonParams_maxIterations(null, ADDRESS, JAVA_INT);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    GaussNewtonParams(MemorySegment pointer) {
        super(pointer, FF.GaussNewtonParams_delete.h);
    }

    public GaussNewtonParams() {
        this((MemorySegment) FF.GaussNewtonParams.invoke());
    }

    public void relativeErrorTol(double tol) {
        FF.GaussNewtonParams_relativeErrorTol.invoke(ptr, tol);
    }

    public void maxIterations(int i) {

    }

}
