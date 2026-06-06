package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class Fair extends Base {
    public enum FF {
        noiseModel_mEstimator_Fair_Create(ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Fair(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Fair> Create(double k) throws Throwable {
        return new shared_ptr<Fair>(
                (MemorySegment) FF.noiseModel_mEstimator_Fair_Create.h.invokeExact(k),
                Fair::new);
    }
}
