package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class Cauchy extends Base {

    public enum FF {
        noiseModel_mEstimator_Cauchy_Create(ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Cauchy(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Cauchy> Create(double k) throws Throwable {
        return new shared_ptr<Cauchy>(
                (MemorySegment) FF.noiseModel_mEstimator_Cauchy_Create.h.invokeExact(k),
                Cauchy::new);
    }
}
