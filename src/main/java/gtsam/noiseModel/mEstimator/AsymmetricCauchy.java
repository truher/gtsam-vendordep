package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class AsymmetricCauchy extends Base {

    public enum FF {
        noiseModel_mEstimator_AsymmetricCauchy_Create(ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public AsymmetricCauchy(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<AsymmetricCauchy> Create(double k) throws Throwable {
        return new shared_ptr<AsymmetricCauchy>(
                (MemorySegment) FF.noiseModel_mEstimator_AsymmetricCauchy_Create.h.invokeExact(k),
                AsymmetricCauchy::new);
    }
}
