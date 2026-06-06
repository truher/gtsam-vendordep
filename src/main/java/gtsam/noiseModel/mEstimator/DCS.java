package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class DCS extends Base {

    public enum FF {
        noiseModel_mEstimator_DCS_Create(ADDRESS, JAVA_DOUBLE),
        noiseModel_mEstimator_DCS_CreateScalar(ADDRESS, JAVA_DOUBLE);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public DCS(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<DCS> Create(double k) throws Throwable {
        return new shared_ptr<DCS>(
                (MemorySegment) FF.noiseModel_mEstimator_DCS_Create.h.invokeExact(k),
                DCS::new);
    }
        public static shared_ptr<DCS> CreateScalar(double k) throws Throwable {
        return new shared_ptr<DCS>(
                (MemorySegment) FF.noiseModel_mEstimator_DCS_CreateScalar.h.invokeExact(k),
                DCS::new);
    }
}
