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

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public DCS(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<DCS> Create(double k) {
        return new shared_ptr<DCS>(
                (MemorySegment) FF.noiseModel_mEstimator_DCS_Create.invoke(k),
                DCS::new);
    }

    public static shared_ptr<DCS> CreateScalar(double k) {
        return new shared_ptr<DCS>(
                (MemorySegment) FF.noiseModel_mEstimator_DCS_CreateScalar.invoke(k),
                DCS::new);
    }
}
