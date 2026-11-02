package gtsam.noiseModel.mEstimator;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.shared_ptr;

public class TruncatedLeastSquares extends Base {

    public enum FF {
        noiseModel_mEstimator_TruncatedLeastSquares_Create(ADDRESS, JAVA_DOUBLE),
        noiseModel_mEstimator_TruncatedLeastSquares_CreateScalar(ADDRESS, JAVA_DOUBLE);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public TruncatedLeastSquares(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<TruncatedLeastSquares> Create(double k) {
        return new shared_ptr<TruncatedLeastSquares>(
                (MemorySegment) FF.noiseModel_mEstimator_TruncatedLeastSquares_Create.invoke(k),
                TruncatedLeastSquares::new);
    }

    public static shared_ptr<TruncatedLeastSquares> CreateScalar(double k) {
        return new shared_ptr<TruncatedLeastSquares>(
                (MemorySegment) FF.noiseModel_mEstimator_TruncatedLeastSquares_CreateScalar.invoke(k),
                TruncatedLeastSquares::new);
    }
}
