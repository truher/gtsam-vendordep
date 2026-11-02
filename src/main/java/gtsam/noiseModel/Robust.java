package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Matrix;
import gtsam.Vector;
import gtsam.shared_ptr;

public class Robust extends Base {
    public enum FF {
        noiseModel_Robust_Create(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Robust_sigmas(ADDRESS, ADDRESS),
        noiseModel_Robust_whiten(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Robust_Whiten(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Robust_unwhiten(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Robust_robust(ADDRESS, ADDRESS),
        noiseModel_Robust_WhitenSystemVector(null, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Robust(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Robust> Create(
            shared_ptr<? extends gtsam.noiseModel.mEstimator.Base> robust,
            shared_ptr<? extends gtsam.noiseModel.Base> noise) {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Robust_Create.invoke(robust.ptr, noise.ptr),
                Robust::new);
    }

    @Override
    public Vector sigmas() {
        return new Vector((MemorySegment) FF.noiseModel_Robust_sigmas.invoke(ptr));
    }

    @Override
    public Vector whiten(Vector v) {
        return new Vector((MemorySegment) FF.noiseModel_Robust_whiten.invoke(ptr, v.ptr));
    }

    @Override
    public Matrix Whiten(Matrix H) {
        return new Matrix((MemorySegment) FF.noiseModel_Robust_Whiten.invoke(ptr, H.ptr));
    }

    @Override
    public Vector unwhiten(Vector v) {
        return new Vector((MemorySegment) FF.noiseModel_Robust_unwhiten.invoke(ptr, v.ptr));
    }

    public shared_ptr<gtsam.noiseModel.mEstimator.Base> robust() {
        return new shared_ptr<gtsam.noiseModel.mEstimator.Base>(
                (MemorySegment) FF.noiseModel_Robust_robust.invoke(ptr),
                gtsam.noiseModel.mEstimator.Base::new);
    }

    public void WhitenSystem(Vector b) {
        FF.noiseModel_Robust_WhitenSystemVector.invoke(ptr, b.ptr);
    }

}
