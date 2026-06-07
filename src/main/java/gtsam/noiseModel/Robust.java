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

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Robust(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Robust> Create(
            shared_ptr<? extends gtsam.noiseModel.mEstimator.Base> robust,
            shared_ptr<? extends gtsam.noiseModel.Base> noise) throws Throwable {
        return new shared_ptr<>(
                (MemorySegment) FF.noiseModel_Robust_Create.h.invokeExact(robust.ptr, noise.ptr),
                Robust::new);
    }

    @Override
    public Vector sigmas() throws Throwable {
        return new Vector((MemorySegment) FF.noiseModel_Robust_sigmas.h.invokeExact(ptr));
    }

    @Override
    public Vector whiten(Vector v) throws Throwable {
        return new Vector((MemorySegment) FF.noiseModel_Robust_whiten.h.invokeExact(ptr, v.ptr));
    }

    @Override
    public Matrix Whiten(Matrix H) throws Throwable {
        return new Matrix((MemorySegment) FF.noiseModel_Robust_Whiten.h.invokeExact(ptr, H.ptr));
    }

    @Override
    public Vector unwhiten(Vector v) throws Throwable {
        return new Vector((MemorySegment) FF.noiseModel_Robust_unwhiten.h.invokeExact(ptr, v.ptr));
    }

    public shared_ptr<gtsam.noiseModel.mEstimator.Base> robust() throws Throwable {
        return new shared_ptr<gtsam.noiseModel.mEstimator.Base>(
                (MemorySegment) FF.noiseModel_Robust_robust.h.invokeExact(ptr),
                gtsam.noiseModel.mEstimator.Base::new);
    }

    public void WhitenSystem(Vector b) throws Throwable {
        FF.noiseModel_Robust_WhitenSystemVector.h.invokeExact(ptr, b.ptr);
    }

}
