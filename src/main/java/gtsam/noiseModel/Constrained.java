package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Vector;
import gtsam.shared_ptr;

public class Constrained extends Diagonal {
    public enum FF {
        noiseModel_Constrained_MixedSigmasVectorVector(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Constrained_MixedSigmasVector(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Constrained(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Constrained> MixedSigmas(Vector mu, Vector sigmas) throws Throwable {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_MixedSigmasVectorVector.h.invokeExact(mu.ptr, sigmas.ptr),
                Constrained::new);
    }

    public static shared_ptr<Constrained> MixedSigmas(Vector sigmas) throws Throwable {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_MixedSigmasVector.h.invokeExact(sigmas.ptr),
                Constrained::new);
    }

}
