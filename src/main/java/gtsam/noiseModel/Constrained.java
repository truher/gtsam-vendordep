package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Vector;
import gtsam.shared_ptr;

public class Constrained extends Diagonal {
    public enum FF {
        noiseModel_Constrained_MixedSigmasDoubleVector(ADDRESS, JAVA_DOUBLE, ADDRESS),
        noiseModel_Constrained_MixedSigmasVectorVector(ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Constrained_MixedSigmasVector(ADDRESS, ADDRESS),
        noiseModel_Constrained_AllInt(ADDRESS, JAVA_INT),
        noiseModel_Constrained_AllIntDouble(ADDRESS, JAVA_INT, JAVA_DOUBLE),
        noiseModel_Constrained_AllIntVector(ADDRESS, JAVA_INT, ADDRESS),
        noiseModel_Constrained_mu(ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Constrained(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Constrained> MixedSigmas(double mu, Vector sigmas) throws Throwable {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_MixedSigmasDoubleVector.h.invokeExact(mu, sigmas.ptr),
                Constrained::new);
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

    public static shared_ptr<Constrained> All(int dim) throws Throwable {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_AllInt.h.invokeExact(dim),
                Constrained::new);
    }

    public static shared_ptr<Constrained> All(int dim, double mu) throws Throwable {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_AllIntDouble.h.invokeExact(dim, mu),
                Constrained::new);
    }

    public static shared_ptr<Constrained> All(int dim, Vector mu) throws Throwable {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_AllIntVector.h.invokeExact(dim, mu.ptr),
                Constrained::new);
    }

    public Vector mu() throws Throwable {
        return new Vector((MemorySegment) FF.noiseModel_Constrained_mu.h.invokeExact(ptr));
    }

}
