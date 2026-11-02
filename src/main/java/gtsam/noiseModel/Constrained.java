package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Matrix;
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
        noiseModel_Constrained_mu(ADDRESS, ADDRESS),
        noiseModel_Constrained_informationFromA(ADDRESS, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Constrained(MemorySegment p) {
        super(p);
    }

    public static shared_ptr<Constrained> MixedSigmas(double mu, Vector sigmas) {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_MixedSigmasDoubleVector.invoke(mu, sigmas.ptr),
                Constrained::new);
    }

    public static shared_ptr<Constrained> MixedSigmas(Vector mu, Vector sigmas) {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_MixedSigmasVectorVector.invoke(mu.ptr, sigmas.ptr),
                Constrained::new);
    }

    public static shared_ptr<Constrained> MixedSigmas(Vector sigmas) {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_MixedSigmasVector.invoke(sigmas.ptr),
                Constrained::new);
    }

    public static shared_ptr<Constrained> All(int dim) {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_AllInt.invoke(dim),
                Constrained::new);
    }

    public static shared_ptr<Constrained> All(int dim, double mu) {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_AllIntDouble.invoke(dim, mu),
                Constrained::new);
    }

    public static shared_ptr<Constrained> All(int dim, Vector mu) {
        return new shared_ptr<Constrained>(
                (MemorySegment) FF.noiseModel_Constrained_AllIntVector.invoke(dim, mu.ptr),
                Constrained::new);
    }

    public Vector mu() {
        return new Vector((MemorySegment) FF.noiseModel_Constrained_mu.invoke(ptr));
    }

    public Matrix informationFromA(Matrix A) {
        return new Matrix((MemorySegment) FF.noiseModel_Constrained_informationFromA.invoke(ptr, A.ptr));
    }

}
