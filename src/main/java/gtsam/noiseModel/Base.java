package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

import gtsam.Matrix;
import gtsam.Vector;

public abstract class Base extends ForeignObject {

    public enum FF {
        noiseModel_Base_isUnit(JAVA_BOOLEAN, ADDRESS),
        noiseModel_Base_dim(JAVA_INT, ADDRESS),
        noiseModel_Base_squaredMahalanobisDistance(JAVA_DOUBLE, ADDRESS, ADDRESS),
        noiseModel_Base_loss(JAVA_DOUBLE, ADDRESS, JAVA_DOUBLE),
        noiseModel_Base_WhitenSystem(null, ADDRESS, ADDRESS, ADDRESS),
        noiseModel_Base_whitenInPlace(null, ADDRESS, ADDRESS),
        noiseModel_Base_unwhitenInPlace(null, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Base(MemorySegment p) {
        super(p, null);
    }

    public boolean isUnit() {
        return (boolean) FF.noiseModel_Base_isUnit.invoke(ptr);
    }

    public int dim() {
        return (int) FF.noiseModel_Base_dim.invoke(ptr);
    }

    public abstract Vector sigmas();

    public abstract Vector whiten(Vector v);

    public abstract Matrix Whiten(Matrix H);

    public abstract Vector unwhiten(Vector v);

    public double squaredMahalanobisDistance(Vector v) {
        return (double) FF.noiseModel_Base_squaredMahalanobisDistance.invoke(ptr, v.ptr);
    }

    public double loss(double d) {
        return (double) FF.noiseModel_Base_loss.invoke(ptr, d);
    }

    public void WhitenSystem(Matrix A, Vector b) {
        FF.noiseModel_Base_WhitenSystem.invoke(ptr, A.ptr, b.ptr);
    }

    public void whitenInPlace(Vector v) {
        FF.noiseModel_Base_whitenInPlace.invoke(ptr, v.ptr);
    }

    public void unwhitenInPlace(Vector v) {
        FF.noiseModel_Base_unwhitenInPlace.invoke(ptr, v.ptr);
    }

}
