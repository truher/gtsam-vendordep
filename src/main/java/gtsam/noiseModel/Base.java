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
        noiseModel_Base_WhitenSystemVector(null, ADDRESS, ADDRESS),
        noiseModel_Base_whitenInPlace(null, ADDRESS, ADDRESS),
        noiseModel_Base_unwhitenInPlace(null, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    public Base(MemorySegment p) {
        super(p, null);
    }

    public boolean isUnit() throws Throwable {
        return (boolean) FF.noiseModel_Base_isUnit.h.invokeExact(ptr);
    }

    public int dim() throws Throwable {
        return (int) FF.noiseModel_Base_dim.h.invokeExact(ptr);
    }

    public abstract Vector sigmas() throws Throwable;

    public abstract Vector whiten(Vector v) throws Throwable;

    public abstract Matrix Whiten(Matrix H) throws Throwable;

    public abstract Vector unwhiten(Vector v) throws Throwable;

    public double squaredMahalanobisDistance(Vector v) throws Throwable {
        return (double) FF.noiseModel_Base_squaredMahalanobisDistance.h.invokeExact(ptr, v.ptr);
    }

    public double loss(double d) throws Throwable {
        return (double) FF.noiseModel_Base_loss.h.invokeExact(ptr, d);
    }

    public void WhitenSystem(Matrix A, Vector b) throws Throwable {
        FF.noiseModel_Base_WhitenSystem.h.invokeExact(ptr, A.ptr, b.ptr);
    }
    public void WhitenSystem(Vector b) throws Throwable {
        FF.noiseModel_Base_WhitenSystemVector.h.invokeExact(ptr, b.ptr);
    }
    public void whitenInPlace(Vector v) throws Throwable {
        FF.noiseModel_Base_whitenInPlace.h.invokeExact(ptr, v.ptr);
    }

    public void unwhitenInPlace(Vector v) throws Throwable {
        FF.noiseModel_Base_unwhitenInPlace.h.invokeExact(ptr, v.ptr);
    }

}
