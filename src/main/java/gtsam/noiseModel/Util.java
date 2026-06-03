package gtsam.noiseModel;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_BOOLEAN;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;

import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

import gtsam.Matrix;
import gtsam.Vector;

public class Util {
    public enum FF {
        // TODO: figure out why the scalar doesn't work.
        noiseModel_matchesDimensionDouble(JAVA_BOOLEAN, ADDRESS, JAVA_DOUBLE),
        noiseModel_matchesDimensionVector(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        noiseModel_matchesDimensionMatrix(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    // TODO: figure out why the scalar doesn't work.
    public static boolean matchesDimension(Base model, double measured) throws Throwable {
        return (boolean) FF.noiseModel_matchesDimensionDouble.h.invokeExact(model.ptr, measured);
    }

    public static boolean matchesDimension(Base model, Vector measured) throws Throwable {
        return (boolean) FF.noiseModel_matchesDimensionVector.h.invokeExact(model.ptr, measured.ptr);
    }

    public static boolean matchesDimension(Base model, Matrix measured) throws Throwable {
        return (boolean) FF.noiseModel_matchesDimensionMatrix.h.invokeExact(model.ptr, measured.ptr);
    }
}
