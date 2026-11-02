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
        noiseModel_matchesDimensionDouble(JAVA_BOOLEAN, ADDRESS, JAVA_DOUBLE),
        noiseModel_matchesDimensionVector(JAVA_BOOLEAN, ADDRESS, ADDRESS),
        noiseModel_matchesDimensionMatrix(JAVA_BOOLEAN, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public static boolean matchesDimension(Base model, double measured) {
        return (boolean) FF.noiseModel_matchesDimensionDouble.invoke(model.ptr, measured);
    }

    public static boolean matchesDimension(Base model, Vector measured) {
        return (boolean) FF.noiseModel_matchesDimensionVector.invoke(model.ptr, measured.ptr);
    }

    public static boolean matchesDimension(Base model, Matrix measured) {
        return (boolean) FF.noiseModel_matchesDimensionMatrix.invoke(model.ptr, measured.ptr);
    }
}
