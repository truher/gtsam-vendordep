package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_INT;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.util.Map;
import java.util.TreeMap;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public abstract class NonlinearOptimizerParams extends ForeignObject {
    public enum FF {
        NonlinearOptimizerParams_getMaxIterations(JAVA_LONG, ADDRESS),
        NonlinearOptimizerParams_getRelativeErrorTol(JAVA_DOUBLE, ADDRESS),
        NonlinearOptimizerParams_getAbsoluteErrorTol(JAVA_DOUBLE, ADDRESS),
        NonlinearOptimizerParams_getErrorTol(JAVA_DOUBLE, ADDRESS),
        NonlinearOptimizerParams_getVerbosity(JAVA_INT, ADDRESS),
        NonlinearOptimizerParams_setMaxIterations(null, ADDRESS, JAVA_INT),
        NonlinearOptimizerParams_setRelativeErrorTol(null, ADDRESS, JAVA_DOUBLE),
        NonlinearOptimizerParams_setAbsoluteErrorTol(null, ADDRESS, JAVA_DOUBLE),
        NonlinearOptimizerParams_setErrorTol(null, ADDRESS, JAVA_DOUBLE),
        NonlinearOptimizerParams_setVerbosity(null, ADDRESS, JAVA_INT);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public enum Verbosity {
        SILENT(0), TERMINATION(1), ERROR(2), VALUES(3), DELTA(4), LINEAR(5);

        private final int val;

        private static final Map<Integer, Verbosity> all = new TreeMap<>();
        static {
            for (Verbosity v : Verbosity.values()) {
                all.put(v.val, v);
            }
        }

        Verbosity(int val) {
            this.val = val;
        }

        public static Verbosity of(int v) {
            return all.get(v);
        }
    };

    NonlinearOptimizerParams(MemorySegment p, MethodHandle deleter) {
        // abstract, deleted by subclass
        super(p, deleter);
    }

    /** The maximum iterations to stop iterating (default 100) */
    public long getMaxIterations() {
        return (long) FF.NonlinearOptimizerParams_getMaxIterations.invoke(ptr);
    }

    /** The maximum relative error decrease to stop iterating (default 1e-5) */
    public double getRelativeErrorTol() {
        return (double) FF.NonlinearOptimizerParams_getRelativeErrorTol.invoke(ptr);
    }

    /** The maximum absolute error decrease to stop iterating (default 1e-5) */
    public double getAbsoluteErrorTol() {
        return (double) FF.NonlinearOptimizerParams_getAbsoluteErrorTol.invoke(ptr);
    }

    /** The maximum total error to stop iterating (default 0.0) */
    public double getErrorTol() {
        return (double) FF.NonlinearOptimizerParams_getErrorTol.invoke(ptr);
    }

    /** The printing verbosity during optimization (default SILENT) */
    public Verbosity getVerbosity() {
        return Verbosity.of((int) FF.NonlinearOptimizerParams_getVerbosity.invoke(ptr));
    }

    public void setMaxIterations(int value) {
        FF.NonlinearOptimizerParams_setMaxIterations.invoke(ptr, value);
    }

    public void setRelativeErrorTol(double value) {
        FF.NonlinearOptimizerParams_setRelativeErrorTol.invoke(ptr, value);
    }

    public void setAbsoluteErrorTol(double value) {
        FF.NonlinearOptimizerParams_setAbsoluteErrorTol.invoke(ptr, value);
    }

    public void setErrorTol(double value) {
        FF.NonlinearOptimizerParams_setErrorTol.invoke(ptr, value);
    }

    public void setVerbosity(Verbosity value) {
        FF.NonlinearOptimizerParams_setVerbosity.invoke(ptr, value.val);
    }
}
