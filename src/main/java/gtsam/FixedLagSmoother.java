package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class FixedLagSmoother {
    /**
     * {@snippet :
     * struct Result {
     *   size_t iterations; ///< The number of optimizer iterations performed
     *   size_t intermediateSteps; ///< The number of intermediate steps performed within the optimization. For L-M, this is the number of lambdas tried.
     *   size_t nonlinearVariables; ///< The number of variables that can be relinearized
     *   size_t linearVariables; ///< The number of variables that must keep a constant linearization point
     *   double error;
     * }
     * }
     */
    public static class Result extends ForeignObject {
        public enum FF {
            Result_delete(null, ADDRESS),
            Result_getIterations(JAVA_LONG, ADDRESS),
            Result_getError(JAVA_DOUBLE, ADDRESS);

            private final MethodHandle h;

            private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
                h = Lib.ff(this, returnType, parameterTypes);
            }

            private Object invoke(Object... args) {
                return Lib.invoke(h, args);
            }
        }

        public Result(MemorySegment p) {
            super(p, FF.Result_delete.h);
        }

        public long getIterations() {
            return (long) FF.Result_getIterations.invoke(ptr);
        }

        public double getError() {
            return (double) FF.Result_getError.invoke(ptr);
        }
    }

    /**
     * {@snippet :
     * in gtsam/nonlinear/FixedLagSmoother.h:
     * typedef std::map<Key, double> KeyTimestampMap;
     * }
     */
    public static class KeyTimestampMap extends ForeignObject {
        public enum FF {
            KeyTimestampMap(ADDRESS),
            KeyTimestampMap_delete(null, ADDRESS),
            KeyTimestampMap_put(null, ADDRESS, JAVA_LONG, JAVA_DOUBLE),
            KeyTimestampMap_clear(null, ADDRESS),
            KeyTimestampMap_print(null, ADDRESS);

            private final MethodHandle h;

            private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
                h = Lib.ff(this, returnType, parameterTypes);
            }

            private Object invoke(Object... args) {
                return Lib.invoke(h, args);
            }
        }

        public KeyTimestampMap(MemorySegment p) {
            super(p, FF.KeyTimestampMap_delete.h);
        }

        public KeyTimestampMap() {
            this((MemorySegment) FF.KeyTimestampMap.invoke());
        }

        public void put(Key k, double v) {
            FF.KeyTimestampMap_put.invoke(ptr, k.j, v);
        }

        public void clear() {
            FF.KeyTimestampMap_clear.invoke(ptr);
        }

        public void print() {
            FF.KeyTimestampMap_print.invoke(ptr);
        }
    }
}
