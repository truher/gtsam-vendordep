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
            Result_delete(null, ADDRESS);

            public final MethodHandle h;

            FF(ValueLayout returnType, ValueLayout... parameterTypes) {
                h = Lib.ff(this, returnType, parameterTypes);
            }
        }

        public Result(MemorySegment p) {
            super(p, FF.Result_delete.h);
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
            KeyTimestampMap_clear(null, ADDRESS);

            public final MethodHandle h;

            FF(ValueLayout returnType, ValueLayout... parameterTypes) {
                h = Lib.ff(this, returnType, parameterTypes);
            }
        }

        public KeyTimestampMap(MemorySegment p) {
            super(p, FF.KeyTimestampMap_delete.h);
        }

        public KeyTimestampMap() throws Throwable {
            this((MemorySegment) FF.KeyTimestampMap.h.invokeExact());
        }

        public void put(Key k, double v) throws Throwable {
            FF.KeyTimestampMap_put.h.invokeExact(ptr, k.j, v);
        }

        public void clear() throws Throwable {
            FF.KeyTimestampMap_clear.h.invokeExact(ptr);
        }
    }
}
