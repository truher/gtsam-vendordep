package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_DOUBLE;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public class FixedLagSmoother {
    /**
     * struct Result {
     * size_t iterations; ///< The number of optimizer iterations performed
     * size_t intermediateSteps; ///< The number of intermediate steps performed
     * within the optimization. For L-M, this is the number of lambdas tried.
     * size_t nonlinearVariables; ///< The number of variables that can be
     * relinearized
     * size_t linearVariables; ///< The number of variables that must keep a
     * constant linearization point
     * double error;
     */
    public static class Result extends ForeignObject {
        private static final MethodHandle Result_delete = Lib.downVoid(
                "Result_delete", ADDRESS);

        public Result(MemorySegment p) {
            super(p, Result_delete);
        }
    }

    /**
     * in gtsam/nonlinear/FixedLagSmoother.h:
     * typedef std::map<Key, double> KeyTimestampMap;
     */
    public static class KeyTimestampMap extends ForeignObject {
        private static final MethodHandle KeyTimestampMap = Lib.down(
                "KeyTimestampMap", ADDRESS);
        private static final MethodHandle KeyTimestampMap_delete = Lib.downVoid(
                "KeyTimestampMap_delete", ADDRESS);
        private static final MethodHandle KeyTimestampMap_put = Lib.downVoid(
                "KeyTimestampMap_put", ADDRESS, JAVA_LONG, JAVA_DOUBLE);
        private static final MethodHandle KeyTimestampMap_clear = Lib.downVoid(
                "KeyTimestampMap_clear", ADDRESS);

        public KeyTimestampMap(MemorySegment p) {
            super(p, KeyTimestampMap_delete);
        }

        public KeyTimestampMap() throws Throwable {
            this((MemorySegment) KeyTimestampMap.invokeExact());
        }

        public void put(Key k, double v) throws Throwable {
            KeyTimestampMap_put.invokeExact(ptr, k.j, v);
        }

        public void clear() throws Throwable {
            KeyTimestampMap_clear.invokeExact(ptr);
        }
    }
}
