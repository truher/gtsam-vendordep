package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

public abstract class NonlinearOptimizer extends ForeignObject {
    public enum FF {
        NonlinearOptimizer_optimize(ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    NonlinearOptimizer(MemorySegment pointer, MethodHandle deleter) {
        // abstract, deleted by subclass
        super(pointer, deleter);
    }

    public Values optimize() {
        return Values.owned((MemorySegment) FF.NonlinearOptimizer_optimize.invoke(ptr));
    }

}
