package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * add() methods use shared_ptr to save copying.
 */
public class NonlinearFactorGraph {
    public enum FF {
        NonlinearFactorGraph(ADDRESS),
        /** Expects shared_ptr<T extends NonlinearFactor>* */
        NonlinearFactorGraph_add(null, ADDRESS, ADDRESS),
        NonlinearFactorGraph_resize(null, ADDRESS, JAVA_LONG);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    /** gtsam::NonlinearFactorGraph* */
    final MemorySegment ptr;

    NonlinearFactorGraph(MemorySegment p) {
        ptr = p;
    }

    public NonlinearFactorGraph() throws Throwable {
        this((MemorySegment) FF.NonlinearFactorGraph.h.invokeExact());
    }

    /**
     * OK to delete f after this: the graph's shared_ptr will hold a reference to
     * the factor itself
     */
    public <T extends NonlinearFactor> void add(shared_ptr<T> f) throws Throwable {
        FF.NonlinearFactorGraph_add.h.invokeExact(ptr, f.sharedPtrPtr);
    }

    public void resize(long size) throws Throwable {
        FF.NonlinearFactorGraph_resize.h.invokeExact(ptr, size);
    }
}
