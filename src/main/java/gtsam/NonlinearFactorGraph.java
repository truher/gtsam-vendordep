package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * add() methods use shared_ptr to save copying.
 */
public class NonlinearFactorGraph {

    private static final MethodHandle NonlinearFactorGraph = Lib.down(
            "NonlinearFactorGraph", ADDRESS);
    /** Expects shared_ptr<T extends NonlinearFactor>* */
    private static final MethodHandle NonlinearFactorGraph_add = Lib.downVoid(
            "NonlinearFactorGraph_add", ADDRESS, ADDRESS);
    private static final MethodHandle NonlinearFactorGraph_resize = Lib.downVoid(
            "NonlinearFactorGraph_resize", ADDRESS, JAVA_LONG);

    /** gtsam::NonlinearFactorGraph* */
    final MemorySegment ptr;

    NonlinearFactorGraph(MemorySegment p) {
        ptr = p;
    }

    public NonlinearFactorGraph() throws Throwable {
        this((MemorySegment) NonlinearFactorGraph.invokeExact());
    }

    /**
     * OK to delete f after this: the graph's shared_ptr will hold a reference to
     * the factor itself
     */
    public <T extends NonlinearFactor> void add(shared_ptr<T> f) throws Throwable {
        NonlinearFactorGraph_add.invokeExact(ptr, f.sharedPtrPtr);
    }

    public void resize(long size) throws Throwable {
        NonlinearFactorGraph_resize.invokeExact(ptr, size);
    }
}
