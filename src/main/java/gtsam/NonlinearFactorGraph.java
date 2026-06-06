package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/**
 * add() methods use shared_ptr to save copying.
 */
public class NonlinearFactorGraph extends ForeignObject {
    public enum FF {
        NonlinearFactorGraph(ADDRESS),
        /** Expects shared_ptr<T extends NonlinearFactor>* */
        NonlinearFactorGraph_add(null, ADDRESS, ADDRESS),
        NonlinearFactorGraph_addNonlinearFactorGraph(null, ADDRESS, ADDRESS),
        NonlinearFactorGraph_resize(null, ADDRESS, JAVA_LONG),
        NonlinearFactorGraph_addPriorPoint2(null, ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        NonlinearFactorGraph_linearize(ADDRESS, ADDRESS, ADDRESS),
        NonlinearFactorGraph_at(ADDRESS, ADDRESS, JAVA_INT),
        NonlinearFactorGraph_size(JAVA_INT, ADDRESS);

        public final MethodHandle h;

        FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }
    }

    NonlinearFactorGraph(MemorySegment p) {
        super(p, null);
    }

    public NonlinearFactorGraph() throws Throwable {
        this((MemorySegment) FF.NonlinearFactorGraph.h.invokeExact());
    }

    /**
     * OK to delete f after this: the graph's shared_ptr will hold a reference to
     * the factor itself
     */
    public <T extends NonlinearFactor> void add(shared_ptr<T> f) throws Throwable {
        FF.NonlinearFactorGraph_add.h.invokeExact(ptr, f.ptr);
    }

    public void add(NonlinearFactorGraph g) throws Throwable {
        FF.NonlinearFactorGraph_addNonlinearFactorGraph.h.invokeExact(ptr, g.ptr);
    }

    public void resize(long size) throws Throwable {
        FF.NonlinearFactorGraph_resize.h.invokeExact(ptr, size);
    }

    public void addPrior(
            Key key,
            Point2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model)
            throws Throwable {
        FF.NonlinearFactorGraph_addPriorPoint2.h.invokeExact(
                ptr, key.j, prior.ptr, model.ptr);
    }

    public shared_ptr<GaussianFactorGraph> linearize(Values init) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.NonlinearFactorGraph_linearize.h.invokeExact(ptr, init.ptr);
        return new shared_ptr<>(sharedPtrPtr, GaussianFactorGraph::new);
    }

    public shared_ptr<NonlinearFactor> at(int i) throws Throwable {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.NonlinearFactorGraph_at.h.invokeExact(ptr, i);
        return new shared_ptr<>(sharedPtrPtr, NonlinearFactor::new);
    }

    public int size() throws Throwable {
        return (int) FF.NonlinearFactorGraph_size.h.invokeExact(ptr);
    }
}
