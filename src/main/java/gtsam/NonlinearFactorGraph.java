package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_INT;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.Lib;

/**
 * add() methods use shared_ptr to save copying.
 * use the superclass, FactorGraph
 */
public class NonlinearFactorGraph extends FactorGraph {
    public enum FF {
        NonlinearFactorGraph(ADDRESS),
        NonlinearFactorGraph_delete(null, ADDRESS),
        /** Expects shared_ptr<T extends NonlinearFactor>* */
        NonlinearFactorGraph_add(null, ADDRESS, ADDRESS),
        NonlinearFactorGraph_addNonlinearFactorGraph(null, ADDRESS, ADDRESS),
        NonlinearFactorGraph_resize(null, ADDRESS, JAVA_LONG),
        NonlinearFactorGraph_addPriorPoint2(null, ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        NonlinearFactorGraph_addPriorPose2(null, ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        NonlinearFactorGraph_addPriorRot2(null, ADDRESS, JAVA_LONG, ADDRESS, ADDRESS),
        NonlinearFactorGraph_linearize(ADDRESS, ADDRESS, ADDRESS),
        NonlinearFactorGraph_at(ADDRESS, ADDRESS, JAVA_INT),
        NonlinearFactorGraph_size(JAVA_INT, ADDRESS),
        NonlinearFactorGraph_print(null, ADDRESS),
        NonlinearFactorGraph_printErrors(null, ADDRESS, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    NonlinearFactorGraph(MemorySegment p) {
        super(p, FF.NonlinearFactorGraph_delete.h);
    }

    public NonlinearFactorGraph() {
        this((MemorySegment) FF.NonlinearFactorGraph.invoke());
    }

    /**
     * OK to delete f after this: the graph's shared_ptr will hold a reference to
     * the factor itself
     */
    public <T extends NonlinearFactor> void add(shared_ptr<T> f) {
        FF.NonlinearFactorGraph_add.invoke(ptr, f.ptr);
    }

    public void add(NonlinearFactorGraph g) {
        FF.NonlinearFactorGraph_addNonlinearFactorGraph.invoke(ptr, g.ptr);
    }

    public void resize(long size) {
        FF.NonlinearFactorGraph_resize.invoke(ptr, size);
    }

    public void addPrior(
            Key key,
            Point2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model)
            {
        if (model.get().dim() != 2)
            throw new IllegalArgumentException();
        FF.NonlinearFactorGraph_addPriorPoint2.invoke(
                ptr, key.j, prior.ptr, model.ptr);
    }

    public void addPrior(
            Key key,
            Pose2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model)
            {
        if (model.get().dim() != 3)
            throw new IllegalArgumentException();
        FF.NonlinearFactorGraph_addPriorPose2.invoke(
                ptr, key.j, prior.ptr, model.ptr);
    }

    public void addPrior(
            Key key,
            Rot2 prior,
            shared_ptr<? extends gtsam.noiseModel.Base> model)
            {
        if (model.get().dim() != 1)
            throw new IllegalArgumentException();
        FF.NonlinearFactorGraph_addPriorRot2.invoke(
                ptr, key.j, prior.ptr, model.ptr);
    }

    public shared_ptr<GaussianFactorGraph> linearize(Values init) {
        MemorySegment sharedPtrPtr = (MemorySegment) FF.NonlinearFactorGraph_linearize.invoke(ptr, init.ptr);
        return new shared_ptr<>(sharedPtrPtr, GaussianFactorGraph::new);
    }

    /** Note the use of the factor *index* here, not the *key*. */
    public shared_ptr<NonlinearFactor> at(int i) {
        MemorySegment p = (MemorySegment) FF.NonlinearFactorGraph_at.invoke(
                ptr, i);
        // always returns shared_ptr, which might point at nullptr.
        return new shared_ptr<>(p, NonlinearFactor::new);
    }

    public int size() {
        return (int) FF.NonlinearFactorGraph_size.invoke(ptr);
    }

    public void print(String label) {
        System.out.println(label);
        FF.NonlinearFactorGraph_print.invoke(ptr);
    }

    public void printErrors(Values values, String label) {
        System.out.println(label);
        FF.NonlinearFactorGraph_printErrors.invoke(ptr, values.ptr);
    }
}
