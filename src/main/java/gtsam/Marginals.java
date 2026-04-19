package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/** https://en.wikipedia.org/wiki/Marginal_distribution */
public class Marginals extends ForeignObject {
    private static final MethodHandle Marginals = Lib.down(
            "Marginals", ADDRESS, ADDRESS, ADDRESS);
    private static final MethodHandle Marginals_delete = Lib.downVoid(
            "Marginals_delete", ADDRESS);
    private static final MethodHandle Marginals_marginalCovariance = Lib.down(
            "Marginals_marginalCovariance", ADDRESS, ADDRESS, JAVA_LONG);

    public Marginals(MemorySegment p) {
        super(p, Marginals_delete);
    }

    public Marginals(NonlinearFactorGraph graph, Values result) throws Throwable {
        this((MemorySegment) Marginals.invokeExact(graph.ptr, result.ptr));
    }

    public Matrix marginalCovariance(Key key) throws Throwable {
        return new Matrix((MemorySegment) Marginals_marginalCovariance.invokeExact(ptr, key.j));
    }

}
