package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;

import org.team100.foreign.ForeignObject;
import org.team100.foreign.Lib;

/** https://en.wikipedia.org/wiki/Marginal_distribution */
public class Marginals extends ForeignObject {
    public enum FF {
        MarginalsCholesky(ADDRESS, ADDRESS, ADDRESS),
        MarginalsQR(ADDRESS, ADDRESS, ADDRESS),
        Marginals_delete(null, ADDRESS),
        Marginals_marginalCovariance(ADDRESS, ADDRESS, JAVA_LONG),
        Marginals_print(null, ADDRESS);

        private final MethodHandle h;

        private FF(ValueLayout returnType, ValueLayout... parameterTypes) {
            h = Lib.ff(this, returnType, parameterTypes);
        }

        private Object invoke(Object... args) {
            return Lib.invoke(h, args);
        }
    }

    public Marginals(MemorySegment p) {
        super(p, FF.Marginals_delete.h);
    }

    /**
     * Default is Cholesky.
     * NOTE! Cholesky can be unstable in confusing ways, producing
     * IndeterminateSystemException. You should use QR instead.
     */
    public Marginals(NonlinearFactorGraph graph, Values result) {
        this((MemorySegment) FF.MarginalsCholesky.invoke(graph.ptr, result.ptr));
    }

    /**
     * NOTE! Cholesky can be unstable in confusing ways, producing
     * IndeterminateSystemException. You should use QR instead.
     */
    public static Marginals Cholesky(NonlinearFactorGraph graph, Values result) {
        return new Marginals(graph, result);
    }

    /**
     * QR is more stable but slower.
     */
    public static Marginals QR(NonlinearFactorGraph graph, Values result) {
        return new Marginals((MemorySegment) FF.MarginalsQR.invoke(graph.ptr, result.ptr));
    }

    public Matrix marginalCovariance(Key key) {
        return new Matrix((MemorySegment) FF.Marginals_marginalCovariance.invoke(ptr, key.j));
    }

    public void print(String label) {
        System.out.println(label);
        FF.Marginals_print.invoke(ptr);
    }

}
