package gtsam;

import static java.lang.foreign.ValueLayout.ADDRESS;
import static java.lang.foreign.ValueLayout.JAVA_LONG;

import java.lang.foreign.MemorySegment;
import java.lang.foreign.ValueLayout;
import java.lang.invoke.MethodHandle;
import java.util.NoSuchElementException;

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
     * NOTE! Cholesky can be unstable in confusing ways, producing
     * IndeterminateSystemException. You should use QR instead.
     */
    public static Marginals Cholesky(NonlinearFactorGraph graph, Values result) {
        MemorySegment p = (MemorySegment) FF.MarginalsCholesky.invoke(graph.ptr, result.ptr);
        if (p.equals(MemorySegment.NULL)) {
            throw new IndeterminateSystemException();
        }
        return new Marginals(p);
    }

    /**
     * QR is more stable but slower.
     */
    public static Marginals QR(NonlinearFactorGraph graph, Values result) {
        MemorySegment p = (MemorySegment) FF.MarginalsQR.invoke(graph.ptr, result.ptr);
        if (p.equals(MemorySegment.NULL)) {
            throw new IndeterminateSystemException();
        }
        return new Marginals(p);
    }

    public Matrix marginalCovariance(Key key) {
        MemorySegment p = (MemorySegment) FF.Marginals_marginalCovariance.invoke(ptr, key.j);
        if (p.equals(MemorySegment.NULL)) {
            throw new NoSuchElementException(String.format("key %s", key));
        }
        return new Matrix(p);
    }

    public void print(String label) {
        System.out.println(label);
        FF.Marginals_print.invoke(ptr);
    }

}
