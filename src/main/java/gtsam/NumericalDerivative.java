package gtsam;

/**
 * I'm not happy with the VectorSpace/LieGroup thing here,
 * but it does work, kinda.
 * TODO: add dimensionality as a type parameter somehow.
 */
public class NumericalDerivative {
    @FunctionalInterface
    interface ThrowingFunction<X, Y> {
        Y apply(X t) throws Throwable;
    }

    public static <Y extends VectorSpace<Y>, X extends LieGroup<X>> Matrix numericalDerivative11(
            ThrowingFunction<X, Y> h, X x) throws Throwable {
        double delta = 1e-5;
        Y hx = h.apply(x);
        int m = hx.getDimension();
        // using Eigen here would be a pain
        int N = x.dimension();
        Vector dx = new Vector(N);
        Matrix H = new Matrix(m, N);
        final double factor = 1.0 / (2.0 * delta);
        for (int j = 0; j < N; ++j) {
            dx.set(j, delta);
            Vector dy1 = hx.local(h.apply(x.retract(dx)));
            dx.set(j, -delta);
            Vector dy2 = hx.local(h.apply(x.retract(dx)));
            dx.set(j, 0);
            System.out.flush();
            H.setCol(j, dy1.minus(dy2).times(factor));
        }
        return H;
    }
}
