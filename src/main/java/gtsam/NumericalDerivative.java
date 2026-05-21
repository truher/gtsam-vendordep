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

    @FunctionalInterface
    interface ThrowingFunction2<X1, X2, Y> {
        Y apply(X1 x1, X2 x2) throws Throwable;
    }

    @FunctionalInterface
    interface ThrowingFunction3<X1, X2, X3, Y> {
        Y apply(X1 x1, X2 x2, X3 x3) throws Throwable;
    }

    public static <Y extends Manifold<Y>, X extends Manifold<X> & LieGroup<X>> Matrix numericalDerivative11(
            ThrowingFunction<X, Y> h, X x) throws Throwable {
        double delta = 1e-5;
        Y hx = h.apply(x);
        int m = hx.dimension();
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
            H.setCol(j, dy1.minus(dy2).times(factor));
        }
        return H;
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1> & LieGroup<X1>, X2 extends Manifold<X2> & LieGroup<X2>> Matrix numericalDerivative21(
            ThrowingFunction2<X1, X2, Y> h, X1 x1, X2 x2)
            throws Throwable {
        return NumericalDerivative.<Y, X1>numericalDerivative11((X1 x) -> h.apply(x, x2), x1);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1> & LieGroup<X1>, X2 extends Manifold<X2> & LieGroup<X2>> Matrix numericalDerivative22(
            ThrowingFunction2<X1, X2, Y> h, X1 x1, X2 x2)
            throws Throwable {
        return NumericalDerivative.<Y, X2>numericalDerivative11((X2 x) -> h.apply(x1, x), x2);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1> & LieGroup<X1>, X2 extends Manifold<X2> & LieGroup<X2>, X3 extends Manifold<X3> & LieGroup<X3>> Matrix numericalDerivative31(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3)
            throws Throwable {
        return NumericalDerivative.<Y, X1>numericalDerivative11((X1 x) -> h.apply(x, x2, x3), x1);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1> & LieGroup<X1>, X2 extends Manifold<X2> & LieGroup<X2>, X3 extends Manifold<X3> & LieGroup<X3>> Matrix numericalDerivative32(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3)
            throws Throwable {
        return NumericalDerivative.<Y, X2>numericalDerivative11((X2 x) -> h.apply(x1, x, x3), x2);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1> & LieGroup<X1>, X2 extends Manifold<X2> & LieGroup<X2>, X3 extends Manifold<X3> & LieGroup<X3>> Matrix numericalDerivative33(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3)
            throws Throwable {
        return NumericalDerivative.<Y, X3>numericalDerivative11((X3 x) -> h.apply(x1, x2, x), x3);

    }

}
