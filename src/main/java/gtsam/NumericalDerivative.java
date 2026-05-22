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

    public static <Y extends Manifold<Y>, X extends Manifold<X>> Matrix numericalDerivative11(
            ThrowingFunction<X, Y> h, X x, double delta) throws Throwable {
        Y hx = h.apply(x);
        int m = hx.dimension();
        // using Eigen here would be a pain
        int N = x.dimension();
        Vector dx = new Vector(N);
        Matrix H = new Matrix(m, N);
        final double factor = 1.0 / (2.0 * delta);
        for (int j = 0; j < N; ++j) {
            dx.set(j, delta);
            System.out.printf("dx %s\n", dx);
            System.out.printf("hx %s\n", hx);
            System.out.printf("x %s\n", x);
            X x1 = x.retract(dx);
            System.out.printf("x1 %s\n", x1);
            Y hx1 = h.apply(x1);
            System.out.printf("hx1 %s\n", hx1);
            Vector dy1 = hx.local(hx1);
            System.out.printf("dy1 %s\n", dy1);
            dx.set(j, -delta);
            System.out.printf("dx %s\n", dx);
            X x2 = x.retract(dx);
            System.out.printf("x2 %s\n", x2);
            Y hx2 = h.apply(x2);
            System.out.printf("hx2 %s\n", hx2);
            Vector dy2 = hx.local(hx2);
            System.out.printf("dy2 %s\n", dy2);
            dx.set(j, 0);
            H.setCol(j, dy1.minus(dy2).times(factor));
        }
        return H;
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1>, X2 extends Manifold<X2>> Matrix numericalDerivative21(
            ThrowingFunction2<X1, X2, Y> h, X1 x1, X2 x2, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, X1>numericalDerivative11((X1 x) -> h.apply(x, x2), x1, delta);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1>, X2 extends Manifold<X2>> Matrix numericalDerivative22(
            ThrowingFunction2<X1, X2, Y> h, X1 x1, X2 x2, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, X2>numericalDerivative11((X2 x) -> h.apply(x1, x), x2, delta);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1>, X2 extends Manifold<X2>, X3 extends Manifold<X3>> Matrix numericalDerivative31(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, X1>numericalDerivative11((X1 x) -> h.apply(x, x2, x3), x1, delta);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1>, X2 extends Manifold<X2>, X3 extends Manifold<X3>> Matrix numericalDerivative32(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, X2>numericalDerivative11((X2 x) -> h.apply(x1, x, x3), x2, delta);
    }

    public static <Y extends Manifold<Y>, X1 extends Manifold<X1>, X2 extends Manifold<X2>, X3 extends Manifold<X3>> Matrix numericalDerivative33(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, X3>numericalDerivative11((X3 x) -> h.apply(x1, x2, x), x3, delta);

    }

}
