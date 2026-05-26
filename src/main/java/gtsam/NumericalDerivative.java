package gtsam;

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

    public static <//
            Y extends Manifold<Y, Yvec>, //
            Yvec extends VectorType<Yvec>, //
            X extends Manifold<X, Xvec>, //
            Xvec extends VectorType<Xvec>//
    > Matrix numericalDerivative11(
            ThrowingFunction<X, Y> h, X x, double delta) throws Throwable {
        // System.out.println("numericalDerivative11");
        Manifold.Traits<X, Xvec> TraitsX = x.traits();
        Y hx = h.apply(x);
        Manifold.Traits<Y, Yvec> TraitsY = hx.traits();
        int m = hx.dimension();
        // using Eigen here would be a pain
        int N = x.dimension();
        Xvec dx = x.dxZero();
        Matrix H = new Matrix(m, N);
        final double factor = 1.0 / (2.0 * delta);
        for (int j = 0; j < N; ++j) {
            // System.out.printf("j %d\n", j);
            dx.set(j, delta);
            // System.out.printf("dx (right) %s\n", dx);
            // System.out.printf("hx %s\n", hx);
            X x1 = TraitsX.Retract(x, dx);
            // System.out.printf("x (center) %s\n", x);
            // System.out.printf("x1 (right) %s\n", x1);
            Y hx1 = h.apply(x1);
            // System.out.printf("hx1 (right, corresponding to x1) %s\n", hx1);
            Yvec dy1 = TraitsY.Local(hx, hx1);
            // System.out.printf("dy1 %s\n", dy1);
            dx.set(j, -delta);
            // System.out.printf("dx (left) %s\n", dx);
            X x2 = TraitsX.Retract(x, dx);
            // System.out.printf("x (center) %s\n", x);
            // System.out.printf("x2 (left) %s\n", x2);
            Y hx2 = h.apply(x2);
            // System.out.printf("hx2 (left, corresponding to x2) %s\n", hx2);
            Yvec dy2 = TraitsY.Local(hx, hx2);
            // System.out.printf("dy2 (y distance, left) %s\n", dy2);
            dx.set(j, 0);
            H.setCol(j, dy1.minus(dy2).times(factor));
        }
        return H;
    }

    public static <//
            Y extends Manifold<Y, Yvec>, //
            Yvec extends VectorType<Yvec>, //
            X1 extends Manifold<X1, X1vec>, //
            X1vec extends VectorType<X1vec>, //
            X2 extends Manifold<X2, X2vec>, //
            X2vec extends VectorType<X2vec>//
    > Matrix numericalDerivative21(
            ThrowingFunction2<X1, X2, Y> h, X1 x1, X2 x2, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, Yvec, X1, X1vec>numericalDerivative11(
                (X1 x) -> h.apply(x, x2), x1, delta);
    }

    public static <//
            Y extends Manifold<Y, Yvec>, //
            Yvec extends VectorType<Yvec>, //
            X1 extends Manifold<X1, X1vec>, //
            X1vec extends VectorType<X1vec>, //
            X2 extends Manifold<X2, X2vec>, //
            X2vec extends VectorType<X2vec>//
    > Matrix numericalDerivative22(
            ThrowingFunction2<X1, X2, Y> h, X1 x1, X2 x2, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, Yvec, X2, X2vec>numericalDerivative11(
                (X2 x) -> h.apply(x1, x), x2, delta);
    }

    public static <//
            Y extends Manifold<Y, Yvec>, //
            Yvec extends VectorType<Yvec>, //
            X1 extends Manifold<X1, X1vec>, //
            X1vec extends VectorType<X1vec>, //
            X2 extends Manifold<X2, X2vec>, //
            X2vec extends VectorType<X2vec>, //
            X3 extends Manifold<X3, X3vec>, //
            X3vec extends VectorType<X3vec>//
    > Matrix numericalDerivative31(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, Yvec, X1, X1vec>numericalDerivative11(
                (X1 x) -> h.apply(x, x2, x3), x1, delta);
    }

    public static < //
            Y extends Manifold<Y, Yvec>, //
            Yvec extends VectorType<Yvec>, //
            X1 extends Manifold<X1, X1vec>, //
            X1vec extends VectorType<X1vec>, //
            X2 extends Manifold<X2, X2vec>, //
            X2vec extends VectorType<X2vec>, //
            X3 extends Manifold<X3, X3vec>, //
            X3vec extends VectorType<X3vec>//
    > Matrix numericalDerivative32(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, Yvec, X2, X2vec>numericalDerivative11(
                (X2 x) -> h.apply(x1, x, x3), x2, delta);
    }

    public static < //
            Y extends Manifold<Y, Yvec>, //
            Yvec extends VectorType<Yvec>, //
            X1 extends Manifold<X1, X1vec>, //
            X1vec extends VectorType<X1vec>, //
            X2 extends Manifold<X2, X2vec>, //
            X2vec extends VectorType<X2vec>, //
            X3 extends Manifold<X3, X3vec>, //
            X3vec extends VectorType<X3vec> //
    > Matrix numericalDerivative33(
            ThrowingFunction3<X1, X2, X3, Y> h, X1 x1, X2 x2, X3 x3, double delta)
            throws Throwable {
        return NumericalDerivative.<Y, Yvec, X3, X3vec>numericalDerivative11(
                (X3 x) -> h.apply(x1, x2, x), x3, delta);

    }

}
