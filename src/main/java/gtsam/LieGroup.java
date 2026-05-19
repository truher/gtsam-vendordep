package gtsam;

/**
 * This is to make NumericalDerivative work.  I'm not happy with it,
 * but it does work, kinda.
 * TODO: include dimension as a type parameter.
 */
public interface LieGroup<X extends LieGroup<X>> {
    /** Manifold dimension. */
    int dimension();

    X retract(VectorSpace<?> v) throws Throwable;
}
