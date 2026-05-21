package gtsam;

public interface Manifold<T extends Manifold<T>> {
    int dimension() throws Throwable;

    /**
     * Tangent vector from this to other.
     * For Lie group, this is Logmap.
     * For vector space, this is just (other - this).
     */
    Vector local(T other) throws Throwable;

}
