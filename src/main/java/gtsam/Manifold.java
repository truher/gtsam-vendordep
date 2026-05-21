package gtsam;

public interface Manifold<T extends Manifold<T>> {
    int dimension() throws Throwable;

    /**
     * Tangent vector from this to other.
     * For Lie group, this is Logmap.
     * For vector space, this is just (other - this).
     */
    Vector local(T other) throws Throwable;

    /**
     * Manifold point that is v away from this.
     * For Lie group, this is Expmap.
     * For vector space, this is just (this + v)
     */
    <V extends Vector> T retract(V v) throws Throwable;
}
