package gtsam;

/**
 * See gtsam/base/Manifold.h.
 * 
 * TODO: add Jacobians.
 */
public interface Manifold<T extends Manifold<T>> {
    public interface Traits<T extends Manifold<T>> {
        /**
         * Tangent vector from p to q.
         * For Lie group, this is Logmap.
         * For vector space, this is just (q - p).
         */
        default Vector Local(T p, T q) throws Throwable {
            return p.local(q);
        }

        /**
         * Manifold point that is v away from p.
         * For Lie group, this is Expmap.
         * For vector space, this is just (p + v)
         */
        default T Retract(T p, Vector v) throws Throwable {
            return p.retract(v);
        }

    }

    Traits<T> traits();

    int dimension() throws Throwable;

    Vector local(T other) throws Throwable;

    T retract(Vector v) throws Throwable;
}
