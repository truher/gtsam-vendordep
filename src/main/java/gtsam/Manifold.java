package gtsam;

/**
 * See gtsam/base/Manifold.h.
 * 
 * TODO: add Jacobians.
 * TODO: fix the vector dimensionality
 * 
 * @param T the manifold type, e.g. Pose2.
 * @param V the type of its tangent vector, e.g. Vector3.
 */
public interface Manifold<//
        T extends Manifold<T, V>, //
        V extends VectorType<V>> {
    public interface Traits<//
            T extends Manifold<T, V>, //
            V extends VectorType<V>> {
        /**
         * Tangent vector from p to q.
         * For Lie group, this is Logmap.
         * For vector space, this is just (q - p).
         */
        default V Local(T p, T q) throws Throwable {
            return p.local(q);
        }

        /**
         * Manifold point that is v away from p.
         * For Lie group, this is Expmap.
         * For vector space, this is just (p + v)
         */
        default T Retract(T p, V v) throws Throwable {
            // System.out.printf("Retract %s %s\n", p, v);
            // System.out.flush();
            return p.retract(v);
        }

    }

    Traits<T, V> traits();

    /** New tangent vector filled with zeros. */
    V dxZero() throws Throwable;

    int dimension() throws Throwable;

    V local(T other) throws Throwable;

    T retract(V v) throws Throwable;
}
