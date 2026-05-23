package gtsam;

/**
 * See gtsam/base/Manifold.h.
 * 
 * TODO: add Jacobians.
 * TODO: fix the vector dimensionality
 * For now, specify the tangent vector type (e.g. Vector2, Vector3, etc)
 */
public interface Manifold<T extends Manifold<T, TangentVectorType>, TangentVectorType> {
    public interface Traits<T extends Manifold<T, TangentVectorType>, TangentVectorType> {
        /**
         * Tangent vector from p to q.
         * For Lie group, this is Logmap.
         * For vector space, this is just (q - p).
         */
        default TangentVectorType Local(T p, T q) throws Throwable {
            return p.local(q);
        }

        /**
         * Manifold point that is v away from p.
         * For Lie group, this is Expmap.
         * For vector space, this is just (p + v)
         */
        default T Retract(T p, TangentVectorType v) throws Throwable {
            return p.retract(v);
        }

    }

    Traits<T, TangentVectorType> traits();

    /** New tangent vector filled with zeros. */
    TangentVectorType dxZero() throws Throwable;

    int dimension() throws Throwable;

    TangentVectorType local(T other) throws Throwable;

    T retract(TangentVectorType v) throws Throwable;
}
