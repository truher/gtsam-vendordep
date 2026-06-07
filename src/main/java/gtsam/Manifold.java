package gtsam;

/**
 * See gtsam/base/Manifold.h, in particular HasManifoldPrereqs.
 * 
 * Traits are implemented using the companion object pattern.
 * 
 * @param <T> the manifold type, e.g. Pose2.
 * @param <V> the type of its tangent vector, e.g. Vector3.
 */
public interface Manifold<//
        T extends Manifold<T, V>, //
        V extends VectorType<V>> {
    /**
     * See ManifoldTraits.
     * Manifold traits are all static.
     * The defaults here match the C++ implementation, which is
     * never overridden.
     */
    public interface Companion<//
            T extends Manifold<T, V>, //
            V extends VectorType<V>> {

        /** Required by GetDimensionImpl. */
        default int GetDimension(T m) throws Throwable {
            return m.dimension();
        };

        /**
         * Tangent vector from p to q.
         * For Lie group, this is Logmap.
         * For vector space, this is just (q - p).
         * Required by ManifoldTraits.
         */
        default V Local(T p, T q) throws Throwable {
            return p.localCoordinates(q);
        }

        /**
         * Manifold point that is v away from p.
         * For Lie group, this is Expmap.
         * For vector space, this is just (p + v)
         * Required by ManifoldTraits.
         */
        default T Retract(T p, V v) throws Throwable {
            // System.out.printf("Retract %s %s\n", p, v);
            // System.out.flush();
            return p.retract(v);
        }

    }

    Companion<T, V> companion();

    /** Zero tangent vector, used by numerical differentiation. */
    V dxZero() throws Throwable;

    int dimension() throws Throwable;

    /** Required by HasManifoldPrereqs */
    V localCoordinates(T other) throws Throwable;

    /** Required by HasManifoldPrereqs */
    T retract(V v) throws Throwable;
}
