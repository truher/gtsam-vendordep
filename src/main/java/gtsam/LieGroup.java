package gtsam;

/**
 * See gtsam/base/Lie.h.
 * 
 * We implement most, but not all, of the following
 * 
 * LieGroup is a large "base class" of implementations (e.g. "compose"),
 * some of which are static (e.g. "Retract"), implemented using a "statics"
 * singleton.
 * 
 * There is also a large LieGroupTraits class, implemented as a singleton.
 * 
 * Some of those traits imply class methods (e.g. "Expmap"), so those
 * are implemented in the "statics" singleton.
 *
 * @param <T> the Lie group type, e.g. Pose2.
 * @param <V> the type of its tangent vector, e.g. Vector3.
 */
public interface LieGroup<//
        T extends LieGroup<T, V>, //
        V extends VectorType<V>>
        extends Group<T>, Manifold<T, V> {
    /**
     * See LieGroupTraits, which also restates the Group and Manifold traits.
     */
    public interface Traits<//
            T extends LieGroup<T, V>, //
            V extends VectorType<V>>
            extends Group.Traits<T>, Manifold.Traits<T, V> {
        // Group
        T Identity() throws Throwable;

        // TODO: REMOVE!
        T Expmap(V v) throws Throwable;

        // TODO: REMOVE!
        V Logmap(T g) throws Throwable;

        // // Manifold
        // V Local(T origin, T other, Matrix H1, Matrix H2) throws Throwable;

        // T Retract(T origin, V v, Matrix H, Matrix Hv) throws Throwable;

        // // LieGroup
        // V Logmap(T m, Matrix Hm) throws Throwable;

        // T Expmap(V v, Matrix Hv) throws Throwable;

        // T Compose(T m1, T m2, Matrix H1, Matrix H2) throws Throwable;

        // T Between(T m1, T m2, Matrix H1, Matrix H2) throws Throwable;

        // T Inverse(T m, Matrix H) throws Throwable;

        // Matrix AdjointMap(T m) throws Throwable;
    }

    /**
     * Static methods defined or implied in the base class.
     */
    public interface Statics<//
            T extends LieGroup<T, V>, //
            V extends VectorType<V>> {
        // Implied

        // T Identity() throws Throwable;

        // T Expmap(V v) throws Throwable;

        // V Logmap(T g) throws Throwable;

        // T Expmap(V v, Matrix H) throws Throwable;

        // V Logmap(T g, Matrix H) throws Throwable;

        // // Explicit

        // T Retract(V v) throws Throwable;

        // V LocalCoordinates(T g) throws Throwable;

        // T Retract(V v, Matrix H) throws Throwable;

        // V LocalCoordinates(T g, Matrix H) throws Throwable;
    }

    Traits<T, V> traits();

    // Statics<T, V> statics();

    T compose(T h) throws Throwable;

    T between(T h) throws Throwable;

    // T compose(T h, Matrix H1, Matrix H2) throws Throwable;

    // T between(T h, Matrix H1, Matrix H2) throws Throwable;

    T inverse(Matrix H) throws Throwable;

    // T expmap(V v) throws Throwable;

    // V logmap(T g) throws Throwable;

    // T expmap(V v, Matrix H1, Matrix H2) throws Throwable;

    // V logmap(T g, Matrix H1, Matrix H2) throws Throwable;

    T retract(V v) throws Throwable;

    V localCoordinates(T g) throws Throwable;

    // T retract(V v, Matrix H1, Matrix H2) throws Throwable;

    // V localCoordinates(T g, Matrix H1, Matrix H2) throws Throwable;

}
