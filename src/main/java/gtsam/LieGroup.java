package gtsam;

/**
 * See gtsam/base/Lie.h.
 * 
 * Traits are implemented using the companion object pattern.
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
    public interface Companion<//
            T extends LieGroup<T, V>, //
            V extends VectorType<V>>
            extends Group.Companion<T>, Manifold.Companion<T, V> {
        /** Implement as statics.Identity(). */
        T Identity() throws Throwable;
        
        default V Local(T origin, T other) throws Throwable {
            return origin.localCoordinates(other);
        }

        default V Local(T origin, T other, Matrix H1, Matrix H2) throws Throwable {
            return origin.localCoordinates(other, H1, H2);
        }

        default T Retract(T origin, V v) throws Throwable {
            return origin.retract(v);
        }

        default T Retract(T origin, V v, Matrix H, Matrix Hv) throws Throwable {
            return origin.retract(v, H, Hv);
        }

        V Logmap(T g) throws Throwable;

        V Logmap(T m, Matrix Hm) throws Throwable;

        T Expmap(V v) throws Throwable;

        T Expmap(V v, Matrix Hv) throws Throwable;
    }

    Companion<T, V> companion();







    Matrix AdjointMap() throws Throwable;

    // TODO: delete this
    T expmap(V v) throws Throwable ;

    // TODO: delete this
    V logmap(T g) throws Throwable;

    // TODO: delete this
    T expmap(V v, Matrix H1, Matrix H2) throws Throwable;

    // TODO: delete this
    V logmap(T g, Matrix H1, Matrix H2) throws Throwable;

    // TODO: delete this
    T retract(V v) throws Throwable;

    // TODO: delete this
    V localCoordinates(T g) throws Throwable;

    // TODO: delete this
    T retract(V v, Matrix H1, Matrix H2) throws Throwable;

    // TODO: delete this
    V localCoordinates(T g, Matrix H1, Matrix H2) throws Throwable;
}
