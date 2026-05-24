package gtsam;

/**
 * See gtsam/base/Lie.h.
 * 
 * TODO: add Jacobians.
 * 
 * @param <T> the liegroup type, e.g. Pose2.
 * @param <V> the type of its tangent vector, e.g. Vector3.
 */
public interface LieGroup<//
        T extends LieGroup<T, V>, //
        V extends VectorType<V>>
        extends Group<T>, Manifold<T, V> {
    public interface Traits<//
            T extends LieGroup<T, V>, //
            V extends VectorType<V>>
            extends Group.Traits<T>, Manifold.Traits<T, V> {
        T Expmap(V v) throws Throwable;

        V Logmap(T g) throws Throwable;

        // TODO: add AdjointMap
        // *Hg = traits<T>::AdjointMap(g);
    }

    Traits<T, V> traits();
}
