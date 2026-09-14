package gtsam;

/**
 * See gtsam/base/Lie.h.
 * 
 * A Lie group is both a group and a manifold.
 * 
 * This requires three more operators:
 * 
 * * a.logmap(b): like local
 * * a.expmap(b): like retract
 * * a.AdjointMap(): matrix for mapping vector-space twists.
 * 
 * https://en.wikipedia.org/wiki/Lie_group
 *
 * @param <T> the Lie group type, e.g. Pose2.
 * @param <V> the type of its tangent vector, e.g. Vector3.
 */
public interface LieGroup<//
        T extends LieGroup<T, V>, //
        V extends VectorType<V>>
        extends Group<T>, Manifold<T, V> {

    Matrix AdjointMap() throws Throwable;

    T expmap(V v) throws Throwable;

    T expmap(V v, Matrix H1, Matrix H2) throws Throwable;

    V logmap(T g) throws Throwable;

    V logmap(T g, Matrix H1, Matrix H2) throws Throwable;
}
