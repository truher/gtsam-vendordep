package gtsam;

/**
 * See gtsam/base/Lie.h.
 * 
 * TODO: add Jacobians.
 */
public interface LieGroup<T extends LieGroup<T, TangentVectorType>, TangentVectorType>
        extends Group<T>, Manifold<T, TangentVectorType> {
    public interface Traits<T extends LieGroup<T, TangentVectorType>, TangentVectorType>
            extends Group.Traits<T>, Manifold.Traits<T, TangentVectorType> {
        T Expmap(TangentVectorType v) throws Throwable;

        TangentVectorType Logmap(T g) throws Throwable;

        // *Hg = traits<T>::AdjointMap(g);
    }

    Traits<T, TangentVectorType> traits();
}
