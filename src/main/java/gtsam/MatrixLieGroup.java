package gtsam;

public interface MatrixLieGroup<T extends MatrixLieGroup<T, TangentVectorType>, TangentVectorType>
        extends LieGroup<T, TangentVectorType> {
    public interface Traits<T extends LieGroup<T, TangentVectorType>, TangentVectorType>
            extends LieGroup.Traits<T, TangentVectorType> {
        Matrix Hat(TangentVectorType xi) throws Throwable;

        TangentVectorType Vee(Matrix X) throws Throwable;

        // (void)traits<T>::Vec(g);
    }

    Traits<T, TangentVectorType> traits();

}
