package gtsam;

public interface MatrixLieGroup<T extends MatrixLieGroup<T>>
        extends LieGroup<T> {
    public interface Traits<T extends LieGroup<T>>
            extends LieGroup.Traits<T> {
        Matrix Hat(Vector xi) throws Throwable;

        Vector Vee(Matrix X) throws Throwable;

        // (void)traits<T>::Vec(g);
    }

    Traits<T> traits();

}
