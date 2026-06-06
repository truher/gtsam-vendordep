#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/PlanarProjectionFactor.h>

extern "C" {
gtsam::NonlinearFactorGraph* NonlinearFactorGraph() {
    return new gtsam::NonlinearFactorGraph();
}
void NonlinearFactorGraph_delete(gtsam::NonlinearFactorGraph* g) {
    delete g;
}
/**
 * NonlinearFactorGraph.add makes a new shared_ptr which shares ownership of the
 * underlying factor, so it's ok to delete p.
 *
 * @param p is actually shared_ptr<T extends NonlinearFactor>* but
 *          since it's not covariant in its parameter, I use void*.
 */
void NonlinearFactorGraph_add(gtsam::NonlinearFactorGraph* g,  //
                              void* p) {                       //
    g->add(*static_cast<std::shared_ptr<gtsam::NonlinearFactor>*>(p));
}
void NonlinearFactorGraph_addNonlinearFactorGraph(  //
    gtsam::NonlinearFactorGraph* f,                 //
    const gtsam::NonlinearFactorGraph* g) {         //
    f->add(*g);
}
void NonlinearFactorGraph_resize(gtsam::NonlinearFactorGraph* g,
                                 uint64_t size) {
    g->resize(size);
}
void NonlinearFactorGraph_addPriorPoint2(gtsam::NonlinearFactorGraph* g,  //
                                         gtsam::Key key,                  //
                                         const gtsam::Point2* prior,      //
                                         const gtsam::SharedNoiseModel* model) {
    g->addPrior(key, *prior, *model);
}
void NonlinearFactorGraph_addPriorPose2(gtsam::NonlinearFactorGraph* g,  //
                                        gtsam::Key key,                  //
                                        const gtsam::Pose2* prior,       //
                                        const gtsam::SharedNoiseModel* model) {
    g->addPrior(key, *prior, *model);
}
std::shared_ptr<gtsam::GaussianFactorGraph>* NonlinearFactorGraph_linearize(  //
    const gtsam::NonlinearFactorGraph* g,                                     //
    const gtsam::Values* init) {
    return new std::shared_ptr<gtsam::GaussianFactorGraph>(g->linearize(*init));
}
std::shared_ptr<gtsam::NonlinearFactor>* NonlinearFactorGraph_at(
    const gtsam::NonlinearFactorGraph* g, int i) {
    try {
        // "at" may return nullptr, which is ok.
        // "at" may throw, so catch and return nullptr anyway.
        return new std::shared_ptr<gtsam::NonlinearFactor>(g->at(i));
    } catch (const std::out_of_range& e) {
        std::cout << "caught err: " << e.what() << std::endl;
        // returning nullptr here is ok, noticed by java.
        return new std::shared_ptr<gtsam::NonlinearFactor>();
    }
}
int NonlinearFactorGraph_size(const gtsam::NonlinearFactorGraph* g) {
    return g->size();
}
}