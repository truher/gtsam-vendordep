#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/CustomFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/PlanarProjectionFactor.h>

extern "C" {
gtsam::NonlinearFactorGraph* NonlinearFactorGraph() {
    return new gtsam::NonlinearFactorGraph();
}
/**
 * NonlinearFactorGraph.add makes a new shared_ptr which shares ownership of the
 * underlying factor, so it's ok to delete p.
 *
 * @param p is actually shared_ptr<T extends NonlinearFactor>* but
 *          since it's not covariant in its parameter, I use void*.
 */
void NonlinearFactorGraph_add(gtsam::NonlinearFactorGraph* g,//
     std::shared_ptr<void>* p) {
    g->add(*static_cast<std::shared_ptr<gtsam::NonlinearFactor>*>(p));
}
void NonlinearFactorGraph_addNonlinearFactorGraph(gtsam::NonlinearFactorGraph* f,
const gtsam::NonlinearFactorGraph* g) {
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
}