#include <iostream>
#include <memory>
#include <random>
#include <array>
#include <caffe/caffe.hpp>

caffe::SolverParameter solver_param;
std::shared_ptr<caffe::Solver<double>> solver;

extern "C" {
  int deep_learning_test (int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy) {
    //
    // caffe::SolverParameter solver_param;
    caffe::ReadProtoFromTextFileOrDie("solver.prototxt", &solver_param);
    // std::shared_ptr<caffe::Solver<double>> solver =
    solver = std::shared_ptr<caffe::Solver<double>>(caffe::GetSolver<double>(solver_param));
    boost::shared_ptr<caffe::Net<double>> net = solver->net();
    boost::shared_ptr<caffe::MemoryDataLayer<double>> input_layer =
      boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(net->layer_by_name("input"));
    assert(input_layer);
    input_layer->Reset(idata, idummy, isize);
    boost::shared_ptr<caffe::MemoryDataLayer<double>> target_layer =
      boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(net->layer_by_name("target"));
    assert(target_layer);
    target_layer->Reset(ddata, ddummy, dsize);
    solver->Solve();
    //
    return 0;
  }
}
