#include <iostream>
#include <memory>
#include <random>
#include <array>
#include <caffe/caffe.hpp>

caffe::SolverParameter solver_param;
std::shared_ptr<caffe::Solver<double>> solver;

extern "C" {
  int get_blob_data (boost::shared_ptr<caffe::Blob<double> > blob, double* ret, int osize) {
    if ( blob == NULL ){
      std::cout << std::endl;
      std::cout << "[get_blob_data] null blob" << std::endl;
      return -1;
    }
    std::cout << "(" << blob->count() << ") "<< " (";
    if ( osize < 0 || osize > blob->count() ) osize = blob->count();
    for ( int i=0; i<osize; i++ ){
      ret[i] = blob->cpu_data()[i];
      std::cout << ret[i] << " ";
    }
    std::cout << ")" << std::endl;
    return 0;
  }
}

extern "C" {
  int get_blob_by_id_and_layer_name (std::string name, int blob_id, double* ret, int osize) {
    boost::shared_ptr<caffe::Net<double>> net = solver->net();
    std::vector<boost::shared_ptr<caffe::Blob<double> > > ip_blobs = net->layer_by_name(name)->blobs();
    std::cout << name << "[" << blob_id << "<" << ip_blobs.size() << "]" ;
    if ( ip_blobs.size() <= blob_id ){
      std::cout << std::endl;
      std::cout << " --- too large blob_id" << std::endl;
      return -1;
    }
    return get_blob_data(ip_blobs[blob_id], ret, osize);
  }
}

extern "C" {
  double caffe_learn (char* solver_path, int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy) {
    //
    caffe::ReadProtoFromTextFileOrDie(solver_path, &solver_param);
    solver = std::shared_ptr<caffe::Solver<double>>(caffe::GetSolver<double>(solver_param));
    //
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
    // get_blob_by_id_and_layer_name("loss",0,idummy);
    get_blob_data( net->blob_by_name("loss"), idummy, 1);
    return idummy[0];
  }
}

extern "C" {
  int get_ip_layer_blob (int blob_id, double* ret, int osize) {
    return get_blob_by_id_and_layer_name("ip",blob_id,ret, osize);
  }
}

extern "C" {
  int calc_forward (int inputsize, int outputsize,
			  double* input, double* output, double* idummy){
    boost::shared_ptr<caffe::Net<double>> net = solver->net();
    boost::shared_ptr<caffe::MemoryDataLayer<double>> input_layer =
      boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(net->layer_by_name("input"));
    assert(input_layer);
    //
    input_layer->Reset(input, idummy, inputsize);
    net->ForwardPrefilled(nullptr);
    //
    get_blob_data(net->blob_by_name("output"), output, outputsize);
    return 0;
  }
}

extern "C" {
  int string_test (char* text) {
    std::cout << text << std::endl;
    return -1;
  }
}
