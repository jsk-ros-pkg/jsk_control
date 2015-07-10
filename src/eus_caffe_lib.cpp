#include <iostream>
#include <memory>
#include <random>
#include <array>
#include <caffe/caffe.hpp>
#include <sys/stat.h>
#include <cassert>

class eus_caffe {
private:
  caffe::SolverParameter solver_param;
  std::shared_ptr<caffe::Solver<double>> solver;

public:
  int _get_blob_data (boost::shared_ptr<caffe::Blob<double> > blob, double* ret, int osize) {
    if ( blob == NULL ){
      std::cout << std::endl;
      std::cout << "[get_blob_data] null blob" << std::endl;
      return -1;
    }
    std::cout << "[i E " << blob->count() << "] = "<< " (";
    if ( osize < 0 || osize > blob->count() ) osize = blob->count();
    for ( int i=0; i<osize; i++ ){
      ret[i] = blob->cpu_data()[i];
      std::cout << ret[i] << " ";
    }
    std::cout << ")" << std::endl;
    return 0;
  }

  int get_blob_data (char* name, double* ret, int osize) {
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    return _get_blob_data( net->blob_by_name(name), ret, osize);
  }

  int get_layer_blob_data (char* name, int blob_id, double* ret, int osize) {
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    const boost::shared_ptr<caffe::Layer<double>> layer = net->layer_by_name(name);
    std::vector<boost::shared_ptr<caffe::Blob<double>>> ip_blobs = layer->blobs();
    std::cout << "<" << layer->type() << ">" << name << "[" << blob_id << "<" << ip_blobs.size() << "]" ;
    if ( ip_blobs.size() <= blob_id ){
      std::cout << std::endl;
      std::cout << " --- too large blob_id" << std::endl;
      return -1;
    }
    return _get_blob_data(ip_blobs[blob_id], ret, osize);
  }

  int create_solver (char* solver_path, char* solverstate){
    struct stat buf;
    if (solver_path && stat(solver_path, &buf)==0){
      caffe::ReadProtoFromTextFileOrDie(solver_path, &this->solver_param);
      if ( this->solver ) this->solver.reset();
      this->solver = std::shared_ptr<caffe::Solver<double>>(caffe::GetSolver<double>(this->solver_param));
      if (solverstate && stat(solverstate, &buf)==0) {
	std::cout << "Restoring previous solver status from " << solverstate << std::endl;
	this->solver->Restore(solverstate);
      }
    }
    return 0;
  }

  int reset_memory_layer(char* name, int size, double* data, double* label){
    assert(this->solver);
    //
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    const boost::shared_ptr<caffe::Layer<double>> layer_org = net->layer_by_name(name);
    if ( ! layer_org ){
      std::cout << " reset_memory_layer failed, " << name << " missing!!" << std::endl;
      return 1;
    }
    if ( layer_org->type() == "MemoryData" ) {
      boost::shared_ptr<caffe::MemoryDataLayer<double>> layer =
	boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(layer_org);
      layer->Reset(data, label, size);
    }
    //
    return 0;
  }

  // deprecated
  int initialize_solver (int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy){
    return this->reset_memory_layer("input", isize, idata, idummy) + this->reset_memory_layer("target", dsize, ddata, ddummy);
  }

  double caffe_learn () {
    assert(this->solver);
    //
    this->solver->Solve();
    //
    double buf[1];
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    get_blob_data("loss", buf, 1);
    return buf[0];
  }

  int calc_forward (int inputsize, int outputsize, double* input, double* output, double* idummy){
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    boost::shared_ptr<caffe::MemoryDataLayer<double>> input_layer =
      boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(net->layer_by_name("input"));
    assert(input_layer);
    //
    input_layer->Reset(input, idummy, inputsize);
    net->ForwardPrefilled(nullptr);
    //
    get_blob_data("output", output, outputsize);
    return 0;
  }
};

std::shared_ptr<eus_caffe> ec(new eus_caffe);

extern "C" {
  int eus_caffe_get_blob_data (char* name, double* ret, int osize){ return ec->get_blob_data(name,ret,osize); }
  int eus_caffe_get_layer_blob_data (char* name, int blob_id, double* ret, int osize) { return ec->get_layer_blob_data(name,blob_id,ret,osize); }
  //
  int eus_caffe_create_solver (char* solver_path, char* solverstate){ return ec->create_solver(solver_path,solverstate); }
  int eus_caffe_reset_memory_layer (char* name, int size, double* data, double* label){ return ec->reset_memory_layer(name,size,data,label); }
  int eus_caffe_initialize_solver (int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy){ return ec->initialize_solver(isize,dsize,idata,ddata,idummy,ddummy); }
  double eus_caffe_learn () { return ec->caffe_learn(); }
  int eus_caffe_calc_forward (int inputsize, int outputsize, double* input, double* output, double* idummy){ return ec->calc_forward(inputsize,outputsize,input,output,idummy);}
}
