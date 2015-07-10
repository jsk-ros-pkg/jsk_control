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

  int get_blob_by_id_and_layer_name (std::string name, int blob_id, double* ret, int osize) {
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    std::vector<boost::shared_ptr<caffe::Blob<double> > > ip_blobs = net->layer_by_name(name)->blobs();
    std::cout << name << "[" << blob_id << "<" << ip_blobs.size() << "]" ;
    if ( ip_blobs.size() <= blob_id ){
      std::cout << std::endl;
      std::cout << " --- too large blob_id" << std::endl;
      return -1;
    }
    return get_blob_data(ip_blobs[blob_id], ret, osize);
  }

  int get_ip_layer_blob (int blob_id, double* ret, int osize) {
    return get_blob_by_id_and_layer_name("ip",blob_id,ret, osize);
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

  int initialize_solver (int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy){
    //
    assert(this->solver);
    //
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    boost::shared_ptr<caffe::MemoryDataLayer<double>> input_layer =
      boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(net->layer_by_name("input"));
    assert(input_layer);
    input_layer->Reset(idata, idummy, isize);
    boost::shared_ptr<caffe::MemoryDataLayer<double>> target_layer =
      boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(net->layer_by_name("target"));
    assert(target_layer);
    target_layer->Reset(ddata, ddummy, dsize);
    //
    return 1;
  }

  double caffe_learn () {
    assert(this->solver);
    //
    this->solver->Solve();
    //
    double buf[1];
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    get_blob_data( net->blob_by_name("loss"), buf, 1);
    return buf[0];
  }

  int calc_forward (int inputsize, int outputsize,
			  double* input, double* output, double* idummy){
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
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
};

std::shared_ptr<eus_caffe> ec(new eus_caffe);

extern "C" {
  int get_blob_data (boost::shared_ptr<caffe::Blob<double> > blob, double* ret, int osize){ return ec->get_blob_data(blob,ret,osize); }
  int get_blob_by_id_and_layer_name (std::string name, int blob_id, double* ret, int osize) { return ec->get_blob_by_id_and_layer_name(name,blob_id,ret,osize); }
  int get_ip_layer_blob (int blob_id, double* ret, int osize) { return ec->get_ip_layer_blob(blob_id,ret,osize); }
  //
  int create_solver (char* solver_path, char* solverstate){ return ec->create_solver(solver_path,solverstate); }
  int initialize_solver (int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy){ return ec->initialize_solver(isize,dsize,idata,ddata,idummy,ddummy); }
  double caffe_learn () { return ec->caffe_learn(); }
  int calc_forward (int inputsize, int outputsize, double* input, double* output, double* idummy){ return ec->calc_forward(inputsize,outputsize,input,output,idummy);}
}
