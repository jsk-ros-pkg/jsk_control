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
  boost::shared_ptr<caffe::Solver<double>> solver;
  boost::shared_ptr<caffe::Net<double>> test_net;

public:
  template <typename T> bool _check(boost::shared_ptr<T> val, std::string name){
    if (! val) {
      std::cout << name << " is NULL" << std::endl;
      return false;
    } else {
      return true;
    }
  }

  int get_blob_data (boost::shared_ptr<caffe::Blob<double> > blob, double* ret, int osize) {
    if ( ! this->_check(blob, "blob") ) return -1 ;
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
    if ( ! this->_check(this->solver, "solver") ) return -1 ;
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    return this->get_blob_data( net, name, ret, osize);
  }

  int get_blob_data (boost::shared_ptr<caffe::Net<double>> net, char* name, double* ret, int osize) {
    if ( ! this->_check(net, "net") ) return -1 ;
    return this->get_blob_data( net->blob_by_name(name), ret, osize);
  }

  int get_input_blob_data(int id, double* ret, int osize){
    if ( ! this->_check(this->solver, "solver") ) return -1 ;
    if ( id >= this->solver->net()->input_blobs().size() ){
      std::cout << "too large id for input layer "
		<< id << " > " << this->solver->net()->input_blobs().size()
		<< std::endl;
      return 1;
    }
    return this->get_blob_data( boost::shared_ptr<caffe::Blob<double> >(this->solver->net()->input_blobs()[id]), ret ,osize);
  }

  int get_layer_blob_data (char* name, int blob_id, double* ret, int osize) {
    if ( ! this->_check(this->solver, "solver") ) return -1 ;
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    const boost::shared_ptr<caffe::Layer<double>> layer = net->layer_by_name(name);
    std::vector<boost::shared_ptr<caffe::Blob<double>>> ip_blobs = layer->blobs();
    std::cout << "<" << layer->type() << ">" << name << "[" << blob_id << "<" << ip_blobs.size() << "]" ;
    if ( ip_blobs.size() <= blob_id ){
      std::cout << std::endl;
      std::cout << " --- too large blob_id" << std::endl;
      return -1;
    }
    return this->get_blob_data(ip_blobs[blob_id], ret, osize);
  }

  int create_solver (char* solver_path, char* solverstate){
    struct stat buf;
    if (solver_path && stat(solver_path, &buf)==0){
      caffe::ReadProtoFromTextFileOrDie(solver_path, &this->solver_param);
      if ( this->solver ) this->solver.reset();
      this->solver = boost::shared_ptr<caffe::Solver<double>>(caffe::GetSolver<double>(this->solver_param));
      if (solverstate && stat(solverstate, &buf)==0) {
	std::cout << "Restoring previous solver status from " << solverstate << std::endl;
	this->solver->Restore(solverstate);
      }
    }
    return 0;
  }

  int reset_memory_layer(char* name, int size, double* data, double* label){
    if ( ! this->_check(this->solver, "solver") ) return -1 ;
    //
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    const boost::shared_ptr<caffe::Layer<double>> layer_org = net->layer_by_name(name);
    if ( ! this->_check(layer_org, name) ) return 1;
    if ( std::string(layer_org->type()) == "MemoryData" ) {
      boost::shared_ptr<caffe::MemoryDataLayer<double>> layer =
	boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(layer_org);
      layer->Reset(data, label, size);
    } else {
      std::cout << " reset_memory failed, type = " << layer_org->type() << std::endl;
    }
    //
    return 0;
  }

  // deprecated
  int initialize_solver (int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy){
    return this->reset_memory_layer((char*)"input", isize, idata, idummy) + this->reset_memory_layer((char*)"target", dsize, ddata, ddummy);
  }

  double caffe_learn () {
    if ( ! this->_check(this->solver, "solver") ) return -1 ;
    //
    // caffe::set_phase(caffe::TRAIN);
    this->solver->Solve();
    //
    double buf[1];
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    this->get_blob_data(net, (char*)"loss", buf, 1);
    return buf[0];
  }

  int memory_calc_forward (int inputsize, int outputsize, double* input, double* output, double* idummy){
    if ( ! this->_check(this->solver, "solver") ) return 1 ;
    boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    const boost::shared_ptr<caffe::Layer<double>> layer_org = net->layer_by_name("input");
    if ( ! this->_check(layer_org, "input_layer") ) return 1;
    if ( ! (std::string(layer_org->type()) == "MemoryData") ) return 1;
    boost::shared_ptr<caffe::MemoryDataLayer<double>> input_layer =
      boost::dynamic_pointer_cast<caffe::MemoryDataLayer<double>>(layer_org);
    //
    input_layer->Reset(input, idummy, inputsize);
    net->ForwardPrefilled(nullptr);
    //
    this->get_blob_data(net, (char*)"output", output, outputsize);
    return 0;
  }

  int calc_forward (int num, int channels, int width, int height,
		    int inputsize, char* input,
		    int outputsize, double* output){
    return _calc_forward(num,channels,width,height,inputsize,input,0,NULL,outputsize,output);
  }

  int calc_forward_double (int num, int channels, int width, int height,
			   int inputsize, double* input,
			   int outputsize, double* output){
    return _calc_forward(num,channels,width,height,0,NULL,inputsize,input,outputsize,output);
  }

  int _calc_forward (int num, int channels, int width, int height,
		     int inputsize, char* input,
		     int finputsize, double* finput,
		     int outputsize, double* output){
    if ( ! this->_check(this->test_net, "test_net") ) return -1 ;
    //
    // std::cout << "calc_forward" << std::endl;
    // boost::shared_ptr<caffe::Net<double>> net = this->solver->net();
    caffe::Blob<double>* blob = new caffe::Blob<double>();
    std::vector<caffe::Blob<double>*> bottom;
    caffe::BlobProto blob_proto;
    blob_proto.set_num(num);
    blob_proto.set_channels(channels);
    blob_proto.set_height(height);
    blob_proto.set_width(width);
    if ( inputsize > 0 && input ){ // set data, char input precede
      for ( int i=0 ; i<inputsize ; i++ ) {
	blob_proto.add_data(input[i]);
      }
    } else if ( finputsize > 0 && finput ){
      for ( int i=0 ; i<finputsize ; i++ ) {
	blob_proto.add_data(finput[i]);
      }
    }
    //
    blob->FromProto(blob_proto);
    bottom.push_back(blob);
    // double ret[10]; this->_get_blob_data(boost::shared_ptr<caffe::Blob<double>>(blob), ret, 10);
    // std::cout << " -- initialize done" << std::endl;
    this->test_net->Forward(bottom, nullptr);
    // net->Forward(bottom, nullptr);
    // std::cout << " -- forwarding done" << std::endl;
    //
    this->get_blob_data(this->test_net, (char*)"output", output, outputsize);
    return 0;
  }

  int gen_test_net (char* net_path_, char* train_file){
    struct stat buf;
    const char* net_path ;
    if ( net_path_ && stat(net_path_, &buf)==0) {
      net_path = net_path_;
    } else if ( this->solver && this->solver_param.has_net() && stat(this->solver_param.net().c_str(), &buf)==0) {
      net_path = this->solver_param.net().c_str();
    } else {
      std::cout << " network missing" << std::endl;
      return 1;
    }
    if ( this->test_net ) this->test_net.reset();
    this->test_net = boost::shared_ptr<caffe::Net<double>>(new caffe::Net<double>(net_path, caffe::TEST));
    //
    if (train_file && stat(train_file, &buf)==0){
      this->test_net->CopyTrainedLayersFrom(train_file);
    } else if ( this->solver ) {
      this->test_net->ShareTrainedLayersWith(this->solver->net().get());
    } else {
      std::cout << " no layer params" << std::endl;
      return 1;
    }
    return 0;
  }
};

boost::shared_ptr<eus_caffe> ec(new eus_caffe);

extern "C" {
  int eus_caffe_get_blob_data (char* name, double* ret, int osize){ return ec->get_blob_data(name,ret,osize); }
  int eus_caffe_get_input_blob_data (int id, double* ret, int osize){ return ec->get_input_blob_data(id,ret,osize); }
  int eus_caffe_get_layer_blob_data (char* name, int blob_id, double* ret, int osize) { return ec->get_layer_blob_data(name,blob_id,ret,osize); }
  //
  int eus_caffe_create_solver (char* solver_path, char* solverstate){ return ec->create_solver(solver_path,solverstate); }
  int eus_caffe_reset_memory_layer (char* name, int size, double* data, double* label){ return ec->reset_memory_layer(name,size,data,label); }
  int eus_caffe_initialize_solver (int isize, int dsize, double* idata, double* ddata, double* idummy, double* ddummy){ return ec->initialize_solver(isize,dsize,idata,ddata,idummy,ddummy); }
  double eus_caffe_learn () { return ec->caffe_learn(); }
  int eus_caffe_calc_forward (int num, int channels, int width, int height, int inputsize, char* input, int outputsize, double* output){ return ec->calc_forward(num,channels,width,height,inputsize,input,outputsize,output);}
  int eus_caffe_calc_forward_double (int num, int channels, int width, int height, int inputsize, double* input, int outputsize, double* output){ return ec->calc_forward_double(num,channels,width,height,inputsize,input,outputsize,output);}
  int eus_caffe_memory_calc_forward (int inputsize, int outputsize, double* input, double* output, double* idummy){ return ec->memory_calc_forward(inputsize,outputsize,input,output,idummy);}
  int eus_caffe_gen_test_net (char* net_path, char* train_file){ return ec->gen_test_net(net_path, train_file); }
}
