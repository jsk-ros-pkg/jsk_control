#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>

std::shared_ptr<std::streambuf> eus_log_stdcout(std::cout.rdbuf());
std::shared_ptr<std::streambuf> eus_log_stdcerr(std::cerr.rdbuf());
std::shared_ptr<std::ofstream> eus_log_ofs;

extern "C" {
  int log_change_output_stream(char* path, int size){
    if ( path && size > 0 ){
      if ( eus_log_ofs ) {
	eus_log_ofs->close();
	eus_log_ofs.reset(new std::ofstream(path));
      } else {
	eus_log_ofs = std::shared_ptr<std::ofstream>(new std::ofstream(path));
      }
      std::cout.rdbuf(eus_log_ofs->rdbuf());
      std::cerr.rdbuf(eus_log_ofs->rdbuf());
    } else {
      if ( eus_log_ofs ) {
	eus_log_ofs->close();
	eus_log_ofs = NULL;
      }
      std::cout.rdbuf(eus_log_stdcout.get());
      std::cerr.rdbuf(eus_log_stdcerr.get());
    }
    return 0;
  }

  int log_test_echo(char* str){
    std::cout << str << std::endl;
    std::cerr << str << std::endl;
    return 0;
  }

  int size_vector(char* path){
    std::ifstream ifs(path, std::ios::binary);
    std::streamsize s = ifs.seekg(0,std::ios::end).tellg();
    ifs.close();
    return (int)(s/sizeof(double));
  }

  int write_vector(char* path, double* val, int size){
    std::ofstream fout(path, std::ios::out|std::ios::binary|std::ios::trunc);
    if ( ! fout ) {
      std::cout << "file not found " << path << std::endl;
      return 255;
    }
    fout.write((char*)val, size * sizeof(double));
    fout.close();
    return 0;
  }

  int read_vector(char* path, double* val, int size){
    std::ifstream fin(path, std::ios::in|std::ios::binary);
    if ( ! fin ) {
      std::cout << "file not found " << path << std::endl;
      return 0;
    }
    double d;
    for (int i=0; i<size ; i++ ){
      if(fin.eof()){
	std::cout << "size exceeded " << i << ">" << size << std::endl;
	return i;
      }
      fin.read( (char*)&d, sizeof(double) );
      val[i] = d;
    }
    fin.close();
    return size;
  }
}
