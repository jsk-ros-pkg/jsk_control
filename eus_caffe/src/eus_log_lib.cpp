#include <iostream>
#include <fstream>
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
}
