#include <iostream>
#include <cassert>
#include <memory>
#include <sys/stat.h>

#include <caffe/util/db.hpp>

class eus_db{
private:
  std::shared_ptr<caffe::db::DB> db_;
  std::shared_ptr<caffe::db::Transaction> txn_;
  caffe::Datum datum_;

public:
  int open(char* db_type, char* path, int m){
    struct stat buf;
    caffe::db::Mode mode = ((m == 'w') ? caffe::db::NEW : caffe::db::READ) ;
    if (mode == caffe::db::NEW && path && stat(path, &buf)==0){
      std::cout << "unable to open file " << path << std::endl;
      return 1;
    }
    //
    if ( this->db_ ){
      this->db_.reset(caffe::db::GetDB(db_type));
    } else {
      this->db_ = std::shared_ptr<caffe::db::DB>(caffe::db::GetDB(db_type));
    }
    this->db_->Open(path, mode);
    //
    if ( this->txn_ ){
      this->txn_.reset(this->db_->NewTransaction());
    } else {
      this->txn_ = std::shared_ptr<caffe::db::Transaction>(this->db_->NewTransaction());
    }
    return 0;
  }

  int put(int chan, int width, int height, int label, char* key, char* data, int data_max){
    this->datum_.set_channels(chan);
    this->datum_.set_height(width);
    this->datum_.set_width(height);
    this->datum_.set_label(label);
    this->datum_.set_data(data, data_max);
    std::string datum_str;
    this->datum_.SerializeToString(&datum_str);
    this->txn_->Put(key, datum_str);
    return 0;
  }

  int close(){
    this->txn_->Commit();
    this->db_->Close();
    return 0;
  }
};

std::shared_ptr<eus_db> ed(new eus_db);

extern "C" {
  int eus_db_open(char* db_type, char* path, int mode){ return ed->open(db_type, path, mode); }
  int eus_db_put(int chan, int width, int height, int label, char* id_str, char* data, int data_max){ return ed->put(chan,width,height,label,id_str,data,data_max); }
  int eus_db_close(){ return ed->close() ; }
}
