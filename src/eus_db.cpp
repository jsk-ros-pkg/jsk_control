#include <iostream>
#include <cassert>

#include <caffe/util/db.hpp>

shared_ptr<db::DB> db;
shared_ptr<db::Transaction> txn;
Datum datum;

extern "C" {
  int db_initialize(char* path){
    if ( db ){
      db.reset(db->NewTransaction());
    } else {
      db = db::GetDB("lmdb");
    }
    db->Open(path, db::NEW);
    if ( txn ) {
      txn.reset(db->NewTransaction());
    } else {
      txn = db->NewTransaction();
    }
  }
}

extern "C" {
  
}
