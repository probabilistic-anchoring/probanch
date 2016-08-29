#ifndef __MONGO_DATABASE_HPP__
#define __MONGO_DATABASE_HPP__

#include <vector>
#include <iostream>
#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>

#include <opencv2/core/core.hpp>

#define DB_SCOPED true

using namespace std;
using namespace mongo;

class MongoDatabase {
  string db;
  string collection;

  // MongoDB client
  #if DB_SCOPED
  mongo::ScopedDbConnection *con;
  #else
  DBClientConnection con;
  #endif

  // Private functions
  BSONObj rawQuery(Query query, BSONObj* fields);
  auto_ptr<DBClientCursor> rawCursor(Query query, BSONObj* fields, int limit = 0);
  BSONObj rawDistinct(const string &distinct, Query query);
  Query queryBuilder(const vector<string> &fields, const vector<string> &params);
  void rawUpdate(Query query, BSONObj obj);

  // Result query object
  BSONObj getObj;

  // Insert query builder object
  BSONObj insObj;
public:
  MongoDatabase( const string &collection, 
		 const string &db = "test");
  ~MongoDatabase();
  void switchCollection(const string &collection);
  void close();
  int size();
  int count();

  // Class for handle nested sub-documents  
  class Subdoc {
    vector<BSONObj> array;
    BSONObj obj;
  public:
    Subdoc() {}
    Subdoc(const BSONElement &element); 
    ~Subdoc() { this->array.clear(); }
    Subdoc& operator=(const Subdoc &other);    

    template <typename T> void add(const string &name, const T &value);
    template <typename T> void add(const string &name, const vector<T> &array);
    void add( const string &name, Subdoc &sub);
    void addBinary(const string &name, unsigned char* values, int length = 1);
    template <typename T> T get(const string &field) const;
    template <typename T> void get(const string &field, vector<T>& array) const;
    void get( const string &field, Subdoc &sub) const;
    void getBinary(const string &field, unsigned char* array) const;
    vector<BSONObj> get() { return this->array; }
    void push();
    void pop();
    bool empty() { return this->array.empty(); }
  };

  // Get id from MongoDB.
  template <typename T> string getId(const string &field, const T &param);
  template <typename T> void getIdArray(const string &field, const T &param, vector<string> &array);
  void getIdAll(vector<string> &array, int limit = 0);
  void getIdArrayExists( const string &field, bool exists, vector<string> &array);
  string getIdBinary(const string &field, unsigned char* array, int length = 1);

  // Check if a field-value pair exists.
  template <typename T> bool exists(const string &field, const T &param);

  // Get single values or arrays from given field.
  template <typename T> T get(const string &id, const string &field);
  template <typename T> void get(const string &id, const string &field, vector<T>& array);
  void get(const string &id, const string &field, MongoDatabase::Subdoc &sub);
  unsigned char getBinary(const string &id, const string &field);
  void getBinary(const string &id, const string &field, unsigned char* array);

  // Get single values or arrays stored result.
  void prepareGet(const string &id);
  template <typename T> T get(const string &field) const;
  template <typename T> void get(const string &field, vector<T>& array) const;
  void get(const string &field, MongoDatabase::Subdoc &sub) const;  
  unsigned char getBinary(const string &field) const;
  void getBinary(const string &field, unsigned char* array) const;
  void clear();

  // Generate an unique id
  static string generateId();

  // Insert single values or arrays
  void prepareInsert();
  void prepareInsert(const string &id);
  template <typename T> void add(const string &name, const T &value);
  template <typename T> void add(const string &name, const vector<T> &array);
  void addBinary(const string &name, unsigned char* values, int length = 1);
  void add(const string &name, MongoDatabase::Subdoc &sub);
  string insert();

  // Remove collection by id
  void remove(const string &id);

  // Remove field from collection
  void removeField(const string &id, const string &field);

  // Update colletion by id
  template <typename T> void update( const string &id,
				     const string &field, 
				     const T &param );
  template <typename T> void update( const string &id,
				     const string &field, 
				     const vector<T> &array );
  void update( const string &id, const string &field, MongoDatabase::Subdoc &sub );

  // Aggregate functions
  template <typename T> T min(const string &field);
  template <typename T> T max(const string &field);

  // Getters
  std::string getDb() { return this->db; }
  std::string getCollection() { return this->collection; }
};


// ----------------------------------------
// Public generic template functions.
// ----------------------------------------
template <typename T> string MongoDatabase::getId(const string &field, const T &param) {
  BSONObj fieldObj = BSONObjBuilder().append("_id", 1).obj();
  return this->rawQuery( QUERY(field<<param).sort("_id"), &fieldObj )["_id"].OID().str();
}

template <typename T> void MongoDatabase::getIdArray(const string &field, const T &param, vector<string> &array) {
  BSONObj fieldObj = BSONObjBuilder().append("_id", 1).obj();
  auto_ptr<DBClientCursor> cursor = this->rawCursor( QUERY(field<<param), 
						     &fieldObj );
  while( cursor->more() ) {
    BSONObj resultObj = cursor->next();
    array.push_back(resultObj["_id"].OID().str());
  }
}

template <typename T> bool MongoDatabase::exists(const string &field, const T &param) {
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  return !this->rawQuery( QUERY(field<<param), &fieldObj ).isEmpty();
}

// Get directly from database.
template <typename T> T MongoDatabase::get(const string &id, const string &field) {
  T result;
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  BSONObj resultObj = this->rawQuery( QUERY("_id"<<OID(id)), &fieldObj );

  // Handles dot in field name
  if( resultObj.hasElement(field) ) {
    resultObj.getField(field).Val(result);
  }
  else {
    resultObj.getFieldDotted(field).Val(result);
  }
  return result;
}

template <typename T> void MongoDatabase::get( const string &id, 
					       const string &field, 
					       vector<T>& array ) {
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  BSONObj resultObj = this->rawQuery( QUERY("_id"<<OID(id)), &fieldObj);
  if( resultObj[field].type() == Array ) {
    vector<BSONElement> elementArray = resultObj[field].Array();
    vector<BSONElement>::iterator ite;
    for( ite = elementArray.begin(); ite != elementArray.end(); ++ite) {
      T element;
      (*ite).Val(element);
      array.push_back(element);
    }
  }
}

// Get from local result.
template <typename T> T MongoDatabase::get(const string &field) const {
  T result;
  if( !this->getObj.isEmpty() ) {
    // Handles dot in field name
    if( this->getObj.hasElement(field) ) {
      this->getObj.getField(field).Val(result);
    }
    else {
      this->getObj.getFieldDotted(field).Val(result);
    }
  }
  return result;
}

template <typename T> void MongoDatabase::get( const string &field, vector<T>& array ) const {
  if( !this->getObj.isEmpty() ) {
    if( this->getObj[field].type() == Array ) {
      vector<BSONElement> elementArray = this->getObj[field].Array();
      vector<BSONElement>::const_iterator ite;
      for( ite = elementArray.begin(); ite != elementArray.end(); ++ite) {
	T element;
	(*ite).Val(element);
	array.push_back(element);
      }
    }
  }
}

template <typename T> void MongoDatabase::add(const string &name, const T &value) {
  insObj = BSONObjBuilder().appendElements(insObj).append( name, value).obj();
}

template <typename T> void MongoDatabase::add(const string &name, const vector<T> &array) {
  BSONArrayBuilder bArray;
  typename vector<T>::const_iterator ite;
  for( ite = array.begin(); ite != array.end(); ++ite) {
    bArray.append(*ite);
  }
  insObj = BSONObjBuilder().appendElements(insObj).append( name, bArray.arr()).obj();
}

template <typename T> void MongoDatabase::update( const string &id,
						  const string &field, 
						  const T &param ) {  
  this->rawUpdate( QUERY("_id"<<OID(id)), 
		   BSON("$set"<<BSON(field<<param)));
}
template <typename T> void MongoDatabase::update( const string &id,
						  const string &field, 
						  const vector<T> &array ) {
  BSONArrayBuilder bArray;
  typename vector<T>::const_iterator ite;
  for( ite = array.begin(); ite != array.end(); ++ite) {
    bArray.append(*ite);
  }
  this->rawUpdate( QUERY("_id"<<OID(id)), 
		   BSON("$set"<<BSON(field<<bArray.arr())));  
}

template <typename T> T MongoDatabase::min(const string &field) {
  T result;
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  BSONObj resultObj = this->rawQuery( Query().sort(field, 1), &fieldObj );

  // Handles dot in filed name.
  if( resultObj.hasElement(field) ) {
    resultObj.getField(field).Val(result);
  }
  return result;
}

template <typename T> T MongoDatabase::max(const string &field) {
  T result;
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  BSONObj resultObj = this->rawQuery( Query().sort(field, -1), &fieldObj );

  // Handles dot in filed name.
  if( resultObj.hasElement(field) ) {
    resultObj.getField(field).Val(result);
  }
  return result;
}

// --------------------------------
// Subdoc public template functions
// --------------------------------
template <typename T> void MongoDatabase::Subdoc::add(const string &name, const T &value) {
  this->obj = BSONObjBuilder().appendElements(obj).append( name, value).obj();
}

template <typename T> void MongoDatabase::Subdoc::add(const string &name, const vector<T> &array) {
  BSONArrayBuilder bArray;
  typename vector<T>::const_iterator ite;
  for( ite = array.begin(); ite != array.end(); ++ite) {
    bArray.append(*ite);
  }
  this->obj = BSONObjBuilder().appendElements(obj).append( name, bArray.arr()).obj();
}

template <typename T> T MongoDatabase::Subdoc::get(const string &field) const {
  T result = T();
  if( this->obj.isValid() ) {
    if( this->obj.hasField(field) ) {
      this->obj.getField(field).Val(result);
    }
  }
  return result;
}

template <typename T> void MongoDatabase::Subdoc::get( const string &field, 
						       vector<T>& array ) const {
  if( this->obj.isValid() ) {
    if( this->obj[field].type() == Array ) {
      std::vector<BSONElement> elementArray = this->obj[field].Array();
      std::vector<BSONElement>::iterator ite;
      for( ite = elementArray.begin(); ite != elementArray.end(); ++ite) {
	T element;
	(*ite).Val(element);
	array.push_back(element);
      }
    }
  }
}

#endif // __MONGO_DATABASE_HPP__

