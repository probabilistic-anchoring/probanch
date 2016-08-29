/* ------------------------------------------ 


   ------------------------------------------ */
#include <mongo/client/dbclientinterface.h>

#include <anchoring/database.hpp>

using namespace std;
using namespace mongo;

// Constant values
const std::string MONGO_HOST = "127.0.0.1";

// Constructor
MongoDatabase::MongoDatabase(const string &collection, const string &db) {
  try {

#if DB_SCOPED
    con = ScopedDbConnection::getScopedDbConnection(MONGO_HOST);
#else
    con.connect(MONGO_HOST);
#endif

    this->collection = collection;
    this->db = db;
  }
  catch( DBException &e ) {
    cout << "MondoDB: " << e.what() << endl;
  }
}

// Destructor
MongoDatabase::~MongoDatabase() {
  this->close();
}

// Constructor (sub-class)
MongoDatabase::Subdoc::Subdoc(const BSONElement &element) {
  vector<BSONElement> elementArray = element.Array();
  vector<BSONElement>::iterator ite;
  for( ite = elementArray.begin(); ite != elementArray.end(); ++ite) {
    this->array.push_back(BSONObj(ite->embeddedObject()));
  }
  if( !this->array.empty() ) {
    this->obj = this->array.front();
  }
  else {
    this->obj = BSONObj();
  }
}

MongoDatabase::Subdoc& MongoDatabase::Subdoc::operator=(const MongoDatabase::Subdoc &other) {  
  this->array.resize(other.array.size());
  for( uint i = 0; i < other.array.size(); i++) {
    this->array[i] = other.array[i].copy(); //Note .copy()!!
  }
  if( !this->array.empty() ) {
    this->obj = this->array.front();
  }
  else {
    this->obj = BSONObj();
  }
  return *this;
}

// --------------------
// Public section
// --------------------
void MongoDatabase::switchCollection(const string &collection) {
  //this->collection = MONGO_DB_NAME + "." + collection;
  this->collection = collection;
}

void MongoDatabase::close() {
#if DB_SCOPED
  con->done();
  //while( (*con)->isStillConnected() ) { /* Wait for socket to clean up. */ }
#endif
}

// Get the size of (this) database [kB]
int MongoDatabase::size() {
  int size = 0;
  BSONObj queryResult;
#if DB_SCOPED
  if( (*con)->runCommand( this->db,
			  BSON("dbstats" << 1 << "scale" << 1024 ),
			  queryResult ) ) {
#else
  if( con.runCommand( this->db,
		      BSON("dbstats" << 1 << "scale" << 1024 ),
		      queryResult ) ) {
#endif
    size = queryResult["storageSize"].Int();
  }      
  return size;
}

// Get the size of (this) database [kB]
int MongoDatabase::count() {
  int count = 0;
  BSONObj queryResult;
#if DB_SCOPED
  if( (*con)->runCommand( this->db,
			  BSON("collstats" << this->collection << "scale" << 1 << "verbose" << true ),
			  queryResult ) ) {
#else
  if( con.runCommand( this->db,
		      BSON("collstats" << this->collection << "scale" << 1 << "verbose" << true ),
		      queryResult ) ) {
#endif
    count = queryResult["count"].Int();
  }      
  return count;
}
// -------------------

  void MongoDatabase::getIdAll( vector<string> &array, int limit) {
  BSONObj fieldObj = BSONObjBuilder().append("_id", 1).obj();
  std::auto_ptr<DBClientCursor> cursor = this->rawCursor( Query(), &fieldObj, limit);
  while( cursor->more() ) {
    BSONObj obj = cursor->next();
    array.push_back(obj["_id"].OID().str());
  }
}

void MongoDatabase::getIdArrayExists( const string &field, bool exists, vector<string> &array) {
  BSONObj fieldObj = BSONObjBuilder().append("_id", 1).obj();
  std::auto_ptr<DBClientCursor> cursor = 
    this->rawCursor( QUERY(field<<BSON("$exists"<<exists)), &fieldObj);
  while( cursor->more() ) {
    BSONObj obj = cursor->next();
    array.push_back(obj["_id"].OID().str());
  }
  
}

std::string MongoDatabase::getIdBinary( const string &field,
				        unsigned char* array,
				        int length) {
  BSONObj queryObj = BSONObjBuilder().appendBinData( field, length, BinDataGeneral, array).obj();
  BSONObj fieldObj = BSONObjBuilder().append("_id", 1).obj();
  return this->rawQuery( Query(queryObj).sort("_id"), &fieldObj )["_id"].OID().str();
}				  
// -------------------

void MongoDatabase::get(const string &id, const string &field, MongoDatabase::Subdoc &sub) {
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  BSONObj resultObj = this->rawQuery( QUERY("_id"<<OID(id)), &fieldObj );
  if( resultObj[field].type() == Array ) {
    
    // Note. this is dependent on overloading of the assignment operator!
    sub = MongoDatabase::Subdoc(resultObj[field]);
  }
}

unsigned char MongoDatabase::getBinary(const string &id, const string &field) {
  uchar result = 0x00;
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  BSONObj resultObj = this->rawQuery( QUERY("_id"<<OID(id)), &fieldObj );
  if( resultObj[field].ok() ) {
    int length;
    result =  resultObj[field].binDataClean(length)[0];
  }
  return result;
}

void MongoDatabase::getBinary( const string &id, 
			       const string &field, 
			       unsigned char* array ) {
  BSONObj fieldObj = BSONObjBuilder().append(field, 1).obj();
  BSONObj resultObj = this->rawQuery( QUERY("_id"<<OID(id)), &fieldObj );
  if( resultObj[field].ok() ) {
    int length;
    const char* result = resultObj[field].binDataClean(length);
    memcpy( array, result, length);
  }
}

// Get a document instance and store the result locally.
void MongoDatabase::prepareGet(const string &id) {
  this->getObj = this->rawQuery( QUERY("_id"<<OID(id)), NULL );
}

void MongoDatabase::get(const string &field, MongoDatabase::Subdoc &sub) const {
  if( !this->getObj.isEmpty() ) {
    if( this->getObj[field].type() == Array ) {
      sub = MongoDatabase::Subdoc(this->getObj[field]);
    }
  }
}

unsigned char MongoDatabase::getBinary(const string &field) const {
  uchar result = 0x00;
  if( !this->getObj.isEmpty() ) {
    if( this->getObj[field].ok() ) {
      int length;
      result = this->getObj[field].binDataClean(length)[0];
    }
  }
  return result;
}

void MongoDatabase::getBinary( const string &field, unsigned char* array ) const {
  if( !this->getObj.isEmpty() ) {
    if( this->getObj[field].ok() ) {
      int length;
      const char* result = this->getObj[field].binDataClean(length);
      memcpy( array, result, length);
    }
  }
}

// Clear the exisitng result.
void MongoDatabase::clear() {
  this->getObj = BSONObj();
}

// Function for generating an id
string MongoDatabase::generateId() {
  BSONObj obj = BSONObjBuilder().genOID().obj();
  return obj["_id"].OID().str();
}

// Insert functions
void MongoDatabase::prepareInsert() {
  // Make sure the object is empty 
  insObj = BSONObjBuilder().genOID().obj();
}
void MongoDatabase::prepareInsert(const string &id) {
  insObj = BSONObjBuilder().append( "_id", OID(id)).obj();
}

void MongoDatabase::addBinary(const string &name, uchar* values, int length) {
  insObj = BSONObjBuilder().appendElements(insObj).appendBinData(name, length, BinDataGeneral, values).obj();
}

void MongoDatabase::add(const string &name, MongoDatabase::Subdoc &sub) {
  insObj = BSONObjBuilder().appendElements(insObj).append( name, sub.get()).obj(); 
}

// Return: last inserted id
string MongoDatabase::insert() {
#if DB_SCOPED
  (*con)->insert( this->db + "." + this->collection, insObj);
#else
  con.insert( this->db + "." + this->collection, insObj);
#endif
  return insObj["_id"].OID().str();
}

// Remove by id
void MongoDatabase::remove(const string &id) {
#if DB_SCOPED
  (*con)->remove( this->db + "." + this->collection, QUERY("_id"<<OID(id)));
#else
  con.remove( this->db + "." + this->collection, QUERY("_id"<<OID(id)));
#endif
}

// Remove field from collection by id
void MongoDatabase::removeField(const string &id, const string &field) {
  this->rawUpdate( QUERY("_id"<<OID(id)), 
		   BSON("$unset"<<BSON(field<<1)));
}

// Update a sub-collection
void MongoDatabase::update( const string &id,
			    const string &field, 
			    MongoDatabase::Subdoc &sub ) {
  this->rawUpdate( QUERY("_id"<<OID(id)), 
		   BSON("$set"<<BSON(field<<sub.get())) );
}

// --------------------
// Private section 
// --------------------
BSONObj MongoDatabase::rawQuery(Query query, BSONObj* fields) {
  std::string coll = this->db + "." + this->collection;
#if DB_SCOPED
  BSONObj result = (*con)->findOne( coll, query, fields);
#else
  BSONObj result = con.findOne( coll, query, fields);
#endif
  return result;
}

BSONObj MongoDatabase::rawDistinct(const string &distinct, Query query) {
  BSONObj result;
  BSONObj queryResult;
  std::cout<<BSON( "distinct" << this->collection <<
		   "key" << distinct <<
		   "query" << query)<<std::endl;
#if DB_SCOPED
  if( (*con)->runCommand( this->db,
			  BSON( "distinct" << this->collection <<
				"key" << distinct <<
				"query" << query),
			  queryResult ) ) {
#else
  if( con.runCommand( this->db,
		      BSON( "distinct" << this->collection <<
			    "key" << distinct <<
			    "query" << query),
		      queryResult ) ) {
#endif
    result = queryResult["values"].Obj();
  }
  else {
    throw "MongoDB: no match for given query.";
  }      
  return result;
}

std::auto_ptr<DBClientCursor> MongoDatabase::rawCursor(Query query, BSONObj* fields, int limit) {
  std::string coll = this->db + "." + this->collection;  
  std::auto_ptr<DBClientCursor> cursor;
  try {
#if DB_SCOPED
    cursor = (*con)->query( coll, query, limit, 0, fields);
#else
    cursor = con.query( coll, query, limit, 0, fields);
#endif
  }
  catch( DBException &e ) {
    std::cout << "MondoDB: " << e.what() << std::endl;
  }
  return cursor;
}

Query MongoDatabase::queryBuilder(const vector<string> &fields, 
				  const vector<string> &params) {
  BSONObjBuilder query;
  for( uint i = 0; i < fields.size(); i++) {
    query.append(fields[i], params[i]);
  }
  return Query(query.obj());
}

// Update
void MongoDatabase::rawUpdate(Query query, BSONObj obj) {
#if DB_SCOPED
  (*con)->update( this->db + "." + this->collection,
		  query, obj, false, true);
#else
  con.update( this->db + "." + this->collection,
	      query, obj, false, true);
#endif
}

// --------------------------
// Subdoc public functions
// --------------------------
void MongoDatabase::Subdoc::add( const string &name, Subdoc &sub) {
  this->obj = BSONObjBuilder().appendElements(obj).append( name, sub.get()).obj();   
}

void MongoDatabase::Subdoc::addBinary(const string &name, unsigned char* values, int length) {
  this->obj = BSONObjBuilder().appendElements(obj).appendBinData(name, length, BinDataGeneral, values).obj();
}

void MongoDatabase::Subdoc::get( const string &field, Subdoc &sub) const {
  if( !this->obj.isEmpty() ) {
    if( this->obj[field].type() == Array ) {
      sub = Subdoc(this->obj[field]);
    }
  }
}
 
void MongoDatabase::Subdoc::getBinary(const string &field, unsigned char* array) const {
  if( this->obj[field].ok() ) {
    int length;
    const char* result = this->obj[field].binDataClean(length);
    memcpy( array, result, length);
  }
}

void MongoDatabase::Subdoc::push() {
  this->array.push_back(this->obj);
  this->obj = BSONObj();
}

void MongoDatabase::Subdoc::pop() {
  this->array.erase( this->array.begin() );
  if( !this->array.empty() ) {
    this->obj = this->array.front();
  }
}
