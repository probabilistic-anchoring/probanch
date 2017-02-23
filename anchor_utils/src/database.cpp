
#include <anchor_utils/database.hpp>

namespace mongo {

  using namespace std;
  using namespace mongocxx;
  using namespace bsoncxx;

  Database::Database( const std::string &db, const std::string &collection ) :
    _db(db),
    _collection(collection) //, _active_doc(P_NONE) 
  {
    _client_ptr = std::unique_ptr<mongocxx::client>(new mongocxx::client(mongocxx::uri{}));
    auto database = _client_ptr->database(db);
    _coll_ptr = mongocxx::stdx::make_unique<mongocxx::collection>(database[collection]);
  }

  // ----------------------------
  // Static methods
  // -----------------------------------

  // Overrided methods for query an array of ids
  void Database::id_array( const std::string &db, 
			   const std::string &collection, 
			   vector<std::string> &array, 
			   int limit ) 
  {
    mongocxx::client conn{mongocxx::uri{}};
    auto coll = conn[db][collection];
    
    // Add options
    mongocxx::options::find opts;
    if( limit >= 0 ) {
      opts.limit( limit );
    }

    // Make the query
    auto cursor = coll.find( {}, opts);
    for (auto&& view: cursor) {
      bsoncxx::oid oid = view["_id"].get_oid().value;
      array.push_back(oid.to_string());
    }
  }
  void Database::id_array( const std::string &db, 
			   const std::string &collection, 
			   vector<std::string> &array,
			   const std::string &key,
			   bool exist, 
			   int limit ) 
  {
    mongocxx::client conn{mongocxx::uri{}};
    auto coll = conn[db][collection];
    
    // Add options
    mongocxx::options::find opts;
    if( limit >= 0 ) {
      opts.limit( limit );
    }

    // Make the query
    using builder::stream::document;
    using builder::stream::open_document;
    using builder::stream::close_document;

    auto filter = document{};
    filter << key << open_document << "$exists" << exist << close_document;
    auto cursor = coll.find( filter.view(), opts);
    for (auto&& view: cursor) {
      bsoncxx::oid oid = view["_id"].get_oid().value;
      array.push_back(oid.to_string());
    }
    
  }

  
  // Method for generate and return a unique id
  std::string Database::generate_id() {
    auto oid = bsoncxx::types::b_oid();
    return oid.value.to_string();
  }


  // ----------------------------
  // Insert / Query documents
  // -----------------------------------
 
  void Database::prepare( PrepareType type, const std::string &id) {
    if( type == P_QUERY ) { // Prepare a query result document.
      if( !id.empty() ) {
	this->_query_doc = this->get_document(id);
      }
      else {
	cout << "[mongocxx::warning]: given id is empty, query result will be empty." << endl;
      }
    }
    else if( type == P_SUB_DOC || type == P_SUB_ARRAY ) {
      _active_doc = (type == P_SUB_DOC) ? P_SUB_DOC : P_SUB_ARRAY;
      _sub_doc.clear();
      _sub_array.clear();
    }
    else { // Prepare an insert stream builder document.
      _active_doc = P_INSERT;
      _insert_doc.clear();
      if( !id.empty() ) {
	using builder::stream::document;
	using builder::stream::finalize;
	_insert_doc.push_back(document{} << "_id" <<  bsoncxx::oid{ id } << finalize);
	std::cout << bsoncxx::to_json(_insert_doc.front().view()) << std::endl;
      }
    }
  }

  // Release a query document
  void Database::release() {
    this->_query_doc = mongocxx::stdx::optional<bsoncxx::document::value> (); // Create new empty unique ptr.
  }

  // ----------------------
  // Insert methods
  // -----------------------------
  
  // Append a sub document to a sub array
  void Database::append() {
    using builder::stream::document;
    using builder::concatenate;
    if( _sub_doc.empty() ) {
      cout << "[mongocxx::warning]: sub document is empty, nothing will be appended to sub array." << endl;
    }
    else {
      auto doc = document{};
      for( auto &el : _sub_doc) {
	doc << concatenate(el.view());
      }
      _sub_array.push_back(doc.extract());
      _sub_doc.clear();
    }
  }

  // Insert the actual document.
  // --------------------------------
  // Return: '_id' of inserted document
  // --------------------------------------
  std::string Database::insert() {
    std::string result = "";
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::concatenate;
    auto doc = document{};
    for( auto &el : _insert_doc) {
      doc << concatenate(el.view());
    }
    try {
      auto response = this->_coll_ptr->insert_one( doc.view() );
      if( !response ) {
	throw std::logic_error("[mongocxx::error]: could not insert document.");
      }
      if( response->inserted_id().type() == bsoncxx::type::k_oid ) {
	bsoncxx::oid oid = response->inserted_id().get_oid().value;
	result = oid.to_string();
      }    
    }
    catch(const mongocxx::bulk_write_exception& e) {
      throw std::logic_error("[mongocxx::error]: a document with given _id already exists (try to update instead)."); //  + std::string(e.what())
    }
    return result;
  }  

  // ----------------------------------
  // Sub- document / array access methods
  // ---------------------------------------

  // Iterator methods for accessing sub-array
  inline 
  const mongo_iterator Database::begin(const std::string &key) {
    if( !_query_doc ) {
      throw std::logic_error("[mongocxx::error]: query document must be prepared before used.");
    }
    if( _sub_array.empty() ) {
      extract(key);
    }
    return _sub_array.begin();
  }  
  inline 
  const mongo_iterator Database::end() {
    return _sub_array.end();
  }  

  // Access a sub-document
  inline
  const document::value Database::operator() (const std::string &key) {
    if( !_query_doc ) {
      throw std::logic_error("[mongocxx::error]: query document must be prepared before used.");
    }
    bsoncxx::document::element el = get_element( _query_doc->view(), key );
    if( el.type() != type::k_document ) {
      throw std::logic_error("[mongocxx::error]: sub element '" + key + "' is not a document.");
    }
    return bsoncxx::document::value(el.get_document());
  }

  // Access an indexed element of extracted sub-array
  inline
  const document::value Database::operator() (const std::string &key, std::size_t idx) {
    if( !_query_doc ) {
      throw std::logic_error("[mongocxx::error]: query document must be prepared before used.");
    }
    if( _sub_array.empty() ) {
      extract(key);
    }
    if( !(idx >= 0 && idx < _sub_array.size()) ) {
      throw std::logic_error("[mongocxx::error]: sub array index out of bounds.");
    }
    return _sub_array[idx];
  }

  // Extract a sub-array (from prepared query document).
  void Database::extract(const std::string &key) {
    if( !_query_doc ) {
      throw std::logic_error("[mongocxx::error]: query document must be prepared before used.");
    }
    else {
      bsoncxx::document::element el = get_element( _query_doc->view(), key );
      if( el.type() == type::k_array ) {
	bsoncxx::array::view subarr{el.get_array().value};
	for( bsoncxx::array::element el : subarr ) {
	  if( el.type() == type::k_document ) {
	    _sub_array.push_back(bsoncxx::document::value(el.get_document()));
	  }
	}      
      }
      else {
	cout << "[mongocxx::warning]: element '" + key +"' is not an array, result will be empty.." << endl;
      }
    }
  }


  // --------------------
  // Remove methods
  // ----------------------

  // Remove document by id
  void Database::remove(const std::string &id) {
    _coll_ptr->delete_one( this->get_query(id) );    
  }
  
  // Remove one element from document
  void Database::remove(const std::string &id, const std::string &key) {
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::stream::open_document;
    using builder::stream::close_document;
    _coll_ptr->update_one( this->get_query(id),
			   document{} << "$unset" << open_document << 
			   key << 1 << close_document << finalize );    
  }


  // -----------------------
  // Private functions
  // --------------------------

  std::string Database::get_id(bsoncxx::document::view &view) {
    bsoncxx::oid oid = view["_id"].get_oid().value;
    return oid.to_string();
  }
  
  bsoncxx::document::value Database::get_query(const std::string &id) {
    using builder::stream::document;
    using builder::stream::finalize;
    auto doc = document{} << "_id" << bsoncxx::oid{ id } << finalize;
    return doc;
  }
  
  bsoncxx::document::element Database::get_element(bsoncxx::document::view view, const std::string &key) {
    bsoncxx::document::element el;
    std::stringstream ss(key);
    std::string k;
    while (std::getline(ss, k, '.')) {
      if( el.length() == 0 ) {
	el = view[k];
      }
      else {
	el = el[k];
      }
    }
    
    if( !el ) {
      throw std::logic_error("[mongocxx::error]: could not find element '" + key + "' in doccument id: '" + this->get_id(view) + "'");
    }
    return el;
  }

  mongocxx::stdx::optional<bsoncxx::document::value> Database::get_document(const std::string &id) {
    mongocxx::stdx::optional<bsoncxx::document::value> doc =
      _coll_ptr->find_one( this->get_query(id) );
    if( !doc ) {
      throw std::logic_error("[mongocxx::error]: could not find any doccument with id: '" + id + "'");
    }
    return std::move(doc);
  }


  // ---------------------------------------
  // Template specializations
  // ---------------------------------------

  // Get value - type: unsigned char* (binary data)
  template<> unsigned char* Database::get_value<unsigned char*>( const bsoncxx::types::value &val, 
								  const std::string &key ) {
    static unsigned char* result = nullptr;
    if( val.type() == type::k_binary ) {
      result = (unsigned char*)val.get_binary().bytes;
    }
    else {
      cout << "[mongocxx::warning]: value for element '" + key + "' is of type '" + 
	bsoncxx::to_string(val.type()) + "', which is different from standard type 'unsigned char*'" +
	" - default 'nullptr' will be returned." << endl;
    }
    return result;
  }

  // Get value - type: std::size_t
  template<> std::size_t Database::get_value<std::size_t>( const bsoncxx::types::value &val, 
							    const std::string &key ) {
    std::size_t result = 0;
    if( val.type() == type::k_binary ) {
      result = (std::size_t)val.get_binary().size;
    }
    else if( val.type() == type::k_array ) {
      bsoncxx::array::view subarr{val.get_array().value};
      result = std::distance( std::begin(subarr), std::end(subarr));
    }
    else {
      cout << "[mongocxx::warning]: this function is only used for returning the 'size' of binary data or an sub array, ";
      cout << "but value for element '" + key + "' is of type '" + bsoncxx::to_string(val.type()) + "'";
      cout << " - default 'std::size_t' value will be returned." << endl;
    }
    return result;
  }

  // Get value - type: int
  template<> int Database::get_value<int>( const bsoncxx::types::value &val, 
					   const std::string &key ) {
    int result = 0;
    if( val.type() == type::k_int32 ) {
      result = val.get_int32().value;
    }
    else {
      cout << "[mongocxx::warning]: value for element '" + key + "' is of type '" + 
	bsoncxx::to_string(val.type()) + "', which is different from standard type 'int'" +
	" - default 'int' value will be returned." << endl;
    }
    return result;
  }

  // Get value - type: double
  template<> double Database::get_value<double>( const bsoncxx::types::value &val, 
						  const std::string &key ) {
    double result = 0.0;
    if( val.type() == type::k_double ) {
      result = val.get_double().value;
    }
    else {
      cout << "[mongocxx::warning]: value for element '" + key + "' is of type '" + 
	bsoncxx::to_string(val.type()) + "', which is different from standard type 'double'" +
	" - default 'double' value will be returned." << endl;
    }
    return result;
  }

  // Get value - type: string
  template<> std::string Database::get_value<std::string>( const bsoncxx::types::value &val,
							    const std::string &key ) {
    std::string result = "";
    if( val.type() == type::k_utf8 ) {
      result = val.get_utf8().value.to_string();
    }
    else {
      cout << "[mongocxx::warning]: value for element '" + key + "' is of type '" + 
	bsoncxx::to_string(val.type()) + "', which is different from standard type 'string'" +
	" - default 'string' value will be returned." << endl;
    }
    return result;
  }

  // Add value - type: unsigned char* (binary data)
  template<> 
  void Database::add<unsigned char*>(const std::string &key, unsigned char* val, std::size_t length) {
    using builder::stream::document;
    using builder::stream::finalize;
    bsoncxx::types::b_binary array;
    array.bytes = val;
    array.size = length;
    if( _active_doc == P_INSERT ) {
      _insert_doc.push_back(document{} << key << array << finalize);
    }
    else {
      _sub_doc.push_back(document{} << key << array << finalize);
    }
  }

  // Special case, use the PrepareType to add sub- docs. and arrays.
  template<> 
  void Database::add(const std::string &key, PrepareType val, std::size_t length) {
    using builder::stream::document;
    using builder::concatenate;
    if( val == P_SUB_DOC ) {

      if( _sub_doc.empty() ) {
	cout << "[mongocxx::warning]: sub document is empty, nothing will be added." << endl;
      }
      else {
	auto doc = document{};
	doc << key << builder::stream::open_document;
	for( auto &el : _sub_doc) {
	  doc << concatenate(el.view());
	}
	doc << builder::stream::close_document;
	_insert_doc.push_back(doc.extract());
	_sub_doc.clear();
	_active_doc = P_INSERT;      
      }
    }
    else if( val == P_SUB_ARRAY ){
      if( _sub_array.empty() ) {
	cout << "[mongocxx::warning]: sub array is empty, nothing will be added." << endl;
      }
      else {
	if( !_sub_doc.empty() ) {
	  this->append();
	}

	auto doc = document{};
	auto array =  doc << key << builder::stream::open_array;
	for( auto &el : _sub_array) {
	  array << concatenate(el.view());
	}
	array << builder::stream::close_array;
	_insert_doc.push_back(doc.extract());
	_sub_doc.clear();
	_sub_array.clear();
	_active_doc = P_INSERT;      
      }
    }
    
  }

} // namespace 'mongo'
