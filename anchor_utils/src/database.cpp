
#include <anchor_utils/database.hpp>

namespace mongo {

  using namespace std;
  using namespace mongocxx;
  using namespace bsoncxx;

  // Constructor
  Database::Database( const std::string &db, const std::string &collection ) :
    _db(db),
    _collection(collection)
  {
    _client_ptr = std::unique_ptr<mongocxx::client>(new mongocxx::client(mongocxx::uri{}));
    auto database = _client_ptr->database(db);
    _coll_ptr = mongocxx::stdx::make_unique<mongocxx::collection>(database[collection]);
  }
  
  // Document constructor
  Database::Document::Document(const std::string &id) {
    using builder::stream::document;
    using builder::stream::finalize;
    if( !id.empty() ) {
      _doc = document{} << "_id" << bsoncxx::oid{ id } << finalize;
    }
    else {
      _doc = document{} << finalize;
    }
  }

  // Copy constructor
  Database::Document::Document(const Database::Document &other) {
    *this = other;
  }
  
  // Copy assignment operator 
  Database::Document& Database::Document::operator=(const Database::Document &other) {
    using builder::stream::document;
    using builder::stream::finalize;
    using bsoncxx::builder::concatenate;
    if (this != &other) {
      this->_doc = document{} << concatenate(other._doc->view()) << finalize;
      for( const auto& el : other._sub_doc ) {
	this->_sub_doc[ el.first ] = el.second;
      }
      for( const auto& el : other._sub_array ) {
	this->_sub_array[ el.first ] = el.second;
      }
    }
    return *this;
  }
  /*
  Database::Document& Database::Document::operator=(Database::Document &&other) {
    if (this != &other) {
      _doc = std::move(other._doc);
      for( const auto& el : other._sub_doc ) {
	this->_sub_doc[ el.first ] = std::move(el.second);
      }
      for( const auto& el : other._sub_array ) {
	this->_sub_array[ el.first ] = std::move(el.second);
      }
    }
    return *this;
  }
  */

  // Change the collection
  void Database::set_collection(const std::string &collection) {
    _collection = collection;
    auto database = _client_ptr->database(_db);
    _coll_ptr = mongocxx::stdx::make_unique<mongocxx::collection>(database[collection]);
  }

  // Get an array of ids (from current active collection).
  void Database::id_array( vector<std::string> &array, int limit ) {

    // Add options
    mongocxx::options::find opts;
    if( limit >= 0 ) {
      opts.limit( limit );
    }   
    // Make the query
    auto cursor = _coll_ptr->find( {}, opts);
    for (auto&& view: cursor) {
      bsoncxx::oid oid = view["_id"].get_oid().value;
      array.push_back(oid.to_string());
    }
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


  // ----------------------------------
  // Sub- document methods
  // ---------------------------------------

  // Return an nested sub-document
  Database::Document Database::Document::get(const std::string &key) const {
    if( _sub_doc.find(key) == _sub_doc.end() ) {
      cout << "[mongocxx::warning]: there exist no sub-document with key '" + key +"', result document will be empty." << endl;
      return Database::Document();
    }
    return _sub_doc.at(key);
  }

  
  // Return an iterator pointer to a nested sub-array
  vector<Database::Document>::const_iterator Database::Document::begin(const std::string &key) const {
    if( _sub_array.find(key) == _sub_array.end() ) {
      throw std::logic_error("[mongocxx::error]: there exist no sub-array with key '" + key +"'.");
    }
    return _sub_array.at(key).begin();
  }

  // ...corresponding end iterator pointer
  vector<Database::Document>::const_iterator Database::Document::end(const std::string &key) const {
    if( _sub_array.find(key) == _sub_array.end() ) {
      throw std::logic_error("[mongocxx::error]: there exist no sub-array with key '" + key +"'.");
    }
    return _sub_array.at(key).end();
  }
  
  // Access a sub-array
  const vector<Database::Document> &Database::Document::access(const std::string &key) {
    if( _sub_array.find(key) == _sub_array.end() ) {
      throw std::logic_error("[mongocxx::error]: there exist no sub-array with key '" + key +"'.");
    }
    return _sub_array[key];
  }

  // Check if a an element or sub-array exists
  bool Database::Document::exist(const std::string &key) {
    if( _sub_array.find(key) != _sub_array.end() ) {
      return true;
    }
    return false;
  }

  
  // Seralize the document structure and return a database document value
  document::value Database::Document::serialize() {
    using builder::stream::document;
    using builder::stream::finalize;
    using bsoncxx::builder::concatenate;
    auto builder = document{};
    
    // Add all sub documents
    if( !_sub_doc.empty() ) {
      using builder::stream::open_document;
      using builder::stream::close_document;
      for( auto &doc : _sub_doc ) {
	builder << doc.first << open_document << concatenate(doc.second.serialize().view()) << close_document;
      }
    }

    // Add all sub arrays
    if( !_sub_array.empty() ) {    
      using builder::stream::open_array;
      using builder::stream::close_array;
      for( auto &doc : _sub_array ) {
	auto arr = builder << doc.first << open_array;
	for( auto &el : doc.second ) {
	  arr << concatenate(el.serialize().view());
	}
	arr << close_array;
      }
    }

    auto result = document{} << concatenate(_doc->view()) << concatenate(builder.view()) << finalize;
    return result;
  }

  // Recursivly deseralize the document structure
  void Database::Document::deserialize() {
    for( auto &el : _doc->view() ) {
      if( el.type() == type::k_document ) {
	Database::Document doc(document::value(el.get_document()));
	doc.deserialize();
	this->_sub_doc[std::string(el.key())] = doc;
      }
      else if( el.type() == type::k_array ) {
	
	bsoncxx::array::view subarr{el.get_array().value};
	for( bsoncxx::array::element subdoc : subarr ) {
	  if( subdoc.type() != type::k_document ) {
	    break;
	  }
	  Database::Document doc(document::value(subdoc.get_document()));
	  doc.deserialize();
	  this->_sub_array[std::string(el.key())].push_back(doc);
	}      
      }
    }
  }

  // Extract a sub-document
  Database::Document Database::get(const std::string &id) const {
    auto result = this->get_document(id);
    if( result ) {
      return Database::Document(*result);
    }
    return Database::Document();
  }  


  // ----------------------
  // Insert method
  // --------------------------------
  // Return: '_id' of inserted document
  // --------------------------------------
  std::string Database::insert(Database::Document &doc) {
    std::string result = "";
    try {
      auto response = this->_coll_ptr->insert_one( doc.serialize().view() );
      if( !response ) {
	throw std::logic_error("[mongocxx::error]: could not insert document.");
      }
      if( response->inserted_id().type() == bsoncxx::type::k_oid ) {
	bsoncxx::oid oid = response->inserted_id().get_oid().value;
	result = oid.to_string();
      }    
    }
    catch(const mongocxx::bulk_write_exception& e) {
      throw std::logic_error("[mongocxx::error]: a document with given _id already exists (try to update instead)."); 
    }
    return result;
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
  
  bsoncxx::document::value Database::get_query(const std::string &id) const {
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
      throw std::logic_error("[mongocxx::error]: could not find element '" + key + "' in doccument id: '" + get_id(view) + "'");
    }
    return el;
  }

  mongocxx::stdx::optional<bsoncxx::document::value> Database::get_document(const std::string &id) const {
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
    if( result == nullptr ) {
      cout << "Getting shitty binary." << endl;
    }
    return result;
  }

  // Get value - type: bool (true/false)
  template<> bool Database::get_value<bool>( const bsoncxx::types::value &val, 
					     const std::string &key ) {
    if( val.type() == type::k_bool ) {
      return val.get_bool().value;
    }
    else {
      cout << "[mongocxx::warning]: value for element '" + key + "' is of type '" + 
	bsoncxx::to_string(val.type()) + "', which is different from standard type 'bool'" +
	" - default 'false' will be returned." << endl;
    }
    return false;
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
  void Database::Document::add<unsigned char*>(const std::string &key, unsigned char* val, std::size_t length) {
    using builder::stream::document;
    using builder::stream::finalize;
    using bsoncxx::builder::concatenate;
    bsoncxx::types::b_binary array;
    array.bytes = val;
    array.size = length;
    _doc = document{} << concatenate(_doc->view()) << key << array << finalize;
  }

  // Update value - type: unsigned char* (binary data)
  template<> 
  void Database::update<unsigned char*>( const std::string &id, 
					const std::string &key, 
					unsigned char* val, 
					std::size_t length ) {
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::stream::open_document;
    using builder::stream::close_document;
    bsoncxx::types::b_binary array;
    array.bytes = val;
    array.size = length;
    _coll_ptr->update_one( this->get_query(id),
			   document{} << "$set" << open_document << 
			   key << array << close_document << finalize );    
  }

  // Update a document (recursivly)
  template<> 
  void Database::update<Database::Document>( const std::string &id, 
					     const std::string &key, 
					     Database::Document val, std::size_t length ) {
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::stream::open_document;
    using builder::stream::close_document;
    using builder::concatenate;
    
    auto el = val.serialize();
    if( !el.view().empty() ) {
      _coll_ptr->update_one( this->get_query(id),
			     document{} << "$set" << open_document << 
			     key << concatenate(el.view()) << close_document << finalize );    
    }
    else {
      cout << "[mongocxx::warning]: sub- document is empty, nothing will be updated." << endl;
    }
  }

  // Update an array of documents (recursivly)
  template<> 
  void Database::update<Database::Document>( const std::string &id, 
					     const std::string &key, 
					     vector<Database::Document> &array ) {

    using builder::stream::document;
    using builder::stream::finalize;
    using builder::stream::open_document;
    using builder::stream::close_document;
    using builder::concatenate;
    
    if( !array.empty() ) {
      auto builder = document{};
      auto arr = builder << key << builder::stream::open_array;
      for( auto &el : array) {
	arr << el.serialize().view();
      }
      arr << builder::stream::close_array;
      auto doc = builder << finalize;

      _coll_ptr->update_one( this->get_query(id),
			     document{} << "$set" << open_document << 
			     concatenate(doc.view()) << close_document << finalize ); 
    }
    else {
      cout << "[mongocxx::warning]: sub- array is empty, nothing will be updated." << endl;
    }
  }


} // namespace 'mongo'
