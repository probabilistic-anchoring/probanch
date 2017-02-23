#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <type_traits>

#include <bsoncxx/builder/stream/array.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>

#include <bsoncxx/json.hpp>
#include <bsoncxx/stdx/make_unique.hpp>

#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

#include <mongocxx/exception/bulk_write_exception.hpp>

namespace mongo {

  // ---[ USed namespaces ]---
  using namespace std;
  using namespace mongocxx;
  using namespace bsoncxx;

  // ---[ Type defines ]--- 
  typedef std::vector<bsoncxx::document::value>::const_iterator mongo_iterator;

  // ---[ Datatype used for background queries ]---
  enum PrepareType {
    P_QUERY = 0,
    P_INSERT,
    P_SUB_DOC,
    P_SUB_ARRAY,
    P_NONE
  };
  

  // ---[ Class definition...
  // --------------------------------
  class Database {

    // ---[ Private DB variables ]--- 
    unique_ptr<collection> _coll_ptr;
    unique_ptr<client> _client_ptr;
    std::string _db;
    std::string _collection;

    // ---[ Private collections ]--- 
    std::vector<document::value> _insert_doc;
    std::vector<document::value> _sub_doc;
    std::vector<document::value> _sub_array;
    mongocxx::stdx::optional<document::value> _query_doc;
    PrepareType _active_doc;

    // ---[ Private functions ]---
    std::string get_id(document::view &view);
    document::value get_query(const std::string &id);
    document::element get_element(document::view view, const std::string &key);
    mongocxx::stdx::optional<document::value> get_document(const std::string &id);

    // ---[ Fn. for accessing specific datatypes ]--- 
    template<typename T> 
    T get_value(const bsoncxx::types::value &val, const std::string &key) { 
      cout << "[mongocxx::warning]: given datatype is not supported, default value will be returned." << endl;
      return T();
    }

  public: 

    // Concstructor and destructor
    Database( const std::string &db, const std::string &collection );
    ~Database() {}
    
    // Preparing (and releasing) methods
    void prepare( PrepareType type, const std::string &id = "");
    void release();

    // Static id methods
    static void id_array( const std::string &db, 
			  const std::string &collection, 
			  vector<std::string> &array, 
			  int limit = -1 );
    static void id_array( const std::string &db, 
			  const std::string &collection, 
			  vector<std::string> &array,
			  const std::string &key,
			  bool exist = true,
			  int limit = -1 );
    template<typename T>
    static void id_array( const std::string &db, 
			  const std::string &collection, 
			  vector<std::string> &array,
			  const std::string &key, 
			  T val, 
			  int limit = -1 );
    static std::string generate_id();
    

    // Insert methods
    template<typename T> void add(const std::string &key, T val, std::size_t length = 0);
    template<typename T> void add(const std::string &key, vector<T> &array);
    void append();
    std::string insert();

    // Bg. collection access methods
    const mongo_iterator begin(const std::string &key);
    const mongo_iterator end(); 
    const document::value operator() (const std::string &key);
    const document::value operator() (const std::string &key, std::size_t idx);
    void extract(const std::string &key);
    
    // Query methods
    template<typename T> T get(const std::string &id, const std::string &key);
    template<typename T> void get(const std::string &id, const std::string &key, vector<T> &array);
    template<typename T> T get(const document::value &doc, const std::string &key);
    template<typename T> void get(const document::value &doc, const std::string &key, vector<T> &array);
    template<typename T> T get(const std::string &key);
    template<typename T> void get(const std::string &key, vector<T> &array);

    // Remove methods
    void remove(const std::string &id);
    void remove(const std::string &id, const std::string &key); 

  }; // ...end of class. ]---

  
  // ---[ Template specializations headers ]---
  template<> unsigned char* Database::get_value<unsigned char*>( const types::value &val, const std::string &key );
  template<> std::size_t Database::get_value<std::size_t>( const types::value &val, const std::string &key );
  template<> int Database::get_value<int>( const types::value &val, const std::string &key );
  template<> double Database::get_value<double>( const types::value &val, const std::string &key );
  template<> std::string Database::get_value<std::string>( const types::value &val, const std::string &key );

  template<> void Database::add<PrepareType>(const std::string &key, PrepareType val, std::size_t length);
  template<> void Database::add<unsigned char*>(const std::string &key, unsigned char* val, std::size_t length);


  // ---[ Template methods ]---

  // Static template method for query an array of ids
  template<typename T>
  void Database::id_array( const std::string &db, 
			   const std::string &collection, 
			   vector<std::string> &array, 
			   const std::string &key, 
			   T val, 
			   int limit ) 
  {
    if( std::is_same<bool, T>::value ) { // Special case for bool type 
      Database::id_array( db, collection, array, key, val, limit);
    }
    else {
      mongocxx::client conn{mongocxx::uri{}};
      auto coll = conn[db][collection];
    
      // Add options
      mongocxx::options::find opts;
      if( limit >= 0 ) {
	opts.limit( limit );
      }

      // Make the query
      using builder::stream::document;
      auto filter = document{};
      filter << key << val;
      auto cursor = coll.find( filter.view(), opts);
      for (auto&& view: cursor) {
	bsoncxx::oid oid = view["_id"].get_oid().value;
	array.push_back(oid.to_string());
      }
    }
  }

  // -----------------------------
  // Insert methods
  // -----------------------------------

  // Add a value to an insert document
  template<typename T> 
  void Database::add(const std::string &key, T val, std::size_t length) {
    using builder::stream::document;
    using builder::stream::finalize;
    if( _active_doc == P_INSERT ) {
      _insert_doc.push_back(document{} << key << val << finalize);
    }
    else {
      _sub_doc.push_back(document{} << key << val << finalize);
    }
  }

  // Add an array to an insert document  
  template<typename T> 
  void Database::add(const std::string &key, vector<T> &array) {
    auto doc = builder::stream::document{};
    auto insert_array = doc << key << builder::stream::open_array;
    for( auto &el : array) {
      insert_array << el;
    }
    insert_array << builder::stream::close_array;
    if( _active_doc == P_INSERT ) {
      _insert_doc.push_back(doc.extract());
    }
    else {
      _sub_doc.push_back(doc.extract());
    }
  }
  

  // ---------------------------
  // Query methods
  // --------------------------------

  // Get a single value from a document given by id.
  template<typename T> 
  T Database::get(const std::string &id, const std::string &key) {
    mongocxx::stdx::optional<bsoncxx::document::value> doc = 
      this->get_document(id);
    bsoncxx::document::element el = this->get_element( doc->view(), key );
    return this->get_value<T>(el.get_value(), key);
  }
  
  // Get an array from a document given by id.
  template<typename T> 
  void Database::get(const std::string &id, const std::string &key, vector<T> &array) { 
    mongocxx::stdx::optional<bsoncxx::document::value> doc = 
      this->get_document(id);
    bsoncxx::document::element el = this->get_element( doc->view(), key);
    if( el.type() == type::k_array ) {
      bsoncxx::array::view subarr{el.get_array().value};
      for( bsoncxx::array::element el : subarr ) {
	array.push_back( this->get_value<T>( el.get_value(), key) );
      }      
    }
    else {
      cout << "[mongocxx::warning]: element with key: '" + key + "', is not an array." << endl;
    }
  }

  // Get a single value from an argument given sub-document.
  template<typename T> 
  T Database::get(const document::value &doc, const std::string &key) {
    bsoncxx::document::element el = this->get_element( doc.view(), key );
    return this->get_value<T>(el.get_value(), key);
  }  

  // Get an array from an argument given sub-document.
  template<typename T> 
  void Database::get(const document::value &doc, const std::string &key, vector<T> &array) {
    bsoncxx::document::element el = this->get_element( doc.view(), key );
    if( el.type() == type::k_array ) {
      bsoncxx::array::view subarr{el.get_array().value};
      for( bsoncxx::array::element el : subarr ) {
	array.push_back( this->get_value<T>( el.get_value(), key) );
      }      
    }
    else {
      cout << "[mongocxx::warning]: element with key: '" + key + "', is not an array." << endl;
    }
  }

  // Get a single value from prepared query document.
  template<typename T> 
  T Database::get(const std::string &key) {
    if( !_query_doc ) {
      throw std::logic_error("[mongocxx::error]: query document must be prepared before used.");
    }
    bsoncxx::document::element el = get_element( _query_doc->view(), key );
    return this->get_value<T>(el.get_value(), key);
  }

  // Get an array from prepared query document.
  template<typename T> 
  void Database::get(const std::string &key, vector<T> &array) {
    if( !_query_doc ) {
      throw std::logic_error("[mongocxx::error]: query document must be prepared before used.");
    } 
    bsoncxx::document::element el = this->get_element( _query_doc->view(), key);
    if( el.type() == type::k_array ) {
      bsoncxx::array::view subarr{el.get_array().value};
      for( bsoncxx::array::element el : subarr ) {
	array.push_back( this->get_value<T>( el.get_value(), key) );
      }      
    }
    else {
      cout << "[mongocxx::warning]: element with key: '" + key + "', is not an array." << endl;
    }
  }

} // namespace 'mongo'
