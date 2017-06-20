#ifndef __DATABASE_HPP__
#define __DATABASE_HPP__

#include <iostream>
#include <vector>
#include <map>
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

  // ---[ Used namespaces ]---
  using namespace std;
  using namespace mongocxx;
  using namespace bsoncxx;
  

  // ---[ Class definition...
  // --------------------------------
  class Database {

    // ---[ Private DB variables ]--- 
    unique_ptr<collection> _coll_ptr;
    unique_ptr<client> _client_ptr;
    std::string _db;
    std::string _collection;

    // ---[ Private functions ]---
    static std::string get_id(document::view &view);
    document::value get_query(const std::string &id) const;
    static document::element get_element(document::view view, const std::string &key);
    mongocxx::stdx::optional<document::value> get_document(const std::string &id) const;

    // ---[ Fn. for accessing specific datatypes ]--- 
    template<typename T> 
    static T get_value(const bsoncxx::types::value &val, const std::string &key) { 
      cout << "[mongocxx::warning]: given datatype is not supported, default value will be returned." << endl;
      return T();
    }
 
  public: 

    // Concstructor and destructor
    Database( const std::string &db, const std::string &collection );
    ~Database() {}

    void set_collection(const std::string &collection);
    template<typename T> std::string get_id( const std::string &key, T val);

    // ---[ Struct for handle a database document...
    // ------------------------------------------------
    struct Document {
      
      // Constructor
      Document(const std::string &id = "");
      Document(const bsoncxx::document::value &doc) : _doc(std::move(doc)) {
	deserialize();
      }
      Document(const Document &other);

      // Copy and move assignment operators
      Document& operator=(const Document &other);
      //Document& operator=(Document &&other);

      template<typename T> void add(const std::string &key, T val, std::size_t length = 0);
      template<typename T> void add(const std::string &key, vector<T> &array);
      void add(const std::string &key, Document &doc) { _sub_doc[key] = doc; }
      void append(const std::string &key, Document &doc) { _sub_array[key].push_back(doc); }

      template<typename T> T get(const std::string &key) const;
      template<typename T> void get(const std::string &key, vector<T> &array) const;
      Document get(const std::string &key) const; 

      vector<Document>::const_iterator begin(const std::string &key) const;
      vector<Document>::const_iterator end(const std::string &key) const;
      const vector<Document> &access(const std::string &key);

      document::value serialize();
      
    private:

      mongocxx::stdx::optional<document::value> _doc;
      map<std::string, Document> _sub_doc;
      map<std::string, vector<Document> > _sub_array;

      void deserialize();
    }; // ...end of struct. ]---


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
    
    // Insert method
    std::string insert(Document &doc);
    
    // Query methods
    Document get(const std::string &id) const;
    template<typename T> T get(const std::string &id, const std::string &key) const;
    template<typename T> void get(const std::string &id, const std::string &key, vector<T> &array) const;

    // Update methods
    template<typename T> void update(const std::string &id, const std::string &key, T val, std::size_t length = 0);
    template<typename T> void update(const std::string &id, const std::string &key, vector<T> &array);
    
    // Remove methods
    void remove(const std::string &id);
    void remove(const std::string &id, const std::string &key); 

  }; // ...end of class. ]---

  
  // ---[ Template specializations headers ]---
  template<> unsigned char* Database::get_value<unsigned char*>( const types::value &val, const std::string &key );
  template<> bool Database::get_value<bool>( const types::value &val, const std::string &key );
  template<> std::size_t Database::get_value<std::size_t>( const types::value &val, const std::string &key );
  template<> int Database::get_value<int>( const types::value &val, const std::string &key );
  template<> double Database::get_value<double>( const types::value &val, const std::string &key );
  template<> std::string Database::get_value<std::string>( const types::value &val, const std::string &key );

  template<> void Database::Document::add<unsigned char*>(const std::string &key, unsigned char* val, std::size_t length);

  template<> void Database::update<unsigned char*>( const std::string &id, 
						    const std::string &key, 
						    unsigned char* val, 
						    std::size_t length );
  template<> void Database::update<Database::Document>( const std::string &id, 
							const std::string &key, 
							Database::Document val, 
							std::size_t length );
  template<> void Database::update<Database::Document>( const std::string &id, 
						        const std::string &key, 
							vector<Database::Document> &array );
  

  // ---[ Template methods ]---

  // Template method for query a single id
  template<typename T> std::string Database::get_id( const std::string &key, T val) {
    using builder::stream::document;
    auto filter = document{};
    filter << key << val;
    auto result = _coll_ptr->find_one( filter.view() );
    if( result ) {
      bsoncxx::oid oid = result->view()["_id"].get_oid().value;
      return oid.to_string();
    }
    return "";
  }

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
  void Database::Document::add(const std::string &key, T val, std::size_t) {
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::concatenate;
    _doc = document{} << concatenate(_doc->view()) << key << val << finalize;
  }

  // Add an array to an insert document  
  template<typename T> 
  void Database::Document::add(const std::string &key, vector<T> &array) {
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::concatenate;

    auto  builder = document{};
    auto arr = builder << key << builder::stream::open_array;
    for( auto &el : array) {
      arr << el;
    }
    arr << builder::stream::close_array;
    auto doc = builder << finalize;
    _doc = builder << concatenate(_doc->view()) << concatenate(doc.view()) << finalize;
  }
  

  // ---------------------------
  // Query methods
  // --------------------------------

  // Get a single value from a document given by id.
  template<typename T> 
  T Database::get(const std::string &id, const std::string &key) const {
    mongocxx::stdx::optional<bsoncxx::document::value> doc = 
      this->get_document(id);
    bsoncxx::document::element el = get_element( doc->view(), key );
    return this->get_value<T>(el.get_value(), key);
  }
  
  // Get an array from a document given by id.
  template<typename T> 
  void Database::get(const std::string &id, const std::string &key, vector<T> &array) const { 
    mongocxx::stdx::optional<bsoncxx::document::value> doc = 
      this->get_document(id);
    bsoncxx::document::element el = get_element( doc->view(), key);
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


  // ----------------------------------
  // Sub- document query methods
  // ---------------------------------------

  // Get a single value from a document.
  template<typename T> 
  T Database::Document::get(const std::string &key) const {
    document::element el = get_element( _doc->view(), key );
    return get_value<T>(el.get_value(), key);
  }

  // Get an array from a document.
  template<typename T> 
  void Database::Document::get(const std::string &key, vector<T> &array) const {
    bsoncxx::document::element el = get_element( _doc->view(), key);
    if( el.type() == type::k_array ) {
      bsoncxx::array::view subarr{el.get_array().value};
      for( bsoncxx::array::element el : subarr ) {
	array.push_back( get_value<T>( el.get_value(), key) );
      }      
    }
    else {
      cout << "[mongocxx::warning]: element with key: '" + key + "', is not an array." << endl;
    }
  }


  // ---------------------------
  // Update methods
  // --------------------------------

  // Update a single value
  template<typename T> 
  void Database::update(const std::string &id, const std::string &key, T val, std::size_t length) {
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::stream::open_document;
    using builder::stream::close_document;
    _coll_ptr->update_one( this->get_query(id),
			   document{} << "$set" << open_document << 
			   key << val << close_document << finalize );    
  }

  // Update an array
  template<typename T> 
  void Database::update(const std::string &id, const std::string &key, vector<T> &array) {
    using builder::stream::document;
    using builder::stream::finalize;
    using builder::stream::open_document;
    using builder::stream::close_document;
    using builder::concatenate;

    auto builder = document{};
    auto arr = builder << key << builder::stream::open_array;
    for( auto &el : array) {
      arr << el;
    }
    arr << builder::stream::close_array;
    auto doc = builder << finalize;

    _coll_ptr->update_one( this->get_query(id),
			   document{} << "$set" << open_document << 
			   concatenate(doc.view()) << close_document << finalize ); 
  }


  // ---[ Type defines ]--- 
  typedef std::vector<Database::Document>::const_iterator document_iterator;


} // namespace 'mongo'

#endif // __DATABASE_HPP__
