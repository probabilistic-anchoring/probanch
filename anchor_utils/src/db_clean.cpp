
#include <anchor_utils/database.hpp>

using namespace mongo;

// --[ Main fn ]--- 
int main(int argc, char* argv[]) { 

  // Parse the command line arguments
  if( argc != 2 ) {
    std::cout << "Usage: rosrun " << argv[0] << " <database>" << std::endl;
    return 1;
  }

  // Get database name...
  char ch;
  std::string db_name = argv[1];
  std::cout << "MondoDB: all anchors from database '"<< db_name << "' will be removed."<< std::endl;
  std::cout << "Do you want to continue? [Y/n] ";
  std::cin.get(ch);  
  if( !(ch == 'Y' || ch == 'y' || ch == '\n') )
    return 0;

  // Get a datbase instance
  try {
    Database db( db_name, "anchors");
  
    // Query and remove ids
    std::vector<std::string> ids;
    db.id_array(ids);
    for( auto id : ids ) {
      db.remove(id);
    }
    std::cout << "Successfully removed " << ids.size() << " anchor(s)." << std::endl; 
    db.set_collection( "predicates" );
    db.id_array(ids);
    for( auto id : ids ) {
      db.remove(id);
    }
  }
  catch( const std::exception &e) {
    cout << e.what() << endl;
  }
  return 0;
}
