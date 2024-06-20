#ifndef _GETDIRFROMPATH_H_
#define _GETDIRFROMPATH_H_

#include <sstream>
#include <iostream>
#include <boost/filesystem.hpp>

static std::vector<std::string> getDirFromPath(const std::string& path_to_folder)
{
  std::vector<std::string> dir_list;
  using namespace std;
  using namespace boost::filesystem;

  path p (path_to_folder);   // p reads clearer than argv[1] in the following code
  try
  {
    if (exists(p)) {   // does p actually exist?
      if (is_regular_file(p))        // is p a regular file?
        std::cout << p << " size is " << file_size(p) << '\n';
      else if (is_directory(p)) {      // is p a directory?
        //std::cout << p << " is a directory containing:\n";
        typedef std::vector<path> vec;             // store paths,
        vec v;                                // so we can sort them later
        copy(directory_iterator(p), directory_iterator(), back_inserter(v));
        sort(v.begin(), v.end());             // sort, since directory iteration
        // is not ordered on some file systems
        for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it) {
          std::stringstream ss;
          ss << *it;
          std::string file_dir = ss.str();
          file_dir.erase(remove(file_dir.begin(), file_dir.end(), '"'), file_dir.end()); //remove " from string
          dir_list.push_back(file_dir);
        }
      }
      else
        std::cout << p << " exists, but is neither a regular file nor a directory\n";
    }
    else
      std::cout << p << " does not exist\n";
  }

  catch (const filesystem_error& ex) {
    std::cout << ex.what() << '\n';
  }

  return dir_list;
}

#endif