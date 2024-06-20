#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <sstream>

#include <boost/filesystem.hpp>

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

using point_t = pcl::PointXYZI;
using point_cloud_ptr_t = pcl::PointCloud<point_t>::Ptr;


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

//X Y Z label
//number of points
struct ASCIIReader {
  static bool read(const std::string& filename, sensor_msgs::PointCloud2& ros_cloud){
    std::ifstream fin;
    fin.open(filename);
    point_cloud_ptr_t cloud (new pcl::PointCloud<point_t>);
    std::string name = filename.substr(filename.find_last_of('/')+1);

    if(!fin.is_open()){
      std::cerr << "ERROR: failed to open file: " << filename << std::endl;
      return false;
    }

    while(!fin.eof()){
      point_t p;
      fin >> p.x >> p.y >> p.z >> p.intensity;
      cloud->push_back(p);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;

    //convert to ros cloud
    pcl::toROSMsg(*cloud, ros_cloud);

    return cloud->size() > 0;
  }
};

static void show_usage(std::string name)
{
  std::cerr << "Usage: " << name << " <option(s)> SOURCES"
            << "Options:\n"
            << "\t-s,--source\t\tScans source dir\n"
            << "\t-d,--dist_dir\t\tDestination directory\n"
            << "\t-o,--bag_name\t\tOutput bag name\n"
            << std::endl;
}

std::string extractID(std::string dir){
  auto f = dir.find_last_of('/');
  auto n = dir.find_last_of('.');
  return dir.substr(f+1, n-f-1); 
}

std::string extractEXT(std::string dir){
  auto n = dir.find_last_of('.');
  return dir.substr(n+1, dir.size()); 
}

void toDir(std::string& f){
  if(f.back() != '/')
    f+='/';
}

ros::Time scanIdToTime(std::string id){
  ros::Time st;
  st.sec = std::stoll(id.substr(0,10));
  if(id.size() == 20)
    st.nsec = std::stoll(id.substr(11,20));
  else
    st.nsec = std::stoll(id.substr(10,19));
  return st;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcd_to_bag");

  std::string source_dir, bag_name, dist_dir;

  if (argc < 6) {
    show_usage(argv[0]);
    exit(EXIT_FAILURE);
  }

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-s") || (arg == "--source")) {
      source_dir = argv[++i];
    } else if ((arg == "-d") || (arg == "--distination")) {
      dist_dir = argv[++i];
    } else if ((arg == "-o") || (arg == "--bag_name")) {
      bag_name = argv[++i];
    }
  }

  std::vector<std::string> scans = getDirFromPath(source_dir);

  std::cout << "Source dir: " << source_dir << std::endl;
  std::cout << "Distination dir: " << dist_dir << std::endl;
  std::cout << "Bag name: " << bag_name << std::endl;

  rosbag::Bag bag;
  toDir(dist_dir);
  bag.open(dist_dir + bag_name + ".bag", rosbag::bagmode::Write);

  //ros::Time::init();

  int counter = 0;

  for(const auto& scan : scans){
    auto scan_id = extractID(scan);
    auto scan_ext = extractEXT(scan);
    ROS_INFO("Processing %s.%s [%d/%d]", scan_id.c_str(), scan_ext.c_str(), ++counter, scans.size());

    // ROS messages
    sensor_msgs::PointCloud2 cloud_;
    if(scan_ext == "asc")
      ASCIIReader::read(scan, cloud_);
    else
      pcl::io::loadPCDFile (scan, cloud_);
    cloud_.header.frame_id = "os_sensor";
    auto stamp = scanIdToTime(scan_id);
    cloud_.header.stamp.sec = stamp.sec;
    cloud_.header.stamp.nsec = stamp.nsec;

    ROS_DEBUG("%d.%d",stamp.sec,stamp.nsec);

    bag.write("/cloud_filtered", stamp, cloud_);

  }
    bag.close();

  return 0;
}