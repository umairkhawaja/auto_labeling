#include "featuresGenerator.h"

#define WITH_OCTOMAP true
#define WITHOUT_OCTOMAP false
#define BAG_OCTO true
#define PCD_OCTO false

using namespace std;

#define DEFAULT_DATASET_DIR "/home/ibrahim/icra2022/ktima/"
#define DEFAULT_DEPTH_QUERY 16

struct TArgs{
  bool help=false;
  bool align=false;
  bool features_raw=false;
  bool features_octo=false;
  bool scans=false;
  bool remove_dynamics=false;
  bool remove_ground=false;
  bool add_occluded_points=false;
  int query_depth=DEFAULT_DEPTH_QUERY;
  float threshold=0.0;
  float knn_dist_scan_to_map=0.0;
  float leaf_size=0.0;
  string id="";
  string dataset_dir="";
};

static void show_usage(std::string name)
{
  std::cerr << "Usage: " << name << " <option(s)> SOURCES"
            << "Options:\n"
            << "\t-h,--help\t\tShow this help message\n"
            << "\t-d,--dataset\t\tSpecify the dataset path\n"
            << "\t-a,--align\t\tAlign raw pcds and octos\n"
            << "\t-q,--depth_query\tSpecify octomap depth query\n"
            << "\t-f,--features_raw\tGenerate features without considering occlusions\n"
            << "\t-F,--features_octo\tGenerate features considering occlusions\n"
            << "\t-s,--scans\t\tLabel raw scans"
            << "\t  ,--knn_dist_scan_to_map\tLabel raw scans"
            << "\t-r,--remove_dynamics\tRemove dynamic points from individual scans\n"
            << "\t-t,--threshold\t\tDynamics threshold\n"
            << "\t-o,--add_occluded\t\tAdd occluded points to the labeled scans\n"
            << "\t-m,--map_id\t\tSet Map ID for processing\n"
            << "\t-l,--leaf_size\t\tSet filter leaf size so speed up the registration process\n"
            << "\t  ,--remove_ground\tRemove ground plane from the filtered scans\n"
            << std::endl;
}

TArgs parseArgs(int argc, char* argv[])
{
  TArgs args;
  if (argc < 3) {
    show_usage(argv[0]);
    exit(EXIT_FAILURE);
  }

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if ((arg == "-h") || (arg == "--help")) {
      show_usage(argv[0]);
      exit(EXIT_SUCCESS);
    } else if ((arg == "-d") || (arg == "--dataset")) {
      args.dataset_dir = argv[++i];
    } else if ((arg == "-a") || (arg == "--align")) {
      args.align = true;
    }else if ((arg == "-q") || (arg == "--depth_query")) {
      args.query_depth = stoi(argv[++i]);
    }else if ((arg == "-f") || (arg == "--features_raw")) {
      args.features_raw = true;
    }else if ((arg == "-F") || (arg == "--features_octo")) {
      args.features_octo = true;
    }else if ((arg == "-s") || (arg == "--scans")) {
      args.scans = true;
    }else if ((arg == "-r") || (arg == "--remove_dynamics")) {
      args.remove_dynamics = true;
    }else if ((arg == "-t") || (arg == "--threshold")) {
      args.threshold = stof(argv[++i]);
    }else if ((arg == "-m") || (arg == "--map_id")) {
      args.id =  argv[++i];
    }else if ((arg == "-o") || (arg == "--add_occluded")) {
      args.add_occluded_points =  true;
    }else if ((arg == "-l") || (arg == "--leaf_size")) {
      args.leaf_size = stof(argv[++i]);
    }else if ((arg == "--remove_ground")) {
      args.remove_ground = true;
    }else if ((arg == "--knn_dist_scan_to_map")) {
      args.knn_dist_scan_to_map = stof(argv[++i]);
    }

  }

  return args;
}

int main (int argc, char* argv[]) 
{

  auto args = parseArgs(argc, argv);

  if(args.align){
    FeaturesGenerator fg;
    fg.setMainDir(args.dataset_dir);
    fg.loadPcdMaps("raw/off_ground");
    fg.loadGroundMaps("raw/ground");
    fg.loadOctomaps("raw/octo");
    fg.setAlignRef();
    fg.alignMaps(args.leaf_size);
  }

  if(args.features_raw){
    FeaturesGenerator fg;
    fg.setMainDir(args.dataset_dir);
    fg.setQueryDpeth(args.query_depth);
    fg.loadPcdMaps("registred/off_ground");
    fg.loadGroundMaps("registred/ground");
    fg.loadOctomaps("registred/octo");  
    fg.extractFeatures(WITHOUT_OCTOMAP);
  }

  if(args.features_octo){
    FeaturesGenerator fg;
    fg.setMainDir(args.dataset_dir);
    fg.setQueryDpeth(args.query_depth);
    fg.loadPcdMaps("registred/off_ground");
    fg.loadGroundMaps("registred/ground"); 
    fg.loadOctomaps("registred/octo"); 
    fg.extractFeatures(WITH_OCTOMAP);
  }

  if(args.scans){
    FeaturesGenerator fg;
    fg.setMainDir(args.dataset_dir);
    fg.loadPcdMaps("labelled/occlusion_free");
    fg.labelScans(args.knn_dist_scan_to_map);
  }

  if(args.remove_dynamics){
    FeaturesGenerator fg;
    fg.setMainDir(args.dataset_dir);
    fg.filterDynamicScans(args.id, args.threshold, args.add_occluded_points, args.remove_ground);
  }

  return 0;
}

