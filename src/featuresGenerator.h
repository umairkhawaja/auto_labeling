#ifndef _FEATURESGENERATOR_H_
#define _FEATURESGENERATOR_H_

#include <map>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

using namespace std;
using namespace octomap;

class FeaturesGenerator{
public:
  FeaturesGenerator() = default;
  ~FeaturesGenerator() = default;

  using point_t = pcl::PointXYZI;
  using point_cloud_ptr_t = pcl::PointCloud<point_t>::Ptr;

  using tree_t = std::shared_ptr<pcl::KdTreeFLANN<point_t>>;
  using forest_t = map<string, tree_t>;

  enum class E_FeatureType {Max, Median};
  
  void setMainDir(const string& dir);

  void loadPcdMaps(string folder);
  void loadGroundMaps(string folder);
  void loadOctomaps(string folder);
  void extractFeatures(bool use_octomaps=false);

  void printLoadedMaps();

  void setAlignRef();
  void alignMaps(float leaf_size = 0.2f);

  void filterDynamicsMap(string d, const float threshold);
  void setQueryDpeth(int qd) {octree_query_depth_ = qd;};

  void labelScans(const float knn_dist_scan_to_map = 0.3);
  void filterDynamicScans(const string& map_id, float threshold, bool add_occluded_points, bool remove_ground);

  void noiseMedianFilter(string m, int k);  //k is number of neighbood hood points 
  void noiseMedianFilter(point_cloud_ptr_t cloud, int k);

private:
  int octree_query_depth_ = 16;
  string align_ref_;
  string main_dir_;
  forest_t forest_;
  vector<std::string> maps_ids_;
  map<std::string, point_cloud_ptr_t> pcd_maps_;
  map<std::string, point_cloud_ptr_t> ground_maps_;
  map<string, pcl::PointCloud<pcl::PointXYZI>::ConstPtr> clouds_;
  map<string, octomap::OcTree*> octomaps_;

  string extractMapID(string file);
  forest_t createSearchTrees();
  bool isPointOccluded(const pcl::PointXYZI& point, string target_map);
  void printExtractoinState(const string& ref_map);
  float getDistanceToClosestPoint(const point_t& point, const tree_t& tree);
  std::tuple<int, float> getIndexToClosestPoint(const point_t& point, const tree_t& tree);

  void saveLabeledCloud(const string& dir, const string& ref_map, \
                        const vector<float>& cloud_features, const string id="");
  Eigen::Matrix4d icp(point_cloud_ptr_t cloud_source, point_cloud_ptr_t cloud_target);
  void savePCD(const string& map_id, const string& dir);
  void savePCD(const string& map_id, const string& dir, point_cloud_ptr_t cloud);
  void saveOcto(const string& map_id, const string& dir);
  void saveOcto(const string& map_id, octomap::OcTree *transformed_octomap, const string& dir);
  void transformOctomap(const string& octo_id, const Eigen::Matrix4d& transform, const string& dir);
  point_cloud_ptr_t filter(const point_cloud_ptr_t cloud, float leaf_size=0.05);
  void saveASC(const string& dir, point_cloud_ptr_t cloud);

  float getPointFeatureMetric(vector<float>& dist, E_FeatureType ft);
};


#endif