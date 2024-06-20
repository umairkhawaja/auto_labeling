#include "progressBar.h"
#include "getDirFromPath.h"
#include "featuresGenerator.h"
#include "saveLoadMatrix.h"

#include <pcl/console/time.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>


#define GROUND_POINT    (-2)
#define OCCLUDED_POINT  (-1)
#define PRINT_SEPARATOR  printf("\n******************************************************\n")  

inline void toDir(string& f){
  if(f.back() != '/')
    f+='/';
}

static void print4x4Matrix (const Eigen::Matrix4d & matrix) {
  printf("\x1B[34m");
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
  printf("\033[0m");
}

Eigen::Matrix4d FeaturesGenerator::icp(point_cloud_ptr_t cloud_source, point_cloud_ptr_t cloud_target){
  pcl::console::TicToc time;
  int iterations = 300;

  // Defining a rotation matrix and translation vector
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

  // The Iterative Closest Point algorithm
  time.tic ();
  pcl::IterativeClosestPoint<point_t, point_t> icp;
  icp.setMaximumIterations (iterations);
  icp.setInputSource (cloud_source);
  icp.setInputTarget (cloud_target);
  icp.align (*cloud_source);
  cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

  if (icp.hasConverged ())
  {
    cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
    cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);
  }
  else
  {
    PCL_ERROR ("\nICP has not converged.\n");
    exit(EXIT_FAILURE);
  }

  return transformation_matrix;
}

string FeaturesGenerator::extractMapID(string dir){
  auto f = dir.find_last_of('/');
  auto n = dir.find_last_of('.');
  return dir.substr(f+1, n-f-1); 
}

void FeaturesGenerator::setMainDir(const string& dir) {
  this->main_dir_ = dir;
  toDir(this->main_dir_);
  printf("\nMain dir %s\n", this->main_dir_.c_str());
}

void FeaturesGenerator::loadPcdMaps(string f){
  toDir(f);
  const string folder = this->main_dir_ + f;
  const vector<string>& files = getDirFromPath(folder);
  PRINT_SEPARATOR;
  printf("Maps dir: %s\n", folder.c_str());
  pcd_maps_.clear();
  maps_ids_.clear();
  for(const auto& file : files){
    point_cloud_ptr_t cloud (new pcl::PointCloud<point_t>);
    pcl::io::loadPCDFile(file, *cloud);
    string file_name = extractMapID(file);
    pcd_maps_.emplace(file_name, cloud);
    maps_ids_.push_back(file_name);
    printf("pointcloud %s loaded with %zu points\n", file_name.c_str(), cloud->size());
  }

  printf("\nInit kd-tree ... ");
  forest_.clear();
  forest_ = createSearchTrees();
}

void FeaturesGenerator::loadGroundMaps(string f){
  toDir(f);
  const string folder = this->main_dir_ + f;
  const vector<string>& files = getDirFromPath(folder);
  PRINT_SEPARATOR;
  printf("Ground maps dir: %s\n", folder.c_str());
  ground_maps_.clear();
  for(const auto& file : files){
    point_cloud_ptr_t cloud (new pcl::PointCloud<point_t>);
    pcl::io::loadPCDFile(file, *cloud);
    string file_name = extractMapID(file);
    ground_maps_.emplace(file_name, cloud);
    printf("ground pointcloud %s loaded with %zu points\n", file_name.c_str(), cloud->size());
  }
}

void FeaturesGenerator::loadOctomaps(string f){
  toDir(f);
  const string folder = this->main_dir_ + f;
  const vector<string>& files = getDirFromPath(folder);
  PRINT_SEPARATOR;
  octomaps_.clear();
  printf("octomaps dir %s\n", folder.c_str());
  for(const auto& file : files){
    AbstractOcTree* tree = AbstractOcTree::read(file);
    OcTree* octree = dynamic_cast<OcTree*>(tree);
    string file_name = extractMapID(file);
    octomaps_.emplace(file_name, octree);
    printf("Octomap %s loaded with %zu nodes\n", file_name.c_str(), octree->size());
  }
}

FeaturesGenerator::forest_t FeaturesGenerator::createSearchTrees(){
  forest_t forest;
    for(const auto& f : this->maps_ids_){
      auto tree = std::make_shared<pcl::KdTreeFLANN<point_t>>();
      tree->setInputCloud(this->pcd_maps_[f]); 
      forest.emplace(f, tree);
    }
  return forest;
}

bool FeaturesGenerator::isPointOccluded(const point_t& point, string target_map){
  point3d query (point.x, point.y, point.z);
  OcTreeNode* result = this->octomaps_[target_map]->search (query, octree_query_depth_);
  if(result != NULL){ 
    return false;
  }
  return true;
}

void FeaturesGenerator::printExtractoinState(const string& ref_map){
  printf("\nExtracting features for \x1B[34m%s\033[0m  as the reference w.r.t: [ ", ref_map.c_str());
  for(const auto& f : this->maps_ids_){
    if(f == ref_map)
      continue;
    printf("'\x1B[35m%s\033[0m' ", f.c_str());
  }
  printf("] ...\n");
}

float FeaturesGenerator::getDistanceToClosestPoint(const point_t& point, const tree_t& tree){
  const int k = 1;
  vector<int> pointIdxKNNSearch(k);
  vector<float> pointKNNSquaredDistance(k);
  auto num = tree->nearestKSearch(point, k, pointIdxKNNSearch, pointKNNSquaredDistance );
  if(num){
    return pointKNNSquaredDistance[0]*100;
  }  
  return 0;    
}

std::tuple<int, float> FeaturesGenerator::getIndexToClosestPoint(const point_t& point, const tree_t& tree){
  const int k = 1;
  vector<int> pointIdxKNNSearch(k);
  vector<float> pointKNNSquaredDistance(k);
  auto num = tree->nearestKSearch(point, k, pointIdxKNNSearch, pointKNNSquaredDistance );
  if(num){
    return std::make_tuple(pointIdxKNNSearch[0], sqrt(pointKNNSquaredDistance[0])); //this will return the point index puls the distance in meters
  }  
  return std::make_tuple(0, 0.0);    
}

void FeaturesGenerator::extractFeatures(bool use_octomaps){
  PRINT_SEPARATOR;
  string dir = main_dir_ + "labelled/"; 
  dir += use_octomaps? "occlusion_free/" : "occlusion/";
  boost::filesystem::create_directories(dir);

  for(const auto& ref_map : this->maps_ids_){
    vector<float> max_features; //create a features vector to hold all the max dis data
    TProgressBar prog_par;  //create progress bar object 
    prog_par.setSize(this->pcd_maps_[ref_map]->size());

    printExtractoinState(ref_map);

    for(auto point : *this->pcd_maps_[ref_map]){
      vector<float> dis;
      //find the nearest distance with respect to all clouds
      for(const auto& query_map : this->maps_ids_){
        if(query_map == ref_map)
          continue;
        bool occlusded = use_octomaps? isPointOccluded(point, query_map) : false;
        if(!occlusded){
          auto d = getDistanceToClosestPoint(point, this->forest_[query_map]);
          dis.push_back(d);
        }
        else {
          dis.push_back(OCCLUDED_POINT);
        }
      }
      float max = *max_element(dis.begin(), dis.end());

      // map the feature distance into exp space in order to bound it between 0 and 1
      if(max != OCCLUDED_POINT){
        max = 1 - exp(- max * (max / 100)); 
      }

      max_features.push_back(max);
      prog_par.update();
    }
    prog_par.end();
    saveLabeledCloud(dir, ref_map, max_features);
  }
}

void FeaturesGenerator::saveLabeledCloud(const string& dir, const string& ref_map, \
                      const vector<float>& cloud_features, const string id){
  int index = 0;
  point_cloud_ptr_t cloud(new pcl::PointCloud<point_t>);
  
  for(auto point : *this->pcd_maps_[ref_map]){
    point.intensity = cloud_features[index];
    cloud->push_back(point);
    index++;    //update index
  }

  //I found that it is good idea to perform the median filter at this stage! 
  cout << "Filter labels noise using Median filter [k=512] ...\n"; 
  noiseMedianFilter(this->pcd_maps_[ref_map], 512);

  // merge Ground
  if(!ground_maps_.empty()){ 
    for(auto point : *this->ground_maps_[ref_map]){
      point.intensity = 0.0;
      cloud->push_back(point);
    }
  }

  string s = dir + ref_map + id + ".pcd";
  cout << "saving " << s << " ...\n";
  pcl::io::savePCDFile(s, *cloud);     
}

void FeaturesGenerator::printLoadedMaps(){
  PRINT_SEPARATOR;
  printf("Total number of maps: %zu\n", this->pcd_maps_.size());
  printf("Maps IDs: \n");
  for(const auto& m : this->maps_ids_){
    printf("*) %s\n", m.c_str());
  }  
}

void FeaturesGenerator::setAlignRef(){
  PRINT_SEPARATOR;

  printf("Maps IDs: \n");
  for(const auto& m : this->maps_ids_){
    printf("\x1B[33m%s\033[0m\t", m.c_str());
  }  

  string s;
  cout << "\n\033[3;100;30mPlease enter reference map for alignment:\033[0m ";
  cout << "\x1B[32m";
  cin >> s;
  cout << "\033[0m";

  //check if the reference map exist in the maps ids
  auto in_ids = std::find(this->maps_ids_.begin(), this->maps_ids_.end(), s) != this->maps_ids_.end();
  if(!in_ids){
    cout << "are you sure!!\n";
    exit(EXIT_FAILURE);
  }

  printf("Align reference map is: \x1B[32m%s\033[0m\n", s.c_str());

  this->align_ref_ = s;
}

// it is good idea to subsample the maps before doing the alignment, that is mainly to save time
// after we got the transformation matix, we can apply it to the original cloud. 
void FeaturesGenerator::alignMaps(float leaf_size){
  PRINT_SEPARATOR;

  string registred_dir = main_dir_ + "registred/";
  string transform_dir = registred_dir + "transform/";
  string octomaps_dir = registred_dir + "octo/";
  string pointcloud_dir = registred_dir + "off_ground/";
  string ground_dir = registred_dir + "ground/";
  boost::filesystem::create_directories(transform_dir);
  boost::filesystem::create_directories(octomaps_dir);
  boost::filesystem::create_directories(pointcloud_dir);
  boost::filesystem::create_directories(ground_dir);

  printf("Subsample leaf size = %f\n", leaf_size);

  // a. subsample the reference cloud 
  printf("Subsampling the reference map %s ... \n", align_ref_.c_str());
  auto ref_map_supsampled = leaf_size == 0.0 ? pcd_maps_[align_ref_] : filter(pcd_maps_[align_ref_], leaf_size);

  // b. start the alignment process
  for(const auto& map_to_align : maps_ids_){
    if(map_to_align == align_ref_){ 
      saveMatrix(transform_dir+map_to_align, Eigen::Matrix4d::Identity ());
      savePCD(map_to_align, pointcloud_dir);
      savePCD(map_to_align, ground_dir, this->ground_maps_[map_to_align]);
      saveOcto(align_ref_, octomaps_dir);
      continue;
    }

    printf("Aligning \x1B[31m%s\033[0m w.r.t \x1B[32m%s\033[0m ... \n", map_to_align.c_str(), align_ref_.c_str());
    auto map_to_align_supsampled = leaf_size == 0.0 ? pcd_maps_[map_to_align] : filter(pcd_maps_[map_to_align], leaf_size);
    auto transform = icp(map_to_align_supsampled, ref_map_supsampled);
    pcl::transformPointCloud(*pcd_maps_[map_to_align],*pcd_maps_[map_to_align],transform);
    pcl::transformPointCloud(*ground_maps_[map_to_align],*ground_maps_[map_to_align],transform);
    saveMatrix(transform_dir+map_to_align, transform);
    savePCD(map_to_align, pointcloud_dir);
    savePCD(map_to_align, ground_dir, this->ground_maps_[map_to_align]);
    transformOctomap(map_to_align, transform, octomaps_dir);
    printf("\n");
  }
}

void FeaturesGenerator::savePCD(const string& map_id, const string& dir){
  printf("Saving transformed map %s into %s ...\n", map_id.c_str(), dir.c_str());
  pcl::io::savePCDFile(dir+map_id+".pcd", *this->pcd_maps_[map_id]);
}

void FeaturesGenerator::savePCD(const string& map_id, const string& dir, point_cloud_ptr_t cloud){
  printf("Saving transformed map %s into %s ...\n", map_id.c_str(), dir.c_str());
  pcl::io::savePCDFile(dir+map_id+".pcd", *cloud);
}

void FeaturesGenerator::saveOcto(const string& octo_id, const string& dir){
  printf("Saving transformed octomap %s into %s ...\n", octo_id.c_str(), dir.c_str());
  octomaps_[octo_id]->write(dir+octo_id+".ot");
}

void FeaturesGenerator::saveOcto(const string& octo_id, octomap::OcTree *transformed_octomap, const string& dir){
  printf("Saving transformed octomap into %s ...\n", dir.c_str());
  transformed_octomap->write(dir+octo_id+".ot");
}

void FeaturesGenerator::transformOctomap(const string& octo_id, const Eigen::Matrix4d& transform, const string& dir){
  auto resolution = octomaps_[octo_id]->getResolution();
  octomap::OcTree *transformed_octomap = new octomap::OcTree(resolution);

  pcl::PointXYZ p_in;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_in->push_back(p_in);

  printf("Transform %s octomap ...\n", octo_id.c_str());

  TProgressBar pb;
  pb.setSize(octomaps_[octo_id]->size());

  for(OcTree::tree_iterator it = octomaps_[octo_id]->begin_tree(),
        end=octomaps_[octo_id]->end_tree(); it!= end; ++it) {
    pcl::PointXYZ p_in(it.getCoordinate().x(),it.getCoordinate().y(),it.getCoordinate().z());
    (*cloud_in)[0] = p_in;
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);
    auto p_out = (*cloud_out)[0];
    point3d new_centroid (p_out.x, p_out.y, p_out.z);
    transformed_octomap->updateNode(new_centroid, it->getOccupancy() > 0.6);
    pb.update();
  }
  pb.end();

  saveOcto(octo_id, transformed_octomap, dir);

}

void FeaturesGenerator::filterDynamicsMap(string d, const float threshold){
  toDir(d);
  const string folder = this->main_dir_ + d;
  const vector<string>& files = getDirFromPath(folder);
  PRINT_SEPARATOR;
  for(const auto& file : files){
    pcl::PointCloud<point_t>::Ptr cloud (new pcl::PointCloud<point_t>);
    pcl::PointCloud<point_t>::Ptr cloud_filtered (new pcl::PointCloud<point_t>);
    pcl::io::loadPCDFile(file, *cloud);
    string file_name = extractMapID(file);
    printf("%s loaded with %zu points\n", file_name.c_str(), cloud->size());
    for(const auto& p : *cloud){
      if(p.intensity <= threshold)
        cloud_filtered->push_back(p);
    }
    printf("cloud_filtered %zu points\n", cloud_filtered->size());
    string new_file = file_name + "_filtered.pcd";
    cout << new_file << endl;
    pcl::io::savePCDFile(new_file, *cloud_filtered);
  }
}


FeaturesGenerator::point_cloud_ptr_t FeaturesGenerator::filter(const point_cloud_ptr_t cloud, float leaf_size){
  point_cloud_ptr_t cloud_filtered (new pcl::PointCloud<point_t>);
  
  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<point_t> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (leaf_size, leaf_size, leaf_size);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  return cloud_filtered;
}

void FeaturesGenerator::labelScans(const float knn_dist_scan_to_map){
  auto debug_dir = main_dir_ + "debug/";
  boost::filesystem::remove_all(debug_dir);
  boost::filesystem::create_directories(debug_dir);

  for(const auto m : this->maps_ids_){

    PRINT_SEPARATOR;

    std::cout << "Creating labelled scans for " << m << std::endl;

    //directory stuff
    auto raw_scans_dir = main_dir_ + "raw/scans/" + m + "/";
    auto labelled_scans_dir = main_dir_ + "labelled"+"/scans/" + m + "/";
    boost::filesystem::remove_all(labelled_scans_dir);
    boost::filesystem::create_directories(labelled_scans_dir);

    //get map transform 
    auto map_transform_pth = main_dir_ + "registred/transform/" + m; 
    Eigen::Matrix4d map_transform = openMatrix(map_transform_pth);

    cout << "raw scans dir: " << raw_scans_dir << endl;
    const vector<string>& scans = getDirFromPath(raw_scans_dir);

    //get transformation matrices for the scans
    auto scans_transform_dir = main_dir_ + "raw/trajectories/" + m + "/";
    const vector<string>& scans_transforms = getDirFromPath(scans_transform_dir);

    //find the number of scans the do not have associated transformation matrix 
    int scans_start_idx = scans.size() - scans_transforms.size();
    cout << "number of frames to skip is: " << scans_start_idx << endl;
    cout << "labelling " << scans_transforms.size() << " scans ...\n";

    #pragma omp parallel
    { 
      point_cloud_ptr_t cloud_debug (new pcl::PointCloud<point_t>);

      #pragma omp for
      for(size_t idx = 0; idx < scans_transforms.size(); idx++)
      {
        //create cloud instances for the raw and transformed scan
        point_cloud_ptr_t cloud (new pcl::PointCloud<point_t>);
        point_cloud_ptr_t cloud_tr (new pcl::PointCloud<point_t>);   

        //load lidar scan into cloud instance and get its ID
        auto& scan_pth = scans[scans_start_idx + idx];
        pcl::io::loadPCDFile(scan_pth, *cloud);
        auto scan_id = extractMapID(scan_pth);

        //transform the scan to the correct coordinate in the labelled map
        Eigen::Matrix4d scan_transform = openMatrix(scans_transforms[idx]);
        pcl::transformPointCloud(*cloud, *cloud_tr, scan_transform);    
        pcl::transformPointCloud(*cloud_tr, *cloud_tr, map_transform);

        for(int p_idx = 0; p_idx < cloud_tr->size(); p_idx++)
        {
          auto [idx, distance] = getIndexToClosestPoint((*cloud_tr)[p_idx], forest_[m]);
          if(distance <= knn_dist_scan_to_map)
            (*cloud)[p_idx].intensity = (*cloud_tr)[p_idx].intensity = (*pcd_maps_[m])[idx].intensity;
          else
            (*cloud)[p_idx].intensity = (*cloud_tr)[p_idx].intensity = 1; //consider point as dunamic point 
          cloud_debug->push_back((*cloud_tr)[p_idx]);
        }

        saveASC(labelled_scans_dir+scan_id+".asc", cloud);
      }
      auto debug_pth = "/tmp/"+m+std::to_string(omp_get_thread_num())+".pcd";
      pcl::io::savePCDFileBinary(debug_pth, *cloud_debug);
    }

    cout << "\n"; 
  }

}


void FeaturesGenerator::filterDynamicScans(const string& m, float threshold, bool add_occluded_points, bool remove_ground){
  PRINT_SEPARATOR;

  auto labelled_scans_dir = main_dir_ + "labelled/scans/" + m + "/";
  auto filtred_scans_dir = main_dir_ + "labelled/filtred_scans/" + m + "/";
  
  boost::filesystem::remove_all(filtred_scans_dir);
  boost::filesystem::create_directories(filtred_scans_dir);

  cout << "filtred scans dir: " << filtred_scans_dir << endl;
  const vector<string>& scans = getDirFromPath(labelled_scans_dir);

  int counter = 0;
  for(const auto& scan : scans){
    point_cloud_ptr_t cloud (new pcl::PointCloud<point_t>);
    point_cloud_ptr_t cloud_filtred (new pcl::PointCloud<point_t>);
    pcl::io::loadPCDFile(scan, *cloud);
    auto file_name = extractMapID(scan);
    for(auto& point : *cloud){
      if(point.intensity == GROUND_POINT && remove_ground)
        continue;
      if(point.intensity == OCCLUDED_POINT && !add_occluded_points)
        continue;
      if(point.intensity > threshold )
        continue;
      cloud_filtred->push_back(point);
    }
    printf("lidar scan %s before filter %zu points, aftre filter %zu points [%d/%d]\n", file_name.c_str(), cloud->size(), cloud_filtred->size(),++counter,scans.size());
    pcl::io::savePCDFile(filtred_scans_dir+file_name+".pcd", *cloud_filtred);
  }
}

void FeaturesGenerator::saveASC(const string& dir, point_cloud_ptr_t cloud){
  std::ofstream ofs (dir, std::ofstream::out);
  for(const auto& p : *cloud){
    ofs << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << endl;
  }
}

void FeaturesGenerator::noiseMedianFilter(string m, int k){
  point_cloud_ptr_t cloud (new pcl::PointCloud<point_t>);

  //load targeted map
  pcl::io::loadPCDFile(m, *cloud);

  noiseMedianFilter(cloud, k);

  //new map id + save the map
  auto m_f = m.substr(0, m.size()-4) + "_filtered.pcd";
  pcl::io::savePCDFileBinary(m, *cloud);
}

float FeaturesGenerator::getPointFeatureMetric(vector<float>& dis, E_FeatureType ft){
  switch (ft)
  {
  case E_FeatureType::Max:
    return *max_element(dis.begin(), dis.end());
  
  case E_FeatureType::Median:
    auto dis_size = dis.size();

    //if the distance vector has only two elements or less, we can not
    //find the median, therefore in this case we return the max of the 
    //vector! OR if the vector size is 3 and 2/3 of the points occluded
    //we return the maximum value! 
    if(dis_size <= 2)
      return getPointFeatureMetric(dis, E_FeatureType::Max);

    if(dis[dis_size/2] == OCCLUDED_POINT || dis[dis_size/2 -1] == OCCLUDED_POINT)
      return getPointFeatureMetric(dis, E_FeatureType::Max);

    sort(dis.begin(), dis.end());
    if ( dis_size / 2 == 0){
      return (dis[dis_size/2] + dis[dis_size/2 -1]) / 2.0 ;
    }
    return dis[((int)dis_size/2)];
  }
}

void FeaturesGenerator::noiseMedianFilter(point_cloud_ptr_t cloud, int k){
  //make it searchable 
  auto tree = std::make_shared<pcl::KdTreeFLANN<point_t>>();
  tree->setInputCloud(cloud); 

  vector<float> filtered_labels;
  filtered_labels.resize(cloud->size());

  //itterate through all the points
  #pragma omp parallel for
  for(size_t i = 0; i < cloud->size(); i++){
    const auto point = cloud->at(i);
    vector<int> pointIdxKNNSearch(k);
    vector<float> pointKNNSquaredDistance(k);
    auto num = tree->nearestKSearch(point, k, pointIdxKNNSearch, pointKNNSquaredDistance );

    vector<float> pointIntensity;
    for (std::size_t i = 0; i < pointIdxKNNSearch.size (); i++){
      pointIntensity.push_back((*cloud)[ pointIdxKNNSearch[i] ].intensity);
    }

    filtered_labels[i] = getPointFeatureMetric(pointIntensity, E_FeatureType::Median);
  }

  //copy the labels to the cloud, this process should be done seperatly as not to affect the original 
  //labels of the cloud
  #pragma omp parallel for
  for(size_t i = 0; i < cloud->size(); i++){
    cloud->at(i).intensity = filtered_labels[i];
  }
}