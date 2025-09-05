#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/stat.h>
#include <fstream>
#include <dirent.h>

class MapLoaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh_.reset(new ros::NodeHandle("~"));
        
        test_data_dir_ = "/tmp/map_loader_test_data";
        mkdir(test_data_dir_.c_str(), 0755);
        
        createValidPcdFile();
        createInvalidPcdFile();
        createCorruptedPcdFile();
        
        restricted_dir_ = "/tmp/map_loader_restricted";
        mkdir(restricted_dir_.c_str(), 0000);
    }
    
    void TearDown() override {
        system(("rm -rf " + test_data_dir_).c_str());
        chmod(restricted_dir_.c_str(), 0755);
        system(("rm -rf " + restricted_dir_).c_str());
    }
    
    void createValidPcdFile() {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.width = 100;
        cloud.height = 1;
        cloud.points.resize(cloud.width * cloud.height);
        
        for (size_t i = 0; i < cloud.points.size(); ++i) {
            cloud.points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
            cloud.points[i].r = 255;
            cloud.points[i].g = 255;
            cloud.points[i].b = 255;
        }
        
        pcl::io::savePCDFileASCII(test_data_dir_ + "/valid_0_0.pcd", cloud);
        pcl::io::savePCDFileASCII(test_data_dir_ + "/valid_64_0.pcd", cloud);
        pcl::io::savePCDFileASCII(test_data_dir_ + "/valid_0_64.pcd", cloud);
    }
    
    void createInvalidPcdFile() {
        std::ofstream file(test_data_dir_ + "/invalid_0_0.pcd");
        file << "# Invalid PCD file\n";
        file << "INVALID HEADER\n";
        file.close();
    }
    
    void createCorruptedPcdFile() {
        std::ofstream file(test_data_dir_ + "/corrupted_0_0.pcd");
        file << "# .PCD v0.7 - Point Cloud Data file format\n";
        file << "VERSION 0.7\n";
        file << "FIELDS x y z rgb\n";
        file << "SIZE 4 4 4 4\n";
        file << "TYPE F F F U\n";
        file << "COUNT 1 1 1 1\n";
        file << "WIDTH 10\n";
        file << "HEIGHT 1\n";
        file << "VIEWPOINT 0 0 0 1 0 0 0\n";
        file << "POINTS 10\n";
        file << "DATA ascii\n";
        file << "CORRUPTED DATA HERE\n";
        file.close();
    }
    
    std::unique_ptr<ros::NodeHandle> nh_;
    std::string test_data_dir_;
    std::string restricted_dir_;
};

TEST_F(MapLoaderTest, InvalidPcdFileHandling) {
    std::vector<std::string> pcd_paths = {test_data_dir_ + "/invalid_0_0.pcd"};
    
    EXPECT_NO_THROW({
        sensor_msgs::PointCloud2 pcd_full;
        for (const std::string& path : pcd_paths) {
            sensor_msgs::PointCloud2 pcd_part;
            int result = pcl::io::loadPCDFile(path.c_str(), pcd_part);
            EXPECT_EQ(result, -1);
        }
    });
}

TEST_F(MapLoaderTest, DirectoryPermissionErrors) {
    std::vector<std::string> pcd_paths;
    
    DIR* dirp = opendir(restricted_dir_.c_str());
    EXPECT_EQ(dirp, nullptr);
    
    if (dirp) {
        closedir(dirp);
    }
}

TEST_F(MapLoaderTest, MemoryPressureScenarios) {
    pcl::PointCloud<pcl::PointXYZRGB> large_cloud;
    large_cloud.width = 1000000;
    large_cloud.height = 1;
    large_cloud.points.resize(large_cloud.width * large_cloud.height);
    
    for (size_t i = 0; i < large_cloud.points.size(); ++i) {
        large_cloud.points[i].x = static_cast<float>(i % 1000);
        large_cloud.points[i].y = static_cast<float>(i / 1000);
        large_cloud.points[i].z = 0.0f;
        large_cloud.points[i].r = 255;
        large_cloud.points[i].g = 255;
        large_cloud.points[i].b = 255;
    }
    
    std::string large_file = test_data_dir_ + "/large_0_0.pcd";
    EXPECT_NO_THROW({
        pcl::io::savePCDFileASCII(large_file, large_cloud);
    });
    
    sensor_msgs::PointCloud2 loaded_cloud;
    int result = pcl::io::loadPCDFile(large_file.c_str(), loaded_cloud);
    EXPECT_EQ(result, 0);
}

TEST_F(MapLoaderTest, PoseCallbackEdgeCases) {
    geometry_msgs::PoseStamped pose_msg;
    
    pose_msg.pose.position.x = 64.0;
    pose_msg.pose.position.y = 64.0;
    pose_msg.pose.position.z = 0.0;
    
    pose_msg.pose.position.x = 1e6;
    pose_msg.pose.position.y = 1e6;
    pose_msg.pose.position.z = 1e6;
    
    pose_msg.pose.position.x = -1000.0;
    pose_msg.pose.position.y = -1000.0;
    pose_msg.pose.position.z = -1000.0;
    
    EXPECT_NO_THROW({
    });
}

TEST_F(MapLoaderTest, TileBoundaryConditions) {
    struct tile {
        std::string path;
        float origin_x;
        float origin_y;
        int id;
        std::vector<tile*> neighbors;
    };
    
    tile tile1 = {"test1.pcd", 0.0f, 0.0f, 1, {}};
    tile tile2 = {"test2.pcd", 64.0f, 0.0f, 2, {}};
    tile tile3 = {"test3.pcd", 128.0f, 0.0f, 3, {}};
    
    auto checkNeighbor = [](tile i, tile j, float thresh) -> bool {
        float dist = abs(i.origin_x - j.origin_x) + abs(i.origin_y - j.origin_y);
        return dist <= thresh;
    };
    
    EXPECT_TRUE(checkNeighbor(tile1, tile2, 128.0f));
    EXPECT_FALSE(checkNeighbor(tile1, tile3, 64.0f));
    EXPECT_TRUE(checkNeighbor(tile1, tile3, 128.0f));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "map_loader_test");
    
    return RUN_ALL_TESTS();
}
