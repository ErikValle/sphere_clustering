#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <vector>
#include <string>

int estimateClusterCount(const std::vector<cv::Point3f>& centers) {
    // Calculate pairwise distances between centers
    std::vector<float> distances;
    for (size_t i = 0; i < centers.size(); ++i) {
        for (size_t j = i + 1; j < centers.size(); ++j) {
            float dist = cv::norm(centers[i] - centers[j]);
            distances.push_back(dist);
        }
    }

    // Use a simple heuristic: number of clusters = number of distance "jumps"
    std::sort(distances.begin(), distances.end());
    float mean_dist = std::accumulate(distances.begin(), distances.end(), 0.0f) / distances.size();
    int clusters = 1;

    for (size_t i = 1; i < distances.size(); ++i) {
        if (distances[i] > mean_dist * 1.5) {
            clusters++;
        }
    }

    return clusters;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " input.pcd\n";
        return -1;
    }

    std::string filename = argv[1];

    // Load point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        std::cerr << "Couldn't read PCD file: " << filename << "\n";
        return -1;
    }

    std::cout << "Loaded point cloud with " << cloud->size() << " points.\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
    std::vector<cv::Point3f> sphere_centers;

    // Sphere segmentation loop
    while (true) {
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_SPHERE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.1);
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() < 50) break;

        std::cout << "Detected sphere: center=("
                  << coefficients->values[0] << ", "
                  << coefficients->values[1] << ", "
                  << coefficients->values[2] << "), radius="
                  << coefficients->values[3] << "\n";

        // Store sphere center
        sphere_centers.emplace_back(coefficients->values[0],
                                    coefficients->values[1],
                                    coefficients->values[2]);

        // Remove sphere points
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud_filtered);
    }

    if (sphere_centers.empty()) {
        std::cout << "No spheres detected.\n";
        return 0;
    }

    // Estimate the number of clusters using a heuristic based on spatial distances
    int num_clusters = std::min(
        static_cast<int>(sphere_centers.size()),
        std::max(2, estimateClusterCount(sphere_centers)) // force at least 2 clusters
    );    
    std::cout << "Estimated number of clusters: " << num_clusters << "\n";

    // Clustering using Spectral Clustering (OpenCV)
    cv::Mat centers_mat;
    cv::Mat data(static_cast<int>(sphere_centers.size()), 3, CV_32F);
    for (size_t i = 0; i < sphere_centers.size(); ++i) {
        data.at<float>(i, 0) = sphere_centers[i].x;
        data.at<float>(i, 1) = sphere_centers[i].y;
        data.at<float>(i, 2) = sphere_centers[i].z;
    }

    // Perform Spectral Clustering using OpenCV
    cv::Mat labels;
    cv::Mat similarity_matrix = cv::Mat::zeros(sphere_centers.size(), sphere_centers.size(), CV_32F);

    for (int i = 0; i < data.rows; ++i) {
        for (int j = i + 1; j < data.rows; ++j) {
            float dist = cv::norm(data.row(i) - data.row(j));
            similarity_matrix.at<float>(i, j) = similarity_matrix.at<float>(j, i) = exp(-dist * dist / 0.1);
        }
    }

    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(similarity_matrix, eigenvalues, eigenvectors);

    // Perform k-means clustering on eigenvectors
    cv::Mat centers;
    cv::kmeans(eigenvectors, num_clusters, labels,
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 1.0),
               3, cv::KMEANS_PP_CENTERS, centers);

    // --- Visualization (XY projection) ---
    int image_size = 600;
    int margin = 50;
    cv::Mat canvas(image_size, image_size, CV_8UC3, cv::Scalar(255, 255, 255));

    // Normalize points into canvas coordinates
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;

    for (const auto& p : sphere_centers) {
        min_x = std::min(min_x, p.x);
        max_x = std::max(max_x, p.x);
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }

    float scale_x = (image_size - 2 * margin) / (max_x - min_x + 1e-5f);
    float scale_y = (image_size - 2 * margin) / (max_y - min_y + 1e-5f);

    auto colorForCluster = [](int id) -> cv::Scalar {
        static std::vector<cv::Scalar> palette = {
            {255, 0, 0}, {0, 255, 0}, {0, 0, 255},
            {255, 255, 0}, {255, 0, 255}, {0, 255, 255},
            {128, 0, 0}, {0, 128, 0}, {0, 0, 128},
            {128, 128, 0}, {128, 0, 128}, {0, 128, 128}
        };
        return palette[id % palette.size()];
    };

    // Draw spheres as 2D points
    for (size_t i = 0; i < sphere_centers.size(); ++i) {
        int cluster_id = labels.at<int>((int)i);
        cv::Scalar color = colorForCluster(cluster_id);

        float x = (sphere_centers[i].x - min_x) * scale_x + margin;
        float y = (sphere_centers[i].y - min_y) * scale_y + margin;
        cv::circle(canvas, cv::Point((int)x, (int)y), 6, color, -1);

        // Optionally add cluster number
        cv::putText(canvas, std::to_string(cluster_id), cv::Point((int)x + 8, (int)y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
    }

    cv::line(canvas, cv::Point(margin, image_size - margin), cv::Point(image_size - margin, image_size - margin), cv::Scalar(0, 0, 0), 2); // X-axis
    cv::line(canvas, cv::Point(margin, image_size - margin), cv::Point(margin, margin), cv::Scalar(0, 0, 0), 2); // Y-axis
    cv::putText(canvas, "X", cv::Point(image_size - margin + 10, image_size - margin + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);
    cv::putText(canvas, "Y", cv::Point(margin - 15, margin - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,0), 1);

    for (int i = 1; i <= 4; ++i) {
        int x = margin + i * (image_size - 2 * margin) / 5;
        int y = image_size - margin;
        cv::line(canvas, cv::Point(x, y - 5), cv::Point(x, y + 5), cv::Scalar(0, 0, 0), 1);
        char label[16];
        snprintf(label, sizeof(label), "%.1f", min_x + (max_x - min_x) * i / 5.0f);
        cv::putText(canvas, label, cv::Point(x - 10, y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

        int yt = margin + i * (image_size - 2 * margin) / 5;
        cv::line(canvas, cv::Point(margin - 5, image_size - yt), cv::Point(margin + 5, image_size - yt), cv::Scalar(0, 0, 0), 1);
        snprintf(label, sizeof(label), "%.1f", min_y + (max_y - min_y) * i / 5.0f);
        cv::putText(canvas, label, cv::Point(margin - 45, image_size - yt + 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    }

    cv::putText(canvas, "Sphere Clusters (XY Projection)", cv::Point(margin, margin - 20), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::imwrite("clusters.jpg", canvas); 
    
    return 0;
}
